#include "fft_process.h"
#include <cmath>
#include <numeric>
#include <algorithm>
#include <fftw3.h>
#include "../sdr-bridge-internal.h"

// FFTProcessor Constructor
FFTProcessor::FFTProcessor() {
    // Initialize vectors that depend on runtime sizes
    thresholdBuffer.assign(confirmation + 1, 0.0f);
}

FFTProcessor::~FFTProcessor() {
    if (circularBuffer) {
        fftwf_free(circularBuffer);
        circularBuffer = nullptr;
    }
}

void FFTProcessor::configure(const FftProcessorConfig& config) {
    config_ = config;
    // Any re-initialization based on new config can go here
    // For example, if circularBuffer size depends on samplesPerReading, it might need re-allocating
    if (currentSampCount != config_.samplesPerReading) { // Or a more robust check based on actual buffer usage
        if (circularBuffer) {
            fftwf_free(circularBuffer);
            circularBuffer = nullptr;
        }
        currentSampCount = config_.samplesPerReading;
        circularBuffer = fftwf_alloc_complex(currentSampCount * integrationPeriod);
    }

    if (maxPeakAndFrequency.empty()) {
        maxPeakAndFrequency = {-130.0f, static_cast<float>(config_.centerFrequency)};
    }
    if (peakBuffer.empty()) {
        peakBuffer.assign(peakRemanance, -130.0);
    }
    if (peakNormalizedBuffer.empty()) {
        peakNormalizedBuffer.assign(peakRemanance, 0.0);
    }
    if (signalTimeTable.empty()) {
        signalTimeTable.push_back(std::chrono::steady_clock::now());
        signalTimeTable.push_back(std::chrono::steady_clock::now());
    }
    if (signalStrengthIndexBuffer.empty()) {
        signalStrengthIndexBuffer.assign(signalStrengthIndexRemanance, 0);
    }
}


void FFTProcessor::process(const std::complex<float> *input_buf, uint32_t input_len) {
    uint32_t sampCount = input_len;

    // Allocate FFTW buffers
    fftwf_complex *signal = fftwf_alloc_complex(sampCount);
    fftwf_complex *fft_signal = fftwf_alloc_complex(sampCount);

    // --- 1. I/Q Copy and Circular Buffer Update ---
    // If currentSampCount needs to change dynamically, handle re-allocation here.
    // Assuming currentSampCount is set via configure and remains stable during processing.
    if (currentSampCount != sampCount) {
        if (circularBuffer) {
            fftwf_free(circularBuffer);
            circularBuffer = nullptr; // Ensure it's null before re-allocating
        }
        currentSampCount = sampCount;
        circularBuffer = fftwf_alloc_complex(currentSampCount * integrationPeriod);
    }


    for (uint32_t i = 0; i < sampCount; i++) {
        signal[i][0] = input_buf[i].real();
        signal[i][1] = input_buf[i].imag();

        if (circularBuffer) { // Check if circularBuffer is valid
            circularBuffer[(bufferIndex * sampCount) + i][0] = signal[i][0];
            circularBuffer[(bufferIndex * sampCount) + i][1] = signal[i][1];
        }
    }

    bufferIndex = (bufferIndex + 1) % integrationPeriod;
    integrationCount++;


    // --- 2. Perform FFT ---
    fftwf_plan plan1 = fftwf_plan_dft_1d(sampCount, signal, fft_signal, FFTW_FORWARD, FFTW_ESTIMATE);
    fftwf_execute(plan1);
    fftwf_destroy_plan(plan1); // Clean up plan


    // --- 3. Compute Power Spectrum ---
    std::vector<float> power(sampCount);
    for (uint32_t i = 0; i < sampCount; i++) {
        power[i] = fft_signal[i][0] * fft_signal[i][0] + fft_signal[i][1] * fft_signal[i][1];
    }
    fftwf_free(signal);
    fftwf_free(fft_signal);


    // --- 4. Shift Power Spectrum ---
    power_shifted_vec.resize(sampCount); // Ensure output vector has correct size
    int half = sampCount / 2;
    for (int i = 0; i < half; i++) {
        power_shifted_vec[i] = power[i + half]; // Negative frequencies
        power_shifted_vec[i + half] = power[i]; // Positive frequencies
    }


    // --- 5. Evaluate Signal Strength ---
    // This part requires access to config_ and updates member variables
    evaluateSignalStrength(sampCount, power_shifted_vec.data(), config_.sampleRate, config_.centerFrequency);

    // After this, getters can be used to retrieve results like peakDb, peakNormalized, etc.
}

// Helper function definitions
void FFTProcessor::computePowerSpectrum(fftwf_complex *fft_signal, uint32_t sampCount, float* power_out) {
    for (uint32_t i = 0; i < sampCount; i++) {
        power_out[i] = fft_signal[i][0] * fft_signal[i][0] + fft_signal[i][1] * fft_signal[i][1];
    }
}

void FFTProcessor::shiftPowerSpectrum(const float* power_in, uint32_t sampCount, float* power_shifted_out) {
    int half = sampCount / 2;
    for (int i = 0; i < half; i++) {
        power_shifted_out[i] = power_in[i + half]; // Negative frequencies
        power_shifted_out[i + half] = power_in[i]; // Positive frequencies
    }
}

void FFTProcessor::evaluateSignalStrength(uint32_t sampCount, const float* power_shifted, uint32_t sampleRate, uint32_t centerFrequency) {

    // --- 6.1 BANDWITH definition for PEAK EVALUATION ---
    float freqPerBin = static_cast<float>(sampleRate) / sampCount;  // Hz per bin
    float lowerWWBound = centerFrequency - config_.freqFocusRangeKhz * 1000.0f;
    float upperWWBound = centerFrequency + config_.freqFocusRangeKhz * 1000.0f;

    int lowerWWIndex = static_cast<int>((lowerWWBound - (centerFrequency - sampleRate / 2)) / freqPerBin);
    int upperWWIndex = static_cast<int>((upperWWBound - (centerFrequency - sampleRate / 2)) / freqPerBin);
    lowerWWIndex = std::max(0, lowerWWIndex);
    upperWWIndex = sampCount - 1 > upperWWIndex ? upperWWIndex : sampCount - 1;
    int WW_size = upperWWIndex - lowerWWIndex + 1;

    // --- 6.2 INITIALISATION of variables for SIGNAL PEAK MEASURE ---
    float totalPower = 0.0f;
    this->peakDb = -130.0f; // Reset for current processing block
    int index_peak = 0;
    std::vector<float> power_shifted_DB(WW_size);

    if (peakBuffer.empty()) {
        peakBuffer.assign(peakRemanance, -130.0);
    }
    if (peakNormalizedBuffer.empty()) {
        peakNormalizedBuffer.assign(peakRemanance, 0.0);
    }

    // --- 6.3 SIGNAL PEAK MEASURE ---
    for (int i = lowerWWIndex; i <= upperWWIndex; i++) {
        float dB = 10 * log10(power_shifted[i] / refPower);
        if (dB > this->peakDb) {
            this->peakDb = dB;
            index_peak = i - lowerWWIndex;
        }
        power_shifted_DB[i - lowerWWIndex] = dB;
    }

    peakBuffer[indexPeakBuffer] = this->peakDb;

    // --- 6.4 NOISE EVALUATION ---
    std::sort(power_shifted_DB.begin(), power_shifted_DB.end());
    float noiseMedian = power_shifted_DB[WW_size / 2];

    std::vector<float> gapTable(WW_size);
    for (int i = 0; i < WW_size; i++) {
        gapTable[i] = std::fabs(power_shifted_DB[i] - noiseMedian);
    }
    std::sort(gapTable.begin(), gapTable.end());
    float noiseSigma = 1.4816f * gapTable[WW_size / 2];

    // --- 6.5 SIGNAL PEAK NORMALIZED MEASURE ---
    this->peakNormalized = this->peakDb - noiseMedian;
    peakNormalizedBuffer[indexPeakBuffer] = this->peakNormalized;
    indexPeakBuffer = (indexPeakBuffer + 1) % peakRemanance;

    thresholdBuffer[indexThreshold] = this->peakNormalized / noiseSigma;
    if (this->peakNormalized > peakNormalizedMax) {
        peakNormalizedMax = this->peakNormalized;
    }
    if (thresholdBuffer[indexThreshold] > maxThreshold) {
        maxThreshold = thresholdBuffer[indexThreshold];
    }
    float minVal = *std::min_element(thresholdBuffer.begin(), thresholdBuffer.end());
    if (minVal > maxThresholdConfirmed) {
        maxThresholdConfirmed = minVal;
    }
    indexThreshold = (indexThreshold + 1) % (confirmation + 1);

    // --- 6.6 INIT of tracking frequency ---
    if (trackingFrequency == 0.0f) {
        trackingFrequency = static_cast<float>(centerFrequency);
    }

    if (sdr_bridge_internal::isCenterFrequencyChanged) {
        trackingFrequency = centerFrequency ;
        sdr_bridge_internal::isCenterFrequencyChanged = false ;
    }

    // For now, assume trackingFrequency is updated only when explicitly told to.
    if (maxPeakAndFrequency.empty()) {
        maxPeakAndFrequency = {-130.0f, static_cast<float>(centerFrequency)};
    }


    // --- 6.7 DEFINITION OF SIGNAL DETECTED ---
    int signalEval = 0;
    loopNB++;

    if (this->peakDb > noiseMedian + 4.0 * noiseSigma) {
        if (peakConfirmed >= confirmation) {
            if (this->peakDb > maxPeakAndFrequency[0]) {
                maxPeakAndFrequency[0] = this->peakDb;
                maxPeakAndFrequency[1] = index_peak * freqPerBin + lowerWWBound;
                timeOfLastMaxPeak = std::chrono::steady_clock::now();
            }
            signalEval = signalStrong + 1;
            lvl1NB++;
            lvl1Ratio = static_cast<float>(lvl1NB) / loopNB;
        } else {
            peakConfirmed++;
        }
    } else {
        peakConfirmed = 0;
    }

    auto now = std::chrono::steady_clock::now();
    auto delaySincePeak = std::chrono::duration_cast<std::chrono::milliseconds>(now - timeOfLastMaxPeak);
    auto delaySincePeak_ms = delaySincePeak.count();

    // TODO : triggered even if no peak! To be investigated
    if (timeOfLastMaxPeakUpdate < timeOfLastMaxPeak && delaySincePeak_ms > 300) {
        trackingFrequency = maxPeakAndFrequency[1];
        timeOfLastMaxPeakUpdate = std::chrono::steady_clock::now();
        maxPeakAndFrequency[0] = -130.0f;
    }

    if (signalTimeTable.empty()) {
        signalTimeTable.push_back(std::chrono::steady_clock::now());
        signalTimeTable.push_back(std::chrono::steady_clock::now());
    }

    int currentSignalStrengthIndex = 0;

    if (remananceStrong > 0) {
        currentSignalStrengthIndex = 3;
        remananceStrong = std::max(0, remananceStrong - 1);
        remananceMedium = std::max(0, remananceMedium - 1);
        remananceWeak = std::max(0, remananceWeak - 1);
    } else {
        auto duration1 = std::chrono::steady_clock::now() - signalTimeTable[0];
        auto duration2 = signalTimeTable[0] - signalTimeTable[1];
        auto duration1_ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration1).count();
        auto duration2_ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration2).count();
        long long abs_diff = std::abs(duration1_ms - duration2_ms);

        if (signalEval >= signalStrong ||
            (signalMedium <= signalEval && signalEval < signalStrong && abs_diff < 300)) {
            currentSignalStrengthIndex = 3;
            remananceStrong = 3;
            if (duration1_ms > 666) {
                signalTimeTable.push_back(std::chrono::steady_clock::now());
            }
        } else {
            if (remananceMedium > 0) {
                currentSignalStrengthIndex = 2;
                remananceMedium = std::max(0, remananceMedium - 1);
                remananceWeak = std::max(0, remananceWeak - 1);
            } else {
                duration1 = std::chrono::steady_clock::now() - signalTimeTable[0];
                duration2 = signalTimeTable[0] - signalTimeTable[1];
                duration1_ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration1).count();
                duration2_ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration2).count();
                abs_diff = std::abs(duration1_ms - duration2_ms);

                if (signalEval >= signalMedium ||
                    (signalWeak <= signalEval && signalEval < signalMedium && abs_diff < 300)) {
                    currentSignalStrengthIndex = 2;
                    remananceMedium = 2;
                    if (duration1_ms > 666) {
                        signalTimeTable.push_back(std::chrono::steady_clock::now());
                    }
                } else {
                    if (remananceWeak > 0) {
                        currentSignalStrengthIndex = 1;
                        remananceWeak = std::max(0, remananceWeak - 1);
                    } else {
                        if (signalEval >= signalWeak) {
                            remananceWeak = 1;
                            currentSignalStrengthIndex = 1;
                            if (duration1_ms > 666) {
                                signalTimeTable.push_back(std::chrono::steady_clock::now());
                            }
                        } else {
                            remananceStrong = std::max(0, remananceStrong - 1);
                            remananceMedium = std::max(0, remananceMedium - 1);
                            remananceWeak = std::max(0, remananceWeak - 1);
                        }
                    }
                }
            }
        }
    }

    if (signalStrengthIndexBuffer.empty()) {
        signalStrengthIndexBuffer.assign(signalStrengthIndexRemanance, 0);
    }
    signalStrengthIndexBuffer[indexsignalStrengthIndexBuffer] = currentSignalStrengthIndex;
    indexsignalStrengthIndexBuffer = (indexsignalStrengthIndexBuffer + 1) % signalStrengthIndexRemanance;

    auto signalStrengthIndexMaxIter = std::max_element(signalStrengthIndexBuffer.begin(), signalStrengthIndexBuffer.end());
    this->signalStrengthIndexSent = *signalStrengthIndexMaxIter;

    int sum = std::accumulate(signalStrengthIndexBuffer.begin(), signalStrengthIndexBuffer.end(), 0);
    double signalStrengthIndexMean = static_cast<double>(sum) / signalStrengthIndexBuffer.size();
    if (*signalStrengthIndexMaxIter == 1 && signalStrengthIndexMean < 0.8) {
        this->signalStrengthIndexSent = 0;
    }
}