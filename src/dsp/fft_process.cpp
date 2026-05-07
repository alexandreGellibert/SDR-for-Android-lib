#include "fft_process.h"
#include <cmath>
#include <numeric>
#include <algorithm>
#include <fftw3.h>
#include "../sdr-bridge-internal.h"

// FFTProcessor Constructor
FFTProcessor::FFTProcessor() {
    detectionFlagBuffer.assign(detectionFlagRemanance, 0);
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
    if (detectionFlagBuffer.empty()) {
        detectionFlagBuffer.assign(detectionFlagRemanance, 0);
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

    // After this, getters can be used to retrieve results like peakDb, meanSnrSigma, etc.
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

void FFTProcessor::evaluateSignalStrength(uint32_t sampCount, const float* power_shifted,
                                           uint32_t sampleRate, uint32_t centerFrequency) {

    const float freqPerBin = static_cast<float>(sampleRate) / static_cast<float>(sampCount);
    const float X_hz       = config_.freqFocusRangeKhz * 1000.0f;  // focus half-width in Hz
    const float nyquist    = sampleRate / 2.0f;                     // max offset from centre

    // offToBin: signed frequency offset from centre → bin index in shifted spectrum
    // (bin 0 = centre − nyquist, bin sampCount/2 = centre, bin sampCount−1 = centre + nyquist − freqPerBin)
    auto offToBin = [&](float offset_hz) -> int {
        return static_cast<int>((offset_hz + nyquist) / freqPerBin);
    };

    // ── 6.1  Focus window ─────────────────────────────────────────────────────
    const int focusLo  = std::max(0, offToBin(-X_hz));
    const int focusHi  = std::min(static_cast<int>(sampCount) - 1, offToBin(+X_hz) - 1);
    const int focusLen = focusHi - focusLo + 1;
    if (focusLen <= 0) return;

    // ── 6.2  Signal band: mean power + absolute peak bin (for frequency tracking) ──
    float absPeakDb        = -130.0f;
    int   peakBinInFocus   = 0;
    float signalPowerSum   = 0.0f;

    for (int i = focusLo; i <= focusHi; i++) {
        float p  = power_shifted[i];
        signalPowerSum += p;
        float dB = 10.0f * log10f(p / refPower + 1e-20f);
        if (dB > absPeakDb) {
            absPeakDb      = dB;
            peakBinInFocus = i - focusLo;
        }
    }
    const float signalPowerDb = 10.0f * log10f((signalPowerSum / focusLen) / refPower + 1e-20f);

    // ── 6.3  Spatial reference windows (OS-CFAR style) ────────────────────────
    // Each window stores its mean dB AND its bin bounds so we can later pool
    // the individual bins of the quiet (bottom-40%) windows to estimate σ_bin.
    const int winBins1k = std::max(1, static_cast<int>(std::ceil(1000.0f / freqPerBin)));

    // Helper: best 1 kHz sliding-window mean (linear) over [lo..hi]
    auto best1kHzMean = [&](int lo, int hi) -> float {
        const int len = hi - lo + 1;
        if (len <= 0) return 0.0f;
        if (len < winBins1k) {
            float s = 0.0f;
            for (int i = lo; i <= hi; i++) s += power_shifted[i];
            return s / len;
        }
        float runSum = 0.0f;
        for (int i = lo; i < lo + winBins1k; i++) runSum += power_shifted[i];
        float best = runSum / winBins1k;
        for (int start = lo + 1; start + winBins1k - 1 <= hi; start++) {
            runSum += power_shifted[start + winBins1k - 1] - power_shifted[start - 1];
            const float m = runSum / winBins1k;
            if (m > best) best = m;
        }
        return best;
    };

    struct RefWindow {
        float meanDb;
        float maxBinDb;
        float best1kDb;
        int lo, hi;
    };
    std::vector<RefWindow> refWindows;
    refWindows.reserve(10);

    for (int k = 1; k <= 5; k++) {
        const float nearX = (4 * k - 2) * X_hz;
        const float farX  =  4 * k      * X_hz;
        if (farX >= nyquist) break;

        auto collectWindow = [&](int lo, int hi) {
            if (hi <= lo) return;
            const int n = hi - lo + 1;
            float sum = 0.0f, maxP = 0.0f;
            for (int i = lo; i <= hi; i++) {
                sum += power_shifted[i];
                if (power_shifted[i] > maxP) maxP = power_shifted[i];
            }
            refWindows.push_back({
                10.0f * log10f((sum / n)                    / refPower + 1e-20f),
                10.0f * log10f(maxP                          / refPower + 1e-20f),
                10.0f * log10f(best1kHzMean(lo, hi)          / refPower + 1e-20f),
                lo, hi
            });
        };

        collectWindow(std::max(0, offToBin(+nearX)),
                      std::min(static_cast<int>(sampCount) - 1, offToBin(+farX) - 1));
        collectWindow(std::max(0, offToBin(-farX)),
                      std::min(static_cast<int>(sampCount) - 1, offToBin(-nearX) - 1));
    }

    const int nRef  = static_cast<int>(refWindows.size());
    const bool valid = (nRef >= 2);
    if (!valid) {
        this->meanSnrDb = this->meanSnrSigma = 0.0f;
        peakAboveNoiseMeanDb = maxBinSnrDb = maxBinSnrSigma = best1kHzSnrDb = best1kHzSnrSigma = 0.0f;
        // (frequency tracking below still runs)
        goto freq_tracking;
    }

    // Sort windows by mean power → quietest first
    std::sort(refWindows.begin(), refWindows.end(),
              [](const RefWindow& a, const RefWindow& b){ return a.meanDb < b.meanDb; });

    {
        const int nBottom = std::max(1, static_cast<int>(nRef * 0.4f));

        // ── 6.4a  Mean-energy noise: bottom-40% window means, MAD ────────────
        {
            float mean = 0.0f;
            for (int i = 0; i < nBottom; i++) mean += refWindows[i].meanDb;
            mean /= nBottom;
            std::vector<float> gaps(nBottom);
            for (int i = 0; i < nBottom; i++) gaps[i] = std::fabs(refWindows[i].meanDb - mean);
            std::sort(gaps.begin(), gaps.end());
            float sigma = std::max(1.4816f * gaps[nBottom / 2], 0.5f);

            const float snrDb    = signalPowerDb - mean;
            this->meanSnrDb         = snrDb;
            this->meanSnrSigma = snrDb / sigma;
        }

        // ── 6.4b  σ_bin: pool all individual bins from the quiet windows ─────
        // The bottom-40% windows contain only noise → their per-bin dB values
        // give a direct, unbiased estimate of the per-bin noise distribution.
        std::vector<float> pooledBinDb;
        pooledBinDb.reserve(nBottom * 70);  // approx capacity
        for (int j = 0; j < nBottom; j++) {
            for (int i = refWindows[j].lo; i <= refWindows[j].hi; i++)
                pooledBinDb.push_back(10.0f * log10f(power_shifted[i] / refPower + 1e-20f));
        }
        float sigmaBin = 1.0f;  // safe default
        float perBinMean = 0.0f;
        if (!pooledBinDb.empty()) {
            for (float v : pooledBinDb) perBinMean += v;
            perBinMean /= static_cast<float>(pooledBinDb.size());
            this->perBinMean = perBinMean;
            std::vector<float> gaps(pooledBinDb.size());
            for (size_t i = 0; i < pooledBinDb.size(); i++)
                gaps[i] = std::fabs(pooledBinDb[i] - perBinMean);
            std::sort(gaps.begin(), gaps.end());
            sigmaBin = std::max(1.4816f * gaps[gaps.size() / 2], 1.0f);  // floor 1 dB
        }

        // ── 6.4b²  Peak-above-noise-mean: max bin in focus vs per-bin noise mean ──
        // Raw dB headroom — no Gumbel correction, no sigma normalization.
        // In pure noise ≈ +16 dB (Gumbel expected offset); grows with signal strength.
        peakAboveNoiseMeanDb = absPeakDb - perBinMean;

        // ── 6.4c  Max-bin SNR: Gumbel correction using σ_bin ─────────────────
        // For max of focusLen i.i.d. ~Gaussian(perBinMean, σ_bin) bins:
        //   E[max]    ≈ perBinMean + σ_bin × √(2 ln focusLen)      (Gumbel location)
        //   σ(max)    ≈ σ_bin × π / (√6 × √(2 ln focusLen))        (Gumbel scale)
        // In pure noise: maxBinSnrSigma ≈ 0 by construction.
        {
            const float logN       = std::log(static_cast<float>(focusLen));
            const float sqrt2logN  = std::sqrt(2.0f * logN);
            const float gumbelLoc  = perBinMean + sigmaBin * sqrt2logN;
            const float gumbelSig  = std::max(sigmaBin * 3.14159f / (std::sqrt(6.0f) * sqrt2logN), 0.5f);
            maxBinSnrDb    = absPeakDb - gumbelLoc;
            maxBinSnrSigma = maxBinSnrDb / gumbelSig;
        }

        // ── 6.4d  Best-1kHz SNR: bottom-40% best-1kHz reference + σ_bin floor ─
        // Sigma floor = σ_bin/√winBins1k (CLT lower bound for a mean of winBins1k bins).
        {
            float mean1k = 0.0f;
            for (int i = 0; i < nBottom; i++) mean1k += refWindows[i].best1kDb;
            mean1k /= nBottom;
            std::vector<float> gaps(nBottom);
            for (int i = 0; i < nBottom; i++) gaps[i] = std::fabs(refWindows[i].best1kDb - mean1k);
            std::sort(gaps.begin(), gaps.end());
            const float sigmaFloor1k = sigmaBin / std::sqrt(static_cast<float>(winBins1k));
            const float sigma1k = std::max({1.4816f * gaps[nBottom / 2], sigmaFloor1k, 0.5f});

            const float focusBest1kLinear = best1kHzMean(focusLo, focusHi);
            if (focusBest1kLinear > 0.0f) {
                const float focusBest1kDb = 10.0f * log10f(focusBest1kLinear / refPower + 1e-20f);
                best1kHzSnrDb    = focusBest1kDb - mean1k;
                best1kHzSnrSigma = best1kHzSnrDb / sigma1k;

                // Find start bin of the winning 1-kHz window in the focus band
                {
                    const int len = focusHi - focusLo + 1;
                    int bestStart = focusLo;
                    if (len >= winBins1k) {
                        float rs = 0.f;
                        for (int i = focusLo; i < focusLo + winBins1k; i++) rs += power_shifted[i];
                        float bv = rs;
                        for (int s = focusLo + 1; s + winBins1k - 1 <= focusHi; s++) {
                            rs += power_shifted[s + winBins1k - 1] - power_shifted[s - 1];
                            if (rs > bv) { bv = rs; bestStart = s; }
                        }
                    }
                    best1kHzCenterFreqHz = (bestStart + winBins1k / 2) * freqPerBin
                                          + (static_cast<float>(centerFrequency) - nyquist);
                }
            } else {
                best1kHzSnrDb = best1kHzSnrSigma = 0.0f;
            }
        }
    }

    freq_tracking:

    // ── 6.5  Frequency tracking (uses internal absolute peak) ─────────────────
    if (trackingFrequency == 0.0f)
        trackingFrequency = static_cast<float>(centerFrequency);

    if (sdr_bridge_internal::isCenterFrequencyChanged) {
        trackingFrequency = static_cast<float>(centerFrequency);
        sdr_bridge_internal::isCenterFrequencyChanged = false;
    }

    if (maxPeakAndFrequency.empty())
        maxPeakAndFrequency = {-130.0f, static_cast<float>(centerFrequency)};

    if (valid && absPeakDb > maxPeakAndFrequency[0]) {
        maxPeakAndFrequency[0] = absPeakDb;
        // Absolute frequency of the peak bin
        maxPeakAndFrequency[1] = static_cast<float>(
            (focusLo + peakBinInFocus) * freqPerBin + (centerFrequency - nyquist));
        timeOfLastMaxPeak = std::chrono::steady_clock::now();
    }

    {
        auto now         = std::chrono::steady_clock::now();
        auto msSincePeak = std::chrono::duration_cast<std::chrono::milliseconds>(
                               now - timeOfLastMaxPeak).count();
        if (timeOfLastMaxPeakUpdate < timeOfLastMaxPeak && msSincePeak > 300) {
            trackingFrequency       = maxPeakAndFrequency[1];
            timeOfLastMaxPeakUpdate = now;
            maxPeakAndFrequency[0]  = -130.0f;
        }
    }

    // ── 6.6  Signal detection → detectionFlag (binary 0 / 3) ───────────
    // Two consecutive frames above detectionThresholdSigma required (confirmation = 1).
    const bool aboveThreshold = valid && (this->meanSnrSigma >= detectionThresholdSigma);

    if (aboveThreshold) {
        if (peakConfirmed < confirmation) peakConfirmed++;
    } else {
        peakConfirmed = 0;
    }

    const int currentFlag = (aboveThreshold && peakConfirmed >= confirmation) ? 3 : 0;

    detectionFlagBuffer[indexdetectionFlagBuffer] = currentFlag;
    indexdetectionFlagBuffer = (indexdetectionFlagBuffer + 1) % detectionFlagRemanance;
    this->detectionFlagSent  = *std::max_element(
        detectionFlagBuffer.begin(), detectionFlagBuffer.end());
}