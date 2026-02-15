#ifndef SDR_PROTRACK_FFT_PROCESS_H
#define SDR_PROTRACK_FFT_PROCESS_H

#include <complex>
#include <vector>
#include <cstdint>
#include <chrono>
#include <fftw3.h>

// Define a struct to hold all the output parameters
struct FftProcessOutput {
    std::vector<float> powerSpectrum; // Shifted power spectrum
    float peakDb;
    float peakNormalized;
    long trackingFrequency;
    int signalStrengthIndex;
};

// Define a struct to hold all the configuration parameters
struct FftProcessorConfig {
    uint32_t centerFrequency;
    uint32_t sampleRate;
    int samplesPerReading;
    int freqFocusRangeKhz;
    // Add other relevant configuration parameters here
};


class FFTProcessor {
public:
    FFTProcessor(); // Default constructor
    ~FFTProcessor(); // Destructor to free allocated memory
    void configure(const FftProcessorConfig& config); // Method to configure the processor

    void process(const std::complex<float> *input_buf, uint32_t input_len);

    // Getters for results
    const std::vector<float>& getPowerSpectrum() const { return power_shifted_vec; }
    float getPeakDb() const { return peakDb; }
    float getPeakNormalized() const { return peakNormalized; }
    long getTrackingFrequency() const { return static_cast<long>(std::round(trackingFrequency)); }
    int getSignalStrengthIndex() const { return signalStrengthIndexSent; }

private:
    // Configuration parameters
    FftProcessorConfig config_;

    // DSP processing internal state variables
    int integrationCount = 0;
    int integrationPeriod = 1; // Number of packets to integrate in temporal. 1 means no integration to be calculated and so far displayed
    int bufferIndex = 0;
    fftwf_complex *circularBuffer = nullptr;
    int currentSampCount = 0;

    float frequence_of_interest = 432000000.0f; // This might become part of config if adjustable
    int index_f_interest = 5; // This might become part of config if adjustable
    float trackingFrequency = 0.0f;
    float lowTrackingFrequency = -3000.0f; //inf tol -3khz
    float upperTrackingFrequency = 1000.0f; //sup tol +1khz
    std::vector<float> maxPeakAndFrequency;
    std::chrono::steady_clock::time_point timeOfLastMaxPeak;
    std::chrono::steady_clock::time_point timeOfLastMaxPeakUpdate;

    float refMagnitude = 50000.0f;
    float refPower = refMagnitude * refMagnitude;

    std::vector<std::chrono::steady_clock::time_point> signalTimeTable;
    std::vector<int> signalStrengthIndexBuffer;
    int signalStrengthIndexRemanance = 3;
    int indexsignalStrengthIndexBuffer = 0;
    int signalWeak = 1;
    int signalMedium = 5;
    int signalStrong = 20;
    int remananceWeak = 0;
    int remananceMedium = 0;
    int remananceStrong = 0;

    std::vector<float> noisePercentileBuffer;
    std::vector<float> noiseMedianBuffer;
    int noiseBufferSize = 1;
    int indexNoiseBuffer = 0;

    float noiseSigmaMin = 100;
    float noiseSigmaMax = 0;
    float peakNormalizedMax = 0;

    std::vector<float> peakBuffer;
    std::vector<float> peakNormalizedBuffer;
    int peakRemanance = 5;
    int indexPeakBuffer = 0;

    int confirmation = 1;
    int peakConfirmed = 0;

    std::vector<float> thresholdBuffer;
    int indexThreshold = 0;

    float maxThreshold = 0;
    float maxThresholdConfirmed = 0;

    float lvl1Ratio = 0.0f;
    int loopNB = 0;
    int lvl1NB = 0;

    // Output variables
    std::vector<float> power_shifted_vec;
    float peakDb = -130.0f;
    float peakNormalized = 0.0f;
    int signalStrengthIndexSent = 0;

    // Helper functions (private)
    void computePowerSpectrum(fftwf_complex *fft_signal, uint32_t sampCount, float* power_out);
    void shiftPowerSpectrum(const float* power_in, uint32_t sampCount, float* power_shifted_out);
    void evaluateSignalStrength(uint32_t sampCount, const float* power_shifted, uint32_t sampleRate, uint32_t centerFrequency);
};

#endif //SDR_PROTRACK_FFT_PROCESS_H