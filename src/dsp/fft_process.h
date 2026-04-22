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
    float meanSnrDb;
    float meanSnrSigma;
    long trackingFrequency;
    int detectionFlag;
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
    float getMeanSnrDb() const { return meanSnrDb; }
    float getMeanSnrSigma() const { return meanSnrSigma; }
    long getTrackingFrequency() const { return static_cast<long>(std::round(trackingFrequency)); }
    int getDetectionFlag() const { return detectionFlagSent; }

    // Max-bin SNR (strongest single bin vs noise)
    float getPeakAboveNoiseMeanDb() const { return peakAboveNoiseMeanDb; }
    float getMaxBinSnrDb()    const { return maxBinSnrDb; }
    float getMaxBinSnrSigma() const { return maxBinSnrSigma; }

    // Best 1 kHz band SNR (mean over best 1 kHz sub-window vs noise)
    float getBest1kHzSnrDb()       const { return best1kHzSnrDb; }
    float getBest1kHzSnrSigma()    const { return best1kHzSnrSigma; }
    float getBest1kHzCenterFreqHz() const { return best1kHzCenterFreqHz; }

private:
    // Configuration parameters
    FftProcessorConfig config_;

    // DSP processing internal state variables
    int integrationCount = 0;
    int integrationPeriod = 10; // Number of packets to integrate in temporal. 1 means no integration to be calculated and so far displayed
    int bufferIndex = 0;
    fftwf_complex *circularBuffer = nullptr;
    int currentSampCount = 0;

    // Frequency tracking
    float trackingFrequency = 0.0f;
    std::vector<float> maxPeakAndFrequency;
    std::chrono::steady_clock::time_point timeOfLastMaxPeak;
    std::chrono::steady_clock::time_point timeOfLastMaxPeakUpdate;

    float refMagnitude = 1.0f;
    float refPower = refMagnitude * refMagnitude;

    // Signal detection
    // Requires `confirmation` consecutive frames above detectionThresholdSigma
    int   confirmation            = 1;
    int   peakConfirmed           = 0;
    float detectionThresholdSigma = 4.0f;  // SNR threshold in σ units

    // detectionFlag: max over a rolling buffer of N frames (remanance)
    std::vector<int> detectionFlagBuffer;
    int detectionFlagRemanance   = 3;
    int indexdetectionFlagBuffer = 0;

    // Output variables
    std::vector<float> power_shifted_vec;
    // meanSnrDb    → mean-window SNR in dB  (mean focus power − noise mean) — UI display
    // meanSnrSigma → mean-window SNR in σ   (meanSnrDb / noiseσ) — state machine
    float meanSnrDb          = 0.0f;
    float meanSnrSigma       = 0.0f;
    int   detectionFlagSent = 0;

    // Peak-above-noise-mean: max bin in focus window − per-bin noise mean (dB, raw, no Gumbel)
    float peakAboveNoiseMeanDb = 0.0f;

    // Max-bin SNR outputs (strongest single FFT bin vs noise reference, Gumbel-corrected)
    float maxBinSnrDb    = 0.0f;
    float maxBinSnrSigma = 0.0f;

    // Best 1 kHz band SNR outputs (mean over the best 1 kHz sub-window vs noise)
    float best1kHzSnrDb       = 0.0f;
    float best1kHzSnrSigma    = 0.0f;
    float best1kHzCenterFreqHz = 0.0f;  // absolute Hz of the best-1kHz window centre

    // Helper functions (private)
    void computePowerSpectrum(fftwf_complex *fft_signal, uint32_t sampCount, float* power_out);
    void shiftPowerSpectrum(const float* power_in, uint32_t sampCount, float* power_shifted_out);
    void evaluateSignalStrength(uint32_t sampCount, const float* power_shifted, uint32_t sampleRate, uint32_t centerFrequency);
};

#endif //SDR_PROTRACK_FFT_PROCESS_H