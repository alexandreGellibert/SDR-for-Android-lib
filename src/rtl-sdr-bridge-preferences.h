//
// Created by alexandre on 4/20/25.
//

#ifndef SDR_BRIDGE_PREFERENCES_H
#define SDR_BRIDGE_PREFERENCES_H


#include <string>
#include <chrono>

class Preferences {
public:
    // Singleton access
    static Preferences& getInstance() {
        static Preferences instance;
        return instance;
    }

    // Initialize with values from Kotlin
    void initialize(
            uint32_t centerFrequency,
            uint32_t sampleRate,
            int samplesPerReading,
            int freqFocusRangeKhz,
            int gain,
            long long refreshFFTMs,
            long long refreshPeakMs,
            long long refreshSignalStrengthMs,
            int soundMode
    ) {
        centerFrequency_ = centerFrequency;
        sampleRate_ = sampleRate;
        samplesPerReading_ = samplesPerReading;
        freqFocusRangeKhz_ = freqFocusRangeKhz;
        gain_ = gain;
        refreshFFTMs_ = std::chrono::milliseconds(refreshFFTMs);
        refreshPeakMs_ = std::chrono::milliseconds(refreshPeakMs);
        refreshSignalStrengthMs_ = std::chrono::milliseconds(refreshSignalStrengthMs);
        soundMode_ = soundMode;
        isPrefsInitialized_ = true;
    }

    // Getters
    uint32_t getCenterFrequency() const { return centerFrequency_; }
    uint32_t getSampleRate() const { return sampleRate_; }
    int getFreqFocusRangeKhz() const { return freqFocusRangeKhz_; }
    int getSamplesPerReading() const { return samplesPerReading_; }
    int getGain() const { return gain_; }
    std::chrono::milliseconds getRefreshGraphMs() const { return refreshFFTMs_; }
    std::chrono::milliseconds getRefreshStrengthMs() const { return refreshPeakMs_; }
    std::chrono::milliseconds getBipMaxLengthMs() const { return refreshSignalStrengthMs_; }
    float getSoundMode() const { return soundMode_; }
    bool isInitialized() const { return isPrefsInitialized_; }

    // Setters (for updates after initialization if needed)
    void setCenterFrequency(long long value) { centerFrequency_ = value; }
    void setSampleRate(long long value) { sampleRate_ = value; }
    void setFreqFocusRangeKhz(int value) { freqFocusRangeKhz_ = value; }
    void setSamplesPerReading(int value) { samplesPerReading_ = value; }
    void setGain(int value) { gain_ = value; }
    void setRefreshFFTMs(long long value) { refreshFFTMs_ = std::chrono::milliseconds(value); }
    void setRefreshPeakMs(long long value) { refreshPeakMs_ = std::chrono::milliseconds(value); }
    void setRefreshSignalStrengthMs(long long value) { refreshSignalStrengthMs_ = std::chrono::milliseconds(value); }
    void setSoundMode(int value) { soundMode_ = value; }

private:
    Preferences() = default; // Private constructor for singleton

    // Member variables
    uint32_t centerFrequency_ = 0;
    uint32_t sampleRate_ = 0;
    int freqFocusRangeKhz_ = 0;
    int samplesPerReading_ = 0;
    int gain_ = 0;
    std::chrono::milliseconds refreshFFTMs_ = std::chrono::milliseconds(0);
    std::chrono::milliseconds refreshPeakMs_ = std::chrono::milliseconds(0);
    std::chrono::milliseconds refreshSignalStrengthMs_ = std::chrono::milliseconds(0);
    int soundMode_ = 1;
    bool isPrefsInitialized_ = false;
};


#endif //SDR_BRIDGE_PREFERENCES_H
