#ifndef SSB_PROCESSOR_H
#define SSB_PROCESSOR_H

#include <vector>
#include <complex>
#include <cstdint>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <functional> // For std::function
#include "audio_pulse_detector.h"

// Forward declarations
struct SSB_Data {
    std::vector<std::complex<float>> iq;
    uint32_t sampleRate;
};

// Define the type for the C++ callback function
using PcmDataCallback = std::function<void(const std::vector<int16_t>&)>;

class SSBProcessor {
public:
    SSBProcessor();
    ~SSBProcessor();

    void startProcessing(PcmDataCallback pcm_cb);
    void startProcessing(PcmDataCallback pcm_cb, std::function<void(float, int)> pulse_cb);
    void stopProcessing();
    void enqueueData(std::vector<std::complex<float>>&& iq_data, uint32_t sample_rate);
    void setPulseConfig(const AudioPulseDetector::Config& cfg);
    float getAmbientEnergy() const { return pulseDetector_.lastPulseStrength(); }
    float getCurrentRatio()  const { return 0.f; }

private:
    PcmDataCallback pcmCallback; // C++ callback function

    AudioPulseDetector pulseDetector_;
    std::function<void(float, int)> pulseCallback_;  // strength (dB SNR) + level (PulseLevel int)

    // Thread management
    std::queue<SSB_Data> ssb_queue;
    std::mutex ssb_mutex;
    std::condition_variable ssb_cv;
    std::thread ssb_worker_thread;
    std::atomic<bool> ssb_worker_running{false};

    std::mutex configMutex_;
    bool pendingConfigUpdate_ = false;
    AudioPulseDetector::Config pendingConfig_;


    // The actual thread function
    static void ssbProcessingThreadEntry(SSBProcessor* processor);
    void ssbProcessingLoop();
};

#endif // SSB_PROCESSOR_H