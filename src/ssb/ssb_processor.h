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

    void startProcessing(PcmDataCallback pcm_callback_func);
    void stopProcessing();
    void enqueueData(std::vector<std::complex<float>>&& iq_data, uint32_t sample_rate);

private:
    PcmDataCallback pcmCallback; // C++ callback function

    // Thread management
    std::queue<SSB_Data> ssb_queue;
    std::mutex ssb_mutex;
    std::condition_variable ssb_cv;
    std::thread ssb_worker_thread;
    std::atomic<bool> ssb_worker_running{false};

    // Internal SSB processing state
    std::vector<int16_t> pcm; // Buffer for processed PCM audio
    int mode = 1; // Sound mode, will be fetched from BridgeConfig
    bool pulse = false; // Pulse detection flag, if applicable

    // The actual thread function
    static void ssbProcessingThreadEntry(SSBProcessor* processor);
    void ssbProcessingLoop();
};

#endif // SSB_PROCESSOR_H