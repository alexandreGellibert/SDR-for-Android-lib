#include "ssb_processor.h"
#include "ssb_demod_opt.h" // For processSSB_opt
#include "audio_pulse_detector.h"
#include "../bridge-config.h" // For BridgeConfig::getInstance()
#include <android/log.h> // For __android_log_print
#include <utility> // For std::move

#define LOG_TAG_SSB_BRIDGE "SSB-Processor-Bridge"

// No need to redeclare SSB_Data here as it's in the header

SSBProcessor::SSBProcessor() {
    // Constructor initializes atomic and other members
    ssb_worker_running = false;
}

SSBProcessor::~SSBProcessor() {
    stopProcessing();
    // The pcmCallback (std::function) doesn't own JNI references, so no need to delete them here.
}

void SSBProcessor::startProcessing(PcmDataCallback pcm_cb) {
    startProcessing(pcm_cb, nullptr);
}

void SSBProcessor::startProcessing(PcmDataCallback pcm_cb,
                                   std::function<void(float)> pulse_cb) {
    if (ssb_worker_running) {
        __android_log_print(ANDROID_LOG_WARN, LOG_TAG_SSB_BRIDGE, "SSB processing already running.");
        return;
    }
    pcmCallback  = pcm_cb;
    pulseCallback_ = pulse_cb;
    ssb_worker_running = true;
    ssb_worker_thread = std::thread(ssbProcessingThreadEntry, this);
    __android_log_print(ANDROID_LOG_INFO, LOG_TAG_SSB_BRIDGE, "SSB processing thread started.");
}


void SSBProcessor::stopProcessing() {
    if (!ssb_worker_running) return;

    ssb_worker_running = false;
    ssb_cv.notify_one(); // Notify the thread to check its running state
    if (ssb_worker_thread.joinable()) {
        ssb_worker_thread.join();
    }
    __android_log_print(ANDROID_LOG_INFO, LOG_TAG_SSB_BRIDGE, "SSB processing thread stopped.");
}

void SSBProcessor::enqueueData(std::vector<std::complex<float>>&& iq_data,
                               uint32_t sample_rate) {
    if (!ssb_worker_running) return;  // ignorer si pas démarré

    std::lock_guard<std::mutex> lock(ssb_mutex);

    // Drop silently the last one
    while (ssb_queue.size() >= 3) {
        ssb_queue.pop();
    }

    ssb_queue.push({std::move(iq_data), sample_rate});
    ssb_cv.notify_one();
}

void SSBProcessor::ssbProcessingThreadEntry(SSBProcessor* processor) {
    processor->ssbProcessingLoop();
}

void SSBProcessor::setPulseConfig(const AudioPulseDetector::Config& cfg) {
    // Reset atomique — le détecteur repart proprement avec la nouvelle config
    pulseDetector_ = AudioPulseDetector(cfg);
}

void SSBProcessor::ssbProcessingLoop() {
    while (ssb_worker_running) {
        SSB_Data data;
        {
            std::unique_lock<std::mutex> lock(ssb_mutex);
            ssb_cv.wait(lock, [this] {
                return !ssb_queue.empty() || !ssb_worker_running;
            });
            if (!ssb_worker_running && ssb_queue.empty()) break;
            if (ssb_queue.empty()) continue;
            data = std::move(ssb_queue.front());
            ssb_queue.pop();
        }

        // Internal SSB processing state
        std::vector<int16_t> pcm; // Buffer for processed PCM audio
        int mode = 1; // Sound mode, will be fetched from BridgeConfig
        bool pulse = false; // Pulse detection flag, if applicable

        processSSB_opt(data.iq, data.sampleRate, true, pcm, pulse, mode);

        if (pcmCallback && !pcm.empty()) {
            pcmCallback(pcm);
        }

        if (pulseCallback_ && pulseDetector_.process(pcm)) {
            pulseCallback_(pulseDetector_.lastPulseStrength());
        }
    }
    __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG_SSB_BRIDGE, "SSB Thread: Loop finished.");
}