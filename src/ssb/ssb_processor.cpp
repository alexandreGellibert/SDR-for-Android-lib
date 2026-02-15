#include "ssb_processor.h"
#include "ssb_demod_opt.h" // For processSSB_opt
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

void SSBProcessor::startProcessing(PcmDataCallback pcm_callback_func) {
    if (ssb_worker_running) {
        __android_log_print(ANDROID_LOG_WARN, LOG_TAG_SSB_BRIDGE, "SSB processing already running.");
        return;
    }

    pcmCallback = pcm_callback_func; // Store the C++ callback

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

void SSBProcessor::enqueueData(std::vector<std::complex<float>>&& iq_data, uint32_t sample_rate) {
    std::lock_guard<std::mutex> lock(ssb_mutex);
    // Clear queue if it gets too large to prevent excessive latency / memory usage
    if (ssb_queue.size() > 5) { // Arbitrary threshold, can be tuned
        std::queue<SSB_Data>().swap(ssb_queue); // Clear queue
        __android_log_print(ANDROID_LOG_WARN, LOG_TAG_SSB_BRIDGE, "SSB queue cleared due to overflow.");
    }
    ssb_queue.push({std::move(iq_data), sample_rate});
    ssb_cv.notify_one();
}

void SSBProcessor::ssbProcessingThreadEntry(SSBProcessor* processor) {
    processor->ssbProcessingLoop();
}

void SSBProcessor::ssbProcessingLoop() {
    // In this pure C++ layer, we no longer interact with JNI directly.
    // The JavaVM attach/detach logic must be handled by the caller of startProcessing.
    // The actual JNI callback will be handled by the pcmCallback.

    while (ssb_worker_running) {
        std::unique_lock<std::mutex> lock(ssb_mutex);
        ssb_cv.wait(lock, [this] { return !ssb_queue.empty() || !ssb_worker_running; });

        if (!ssb_worker_running && ssb_queue.empty()) {
            break; // Exit if worker is stopped and queue is empty
        }

        if (ssb_queue.empty()) {
            continue; // Spurious wakeup
        }

        SSB_Data data = ssb_queue.front();
        ssb_queue.pop();
        lock.unlock();

        // Get sound mode from BridgeConfig - this is a C++ singleton, so it's fine here
        mode = BridgeConfig::getInstance().getSoundMode();

        // Perform SSB processing
        // 'true' for USB, 'pulse' can be updated by processSSB_opt
        processSSB_opt(data.iq, data.sampleRate, true, pcm, pulse, mode); 

        // Invoke the C++ callback with processed PCM data
        if (pcmCallback && !pcm.empty()) {
            pcmCallback(pcm);
        }
        // TODO: Handle pulse detection callback if needed
    }

    __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG_SSB_BRIDGE, "SSB Thread: Loop finished.");
}