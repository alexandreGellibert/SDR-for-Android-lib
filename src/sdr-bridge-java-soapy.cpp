#include <string>
#include <vector>
#include <complex>
#include <algorithm>
#include <numeric>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <ranges>

// Third-party headers required for the implementation details not just declarations
#include <fftw3.h>
#include <SoapySDR/Modules.hpp>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Types.hpp>
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Formats.h>

// Local project headers required for the implementation details not just declarations
#include "sdr-logger.h"
#include "bridge-config.h" // For BridgeConfig
#include "ssb/ssb_demod_opt.h" // For processSSB_opt
#include "ssb/ssb_processor.h" // For SSBProcessor
#include "dsp/fft_process.h" // For FFTProcessor
#include "sdr-bridge-internal.h"

template <typename Range, typename T>
bool contains(const Range& r, const T& value) {
    return std::find(std::begin(r), std::end(r), value) != std::end(r);
}

// anonymous namespace style (cleaner that static)
namespace {

    // Definition of global variables
    jobject fftCallbackObj = nullptr;
    jmethodID fftCallbackMethod = nullptr;
    jobject strengthCallbackObj = nullptr;
    jmethodID strengthCallbackMethod = nullptr;
    jobject peakCallbackObj = nullptr;
    jmethodID peakCallbackMethod = nullptr;
    jobject peakNormalizedCallbackObj = nullptr;
    jmethodID peakNormalizedCallbackMethod = nullptr;
    jobject peakFrequencyCallbackObj = nullptr;
    jmethodID peakFrequencyCallbackMethod = nullptr;
    jobject pcmCallbackObj = nullptr;
    jmethodID pcmCallbackMethod = nullptr;

    // Global JNI object for passing float arrays back to Java/Kotlin
    jfloatArray result = nullptr;
    jint resultSize = 0;

    // Global state variables related to device and processing status
    struct RxChunk {
        std::vector<std::complex<float>> samples;
    };
    // SoapySDR device and stream
    SoapySDR::Device *sdrDevice;
    SoapySDR::Stream *rxStream;

    // FFT Processor instance
    FFTProcessor fftProcessor;

    // SSB Processor instance
    SSBProcessor ssbProcessor;

    // Internal mutex, condition variable, queue and buffer
    std::mutex rx_mutex;
    std::condition_variable rx_cv;
    // Put the Received Data in a Queue because SoapyRTLSDR reads are not continuous but bursty
    std::deque<RxChunk> rx_queue;
    // Put the Received Data in a Buffer in case the received data are below the expected data on the UI (samplesPerReading)
    std::vector<std::complex<float>> accBuffer;

    std::thread rx_reading_thread;
    std::thread rx_process_thread;

    constexpr size_t RX_QUEUE_MAX = 20;
    size_t current_rx_MTU = 0; // Initialize to 0

    std::atomic<bool> rx_running{false};

    std::atomic<bool> isUpdatingConfiguration(false);

    // --- Helper Functions (declared for internal use within the module) ---

    /**
     * Initializes Java-related global variables (JNIEnv, method IDs)
     * once the Java wrapper object is available.
     * @param env The JNI environment.
     * @param thiz The Java 'this' object (instance of SDRBridgeWrapper).
     */
    void initJavaVariables(JNIEnv *env, jobject thiz) {
        initLoggerJni(env, thiz);
    }

    /**
     * Calculates an optimal buffer length (in bytes) for USB transfers,
     * aiming for low latency without excessive overhead.
     * @param sampleRateHz The desired sample rate in Hz.
     * @param format The SoapySDR stream format (e.g., SOAPY_SDR_CF32).
     * @return The calculated optimal buffer length in bytes, clamped to reasonable min/max values.
     */
    size_t calculateOptimalBufflen(uint32_t sampleRateHz, const std::string &format){
        if (sampleRateHz <= 0.0) return 16384;  // fallback for invalid rate

        constexpr double targetFillTimeLow = 0.025;  // 25 ms — good compromise for tracking
        constexpr double targetFillTimeHigh = 0.050;  // 50 ms — more throughput at high rates

        double targetTime = (sampleRateHz < 500000.0) ? targetFillTimeLow : targetFillTimeHigh;

        // samples = rate × time
        double samplesPerBuffer = sampleRateHz * targetTime;

        size_t bytesPerSample;
        if (format == SOAPY_SDR_CF32) {
            bytesPerSample = 8;  // 2 × float = 8 bytes
        } else if (format == SOAPY_SDR_CS16) {
            bytesPerSample = 4;  // 2 × int16 = 4 bytes
        } else {
            bytesPerSample = 4;  // fallback, or throw error
        }

        double bytesRaw = samplesPerBuffer * bytesPerSample;

        // Round to nearest multiple of 512
        size_t bufflen = static_cast<size_t>(std::round(bytesRaw / 512.0) * 512.0);

        // Hard clamps: too small → high CPU overhead; too big → bursty at low rates
        bufflen = std::max<size_t>(4096, bufflen);   // min ~10 ms @ 100 kS/s
        bufflen = std::min<size_t>(262144, bufflen);   // don't exceed typical default

        return bufflen;
    }

    /**
     * Calculates an optimal number of ring buffers for SoapySDR,
     * providing headroom against scheduling jitter.
     * @param sampleRateHz The desired sample rate in Hz.
     * @return The calculated optimal number of buffers.
     */
    size_t calculateOptimalBuffers(uint32_t sampleRateHz) {
        if (sampleRateHz <= 0.0) return 12;

        // Base: 12 is usually safe
        size_t buffers = 12;

        // Very low rates (<250 kS/s) → give a bit more headroom against Android scheduling jitter
        if (sampleRateHz < 250000.0) {
            buffers += 4;  // → 16
        }
            // High rates (>2 MS/s) → can be a bit more aggressive, but don't go crazy
        else if (sampleRateHz > 2000000.0) {
            buffers = std::min<size_t>(24, buffers + 4);
        }

        return buffers;
    }

    /**
     * Sets up or updates the SoapySDR receive stream based on current preferences.
     * Tears down and re-initializes the stream if already active.
     * @param currentSampleRate The sample rate to configure the stream with.
     * @return True if the stream was successfully set up or updated, false otherwise.
     */
    bool setupOrUpdateRxStream(uint32_t currentSampleRate) {
        if (sdrDevice == nullptr) {
            // Log error: device not open
            return false;
        }

        // If stream already active → tear down first
        if (rxStream != nullptr) {
            sdrDevice->deactivateStream(rxStream);
            sdrDevice->closeStream(rxStream);
            rxStream = nullptr;
        }

        // SOAPY_SDR_CF32 (complex float 32-bit, i.e., two float values per IQ sample)
        std::string streamFormat = SOAPY_SDR_CF32;

        SoapySDR::Kwargs streamArgs;

        // RTL-SDR specific keys
        size_t bufflen = calculateOptimalBufflen(currentSampleRate, streamFormat);
        size_t numBuffers = calculateOptimalBuffers(currentSampleRate);

//         bufflen = 16384;   // bytes → 4096 IQ samples
//         numBuffers = 8;

        streamArgs["bufflen"] = std::to_string(bufflen);
        streamArgs["buffers"] = std::to_string(numBuffers);

        // Lime specific key
        streamArgs["bufferLength"] = std::to_string(bufflen / 4);  // in samples, not bytes

        // You can add more args if needed, e.g. "asyncBuffs"="4" for advanced tuning

        // empty channels -> automatic
        std::vector<size_t> channels = std::vector<size_t>();

        try {
            rxStream = sdrDevice->setupStream(
                    SOAPY_SDR_RX,
                    streamFormat,
                    channels,
                    streamArgs
            );

            if (rxStream == nullptr) {
                // Log: setup failed
                return false;
            }

            // Activate in continuous mode (numElems=0)
            int ret = sdrDevice->activateStream(rxStream, 0, 0LL, 0);
            if (ret != 0) {
                sdrDevice->closeStream(rxStream);
                rxStream = nullptr;
                // Log error with ret
                return false;
            }

            // Query new MTU after setup (good practice)
            current_rx_MTU = sdrDevice->getStreamMTU(rxStream);
            if (current_rx_MTU == 0) current_rx_MTU = 8192;  // fallback if not supported

            LOGD("Stream (re)setup OK | bufflen=%zu | buffers=%d | MTU=%d", bufflen, numBuffers,
                 current_rx_MTU);

            return true;
        }
        catch (const std::exception &e) {
            // Log exception
            if (rxStream != nullptr) {
                sdrDevice->closeStream(rxStream);
                rxStream = nullptr;
            }
            return false;
        }
    }

    /**
     * Identifies the main receive gain stage available on the SoapySDR device.
     * @param device Pointer to the SoapySDR device.
     * @param channel The channel index (typically 0 for single-channel devices).
     * @return The name of the main gain stage (e.g., "TUNER", "PGA", "RF", "VGA"), or an empty string if not found.
     */
    std::string findMainRxGain(SoapySDR::Device *device, size_t channel = 0) {
        if (!device) return "";

        const auto gains = device->listGains(SOAPY_SDR_RX, channel);

        // Only interested in Main Gain Settings
        // TUNER: Case RTL-SDR, only this gain is of interest
        // PGA: Lime SDR TODO handle cases for Gain TIA and LNA?
        // RF: Airspy
        // HackRF: VGA
        static const std::vector<std::string> preferred = {
                "TUNER",
                "PGA",
                "RF",
                "VGA"
        };

        for (const auto &p: preferred) {
            for (const auto &g: gains) {
                if (g == p) {
                    return g;
                }
            }
        }

        LOGD("No known Gain Setting Found");
        return "";
    }

    /**
     * Sets the gain to the Main gain of the connected device
     * @param device Pointer to the SoapySDR device.
     * @param the gain value
     * @param channel The channel index (typically 0 for single-channel devices).
     * @return The name of the main gain stage (e.g., "TUNER", "PGA", "RF", "VGA"), or an empty string if not found.
     */
    int setMainRxGain(SoapySDR::Device *device, int32_t gain, size_t channel = 0) {
        const std::string mainGain = findMainRxGain(device, channel);

        if (mainGain.empty()) {
            LOGE("No main gain found for device ");
            return -1;
        }
        device->setGainMode(SOAPY_SDR_RX, 0, false);

        device->setGain(SOAPY_SDR_RX, channel, mainGain, gain);
        return 0;
    }
}

// Namespace used when different internall cpp needs access to a variable
namespace sdr_bridge_internal {
    std::atomic<bool> isCenterFrequencyChanged{false};   // or = false;
    JavaVM *gJavaVM = nullptr;
}

JavaVM *getJavaVM() {
    return sdr_bridge_internal::gJavaVM;
}

// Cache JavaVM during JNI initialization
extern "C" JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM *vm, void *reserved) {
    sdr_bridge_internal::gJavaVM = vm;
    LOGD("JNI_OnLoad called, JavaVM cached");

    try {
        std::string err = SoapySDR::loadModule("libSoapyRTLSDR.so");
        if (err.empty()) {
            LOGI("SoapyRTLSDR module loaded");
        } else {
            LOGE("Failed to load SoapyRTLSDR: %s", err.c_str());
        }
    } catch (const std::exception &e) {
        LOGE("Failed to load SoapyRTLSDR: %s", e.what());
    }

    try {
        std::string err = SoapySDR::loadModule("libSoapyLMS7.so");
        if (err.empty()) {
            LOGI("SoapyLimeSDR module loaded");
        } else {
            LOGE("Failed to load SoapyLimeSDR: %s", err.c_str());
        }
    } catch (const std::exception &e) {
        LOGE("Failed to load SoapyLimeSDR: %s", e.what());
    }

    try {
        std::string err = SoapySDR::loadModule("libSoapyAirspy.so");
        if (err.empty()) {
            LOGI("SoapyAirspy module loaded");
        } else {
            LOGE("Failed to load SoapyAirspy: %s", err.c_str());
        }
    } catch (const std::exception &e) {
        LOGE("Failed to load SoapyAirspy: %s", e.what());
    }

    return JNI_VERSION_1_6;
}

extern "C" JNIEXPORT jboolean JNICALL
Java_fr_intuite_sdr_bridge_SDRBridge_initDongle(JNIEnv *env, jobject obj, jint fd,
                                                               jstring usbfsPath, jstring device) {

    initJavaVariables(env, obj);

    BridgeConfig &prefs = BridgeConfig::getInstance();
    if (!prefs.isInitialized()) {
        LOGD("BridgeConfig and Settings not yet initialized");
        return false;
    }

    const char *usbfsPathCStr = env->GetStringUTFChars(usbfsPath, nullptr);
    const char *deviceCStr = env->GetStringUTFChars(device, nullptr);

    SoapySDR::Kwargs args;
    args["driver"] = deviceCStr;
    args["fd"] = std::to_string(fd);
    args["usbfs"] = usbfsPathCStr;
    args["addr"] = "0000:0000";
    args["serial"] = "0000:0000";

    try {
        sdrDevice = SoapySDR::Device::make(args);

        sdrDevice->setFrequency(SOAPY_SDR_RX, 0, prefs.getCenterFrequency());
        sdrDevice->setSampleRate(SOAPY_SDR_RX, 0, prefs.getSampleRate());
        setMainRxGain(sdrDevice, prefs.getGain() / 10);
    } catch (const std::exception &e) {
        LOGD("SoapySDR init failed: %s", e.what());
        return false;
    }

    env->ReleaseStringUTFChars(usbfsPath, usbfsPathCStr);
    env->ReleaseStringUTFChars(device, deviceCStr);
    return true;
}

const bool USB = true;


void soapyCallback(std::complex<float> *buf, uint32_t len) {
    if (!rx_running || isUpdatingConfiguration) return;

    JNIEnv *env = nullptr;
    bool didAttach = false;

    if (getJavaVM()->GetEnv((void **) &env, JNI_VERSION_1_6) != JNI_OK) {
        if (getJavaVM()->AttachCurrentThread(&env, nullptr) == JNI_OK) {
            didAttach = true;
        } else {
            // impossible d’attacher le thread
            return;
        }
    }


//    // Check if the thread is already attached
//    jint result2 = getJavaVM()->GetEnv((void **) &env, JNI_VERSION_1_6);

    uint32_t sampleRate = BridgeConfig::getInstance().getSampleRate();
    // Enqueue data for SSB processing in a separate thread
    ssbProcessor.enqueueData(std::vector<std::complex<float>>(buf, buf + len), sampleRate);

    // Call the FFT processor
    fftProcessor.process(buf, len);

    // Get results from FFT processor
    const std::vector<float> &power_shifted = fftProcessor.getPowerSpectrum();
    float peakDb = fftProcessor.getPeakDb();
    float peakNormalized = fftProcessor.getPeakNormalized();
    long trackingFrequency = fftProcessor.getTrackingFrequency();
    int signalStrengthIndexSent = fftProcessor.getSignalStrengthIndex();

    // Handle global jfloatArray `result`
    if (result == nullptr || resultSize != power_shifted.size()) {
        if (result != nullptr) {
            env->DeleteGlobalRef(result);
            result = nullptr;
        }
        jfloatArray local = env->NewFloatArray(power_shifted.size());
        if (local == nullptr) {
            LOGD("Failed to allocate float array (%zu)", power_shifted.size());
            return;
        }
        result = (jfloatArray) env->NewGlobalRef(local);
        env->DeleteLocalRef(local);
        resultSize = power_shifted.size();
    }

    // Send power spectrum to Java
    env->SetFloatArrayRegion(result, 0, power_shifted.size(), power_shifted.data());
    env->CallVoidMethod(fftCallbackObj, fftCallbackMethod, result);

    // Send other processed data to Java
    env->CallVoidMethod(strengthCallbackObj, strengthCallbackMethod, signalStrengthIndexSent);
    env->CallVoidMethod(peakCallbackObj, peakCallbackMethod, peakDb);
    env->CallVoidMethod(peakNormalizedCallbackObj, peakNormalizedCallbackMethod, peakNormalized);
    env->CallVoidMethod(peakFrequencyCallbackObj, peakFrequencyCallbackMethod, trackingFrequency);

    if (didAttach) {
        getJavaVM()->DetachCurrentThread();
    }
}


void soapy_sdr_read_async() {
    rx_running = true;

    // ===============================
    // THREAD 1 : READ STREAM (sometimes bursty, sometimes smooth, as best as we can)
    // ===============================
    rx_reading_thread = std::thread([=]() {

        void *buffs[1];

        size_t currentSampleSize = 0;

        std::vector<std::complex<float>> rxBuffer;
        while (rx_running) {

            size_t samplesPerReading_to_process =
                    BridgeConfig::getInstance().getSamplesPerReading();

            size_t wantedSampleSize = std::min(current_rx_MTU, samplesPerReading_to_process);

            // 🔧 Change dimension only when necessary
            if (wantedSampleSize != currentSampleSize) {
                currentSampleSize = wantedSampleSize;
                rxBuffer.resize(currentSampleSize);
                buffs[0] = rxBuffer.data();

                LOGD("SoapySDR read size updated: %zu samples", currentSampleSize);
            }

            int flags;
            long long timeNs;

            if(isUpdatingConfiguration) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            int ret = sdrDevice->readStream(
                    rxStream,
                    buffs,
                    currentSampleSize,
                    flags,
                    timeNs,
                    1000000
            );

            if (ret > 0) {
                // 1️⃣ Accumulate in a buffer
                accBuffer.insert(
                        accBuffer.end(),
                        rxBuffer.begin(),
                        rxBuffer.begin() + ret
                );

                LOGD("ret=%d acc=%zu ui=%zu", ret, accBuffer.size(), currentSampleSize);

                // 2️⃣ Cut in Exact size blocs
                while (accBuffer.size() >= samplesPerReading_to_process) {
                    RxChunk chunk;
                    chunk.samples.assign(
                            accBuffer.begin(),
                            accBuffer.begin() + samplesPerReading_to_process
                    );

                    {
                        std::lock_guard<std::mutex> lock(rx_mutex);

                        if (rx_queue.size() >= RX_QUEUE_MAX)
                            rx_queue.pop_front();

                        rx_queue.push_back(std::move(chunk));
                    }

                    accBuffer.erase(
                            accBuffer.begin(),
                            accBuffer.begin() + samplesPerReading_to_process
                    );

                    rx_cv.notify_one();
                }
            }
        }
    });

    // ===================================
    // THREAD 2 : PROCESS
    // ===================================
    rx_process_thread = std::thread([=]() {

        const double sampleRate =
                BridgeConfig::getInstance().getSampleRate();
        size_t uiSamples =
                BridgeConfig::getInstance().getSamplesPerReading();

        while (rx_running) {
            auto next = std::chrono::steady_clock::now();

            RxChunk chunk;

            {
                std::unique_lock<std::mutex> lock(rx_mutex);

                // wait for data
                rx_cv.wait_for(lock, std::chrono::milliseconds(60), [] {
                    return !rx_queue.empty() || !rx_running;
                });

                if (!rx_queue.empty()) {
                    chunk = std::move(rx_queue.front());
                    rx_queue.pop_front();
                }
            }

            if (!chunk.samples.empty()) {
                // Tick for UI - if sample rate is too small, we'll pause longer
//                double tickMs = (uiSamples * 1000.0) / sampleRate;
//                tickMs = std::min(tickMs, 80.0);
//                tickMs = std::max(tickMs, 10.0);

                soapyCallback(
                        chunk.samples.data(),
                        chunk.samples.size()
                );

//                next += std::chrono::milliseconds(static_cast<int64_t>(tickMs));
            }

            //std::this_thread::sleep_until(next);
        }
    });
}

extern "C" JNIEXPORT void JNICALL
Java_fr_intuite_sdr_bridge_SDRBridge_read(
        JNIEnv *env, jobject thiz,
        jobject fftCallback,
        jobject signalStrengthCallback,
        jobject peakCallback,
        jobject peakNormalizedCallback,
        jobject peakFrequencyCallback,
        jobject pcmCallback) {

    if (sdrDevice == nullptr) {
        LOGD("Device not initialized");
        return;
    }

    // Store the fftCallback globally
    fftCallbackObj = env->NewGlobalRef(fftCallback);
    jclass fftCallbackClass = env->GetObjectClass(fftCallback);
    fftCallbackMethod = env->GetMethodID(fftCallbackClass, "invoke", "([F)V");

    strengthCallbackObj = env->NewGlobalRef(signalStrengthCallback);
    jclass strengthCallbackClass = env->GetObjectClass(signalStrengthCallback);
    strengthCallbackMethod = env->GetMethodID(strengthCallbackClass, "invoke", "(I)V");

    peakCallbackObj = env->NewGlobalRef(peakCallback);
    jclass peakCallbackClass = env->GetObjectClass(peakCallback);
    peakCallbackMethod = env->GetMethodID(peakCallbackClass, "invoke", "(F)V");

    peakNormalizedCallbackObj = env->NewGlobalRef(peakNormalizedCallback);
    jclass peakNormalizedCallbackClass = env->GetObjectClass(peakNormalizedCallback);
    peakNormalizedCallbackMethod = env->GetMethodID(peakNormalizedCallbackClass, "invoke", "(F)V");

    peakFrequencyCallbackObj = env->NewGlobalRef(peakFrequencyCallback);
    jclass peakFrequencyCallbackClass = env->GetObjectClass(peakFrequencyCallback);
    peakFrequencyCallbackMethod = env->GetMethodID(peakFrequencyCallbackClass, "invoke", "(J)V");

    pcmCallbackObj = env->NewGlobalRef(pcmCallback);
    jclass pcmCallbackClass = env->GetObjectClass(pcmCallback);
    pcmCallbackMethod = env->GetMethodID(pcmCallbackClass, "invoke", "([S)V");

    //THREAD for SSB PART
    // Start SSB worker thread using SSBProcessor
    // Define the C++ callback that handles the JNI part
    PcmDataCallback pcmDataToJavaCallback = [&](const std::vector<int16_t> &pcm_data) {
        if (pcmCallbackObj != nullptr && pcmCallbackMethod != nullptr && !pcm_data.empty()) {
            JNIEnv *env;
            bool attached = false;
            // Attach current thread to JVM if not already attached
            if (sdr_bridge_internal::gJavaVM->GetEnv((void **) &env, JNI_VERSION_1_6) == JNI_EDETACHED) {
                if (sdr_bridge_internal::gJavaVM->AttachCurrentThread(&env, nullptr) == JNI_OK) {
                    attached = true;
                } else {
                    LOGE("Failed to attach thread to JVM for PCM callback.");
                    return;
                }
            } else if (sdr_bridge_internal::gJavaVM->GetEnv((void **) &env, JNI_VERSION_1_6) != JNI_OK) {
                LOGE("Failed to get JNIEnv for PCM callback.");
                return;
            }

            jshortArray pcmArray = env->NewShortArray(pcm_data.size());
            if (pcmArray != nullptr) {
                env->SetShortArrayRegion(pcmArray, 0, pcm_data.size(), pcm_data.data());
                env->CallVoidMethod(pcmCallbackObj, pcmCallbackMethod, pcmArray);
                if (env->ExceptionOccurred()) {
                    LOGE("Exception during PCM callback to Java.");
                    env->ExceptionDescribe();
                    env->ExceptionClear();
                }
                env->DeleteLocalRef(pcmArray);
            }

            if (attached) {
                sdr_bridge_internal::gJavaVM->DetachCurrentThread();
            }
        }
    };
    ssbProcessor.startProcessing(pcmDataToJavaCallback);

    if (!setupOrUpdateRxStream(BridgeConfig::getInstance().getSampleRate())) {
        LOGD("setup SoapySDR Stream failed");
        return;
    }

    int ret = sdrDevice->activateStream(
            rxStream,
            0,      // flags
            0,      // timeNs
            0       // numElems (0 = unlimited)
    );

    if (ret != 0) {
        LOGD("activate SoapySDR Stream failed: %d", ret);
        return;
    }

    // Start asynchronous reading
    soapy_sdr_read_async();
}
// Function to cancel asynchronous reading
extern "C" JNIEXPORT void JNICALL
Java_fr_intuite_sdr_bridge_SDRBridge_stopReading(JNIEnv *env, jobject thiz) {
    LOGD("nativeCancelAsync called");

    // 1. Stop RX loop
    rx_running = false;

    // 2. Stop RX thread
    if (rx_reading_thread.joinable()) {
        rx_reading_thread.join();
    }
    if (rx_process_thread.joinable()) {
        rx_process_thread.join();
    }

    // 3. Deactivate stream
    if (sdrDevice && rxStream) {
        try {
            sdrDevice->deactivateStream(rxStream);
            LOGD("RX stream deactivated");
        } catch (const std::exception &e) {
            LOGD("Error while deactivating stream: %s", e.what());
        }
    }

    // 4. Stop SSB worker
    ssbProcessor.stopProcessing();
}

// Close: Cleanup device
extern "C" JNIEXPORT void JNICALL
Java_fr_intuite_sdr_bridge_SDRBridge_close(JNIEnv *env, jobject obj) {
    LOGD("nativeCloseSDR called");

    // 1. Stop thread
    rx_running = false;
    if (rx_process_thread.joinable()) {
        rx_process_thread.join();
    }
    if (rx_reading_thread.joinable()) {
        rx_reading_thread.join();
    }

    // 2. Stop stream
    if (rxStream) {
        sdrDevice->deactivateStream(rxStream);
        sdrDevice->closeStream(rxStream);
        rxStream = nullptr;
    }

    // 3. Destroy device
    if (sdrDevice) {
        SoapySDR::Device::unmake(sdrDevice);
        sdrDevice = nullptr;
    }

    // 4️⃣ Stop SSB worker
    ssbProcessor.stopProcessing();

    if (fftCallbackObj != nullptr) {
        env->DeleteGlobalRef(fftCallbackObj);
        fftCallbackObj = nullptr;
    }
    if (strengthCallbackObj != nullptr) {
        env->DeleteGlobalRef(strengthCallbackObj);
        strengthCallbackObj = nullptr;
    }
    if (peakFrequencyCallbackObj != nullptr) {
        env->DeleteGlobalRef(peakFrequencyCallbackObj);
        peakFrequencyCallbackObj = nullptr;
    }
    if (peakCallbackObj != nullptr) {
        env->DeleteGlobalRef(peakCallbackObj);
        peakCallbackObj = nullptr;
    }
    if (peakNormalizedCallbackObj != nullptr) {
        env->DeleteGlobalRef(peakNormalizedCallbackObj);
        peakNormalizedCallbackObj = nullptr;
    }
    if (pcmCallbackObj != nullptr) {
        env->DeleteGlobalRef(pcmCallbackObj);
        pcmCallbackObj = nullptr;
    }
    if (result != nullptr) {
        env->DeleteGlobalRef(result);
        result = nullptr;
    }
    LOGD("Device closed");
}

extern "C" JNIEXPORT void JNICALL
Java_fr_intuite_sdr_bridge_SDRBridge_setFrequency(JNIEnv *env, jobject obj,
                                                                    jlong frequency) {
    isUpdatingConfiguration = true;
    BridgeConfig::getInstance().setCenterFrequency(frequency);

    if (sdrDevice == nullptr) {
        isUpdatingConfiguration = false;
        return;
    }

    try {
        sdrDevice->setFrequency(
                SOAPY_SDR_RX,
                0,
                static_cast<double>(frequency)
        );

        LOGD("New Frequency %lld Hz", frequency);
        sdr_bridge_internal::isCenterFrequencyChanged = true;
    }
    catch (const std::exception &e) {
        LOGD("ERROR: Failed to set frequency %lld : %s",
             frequency, e.what());
    }

    isUpdatingConfiguration = false;
}

extern "C" JNIEXPORT void JNICALL
Java_fr_intuite_sdr_bridge_SDRBridge_setSampleRate(JNIEnv *env, jobject obj,
                                                                     jlong sampleRate) {
    isUpdatingConfiguration = true;
    BridgeConfig::getInstance().setSampleRate(sampleRate);

    if (sdrDevice == nullptr) {
        isUpdatingConfiguration = false;
        return;
    }

    try {
        sdrDevice->setSampleRate(
                SOAPY_SDR_RX,
                0,
                static_cast<double>(sampleRate)
        );
        LOGD("New Sample Rate %lld Hz", sampleRate);

        if (!setupOrUpdateRxStream(sampleRate)) {
            LOGD("Could not adjust Stream Setup to new sample rate %lld Hz", sampleRate);
        }
    }
    catch (const std::exception &e) {
        LOGD("ERROR: Failed to set sample rate %lld : %s",
             sampleRate, e.what());
    }

    isUpdatingConfiguration = false;
}

extern "C" JNIEXPORT void JNICALL
Java_fr_intuite_sdr_bridge_SDRBridge_setGain(JNIEnv *env, jobject obj,
                                                               jint gain) {
    isUpdatingConfiguration = true;
    BridgeConfig::getInstance().setGain(gain);

    if (!sdrDevice) {
        isUpdatingConfiguration = false;
        return;
    }

    auto gains = sdrDevice->listGains(SOAPY_SDR_RX, 0);

    for (const auto &g: gains) {
        LOGD("Gain stage: %s", g.c_str());
        double v = sdrDevice->getGain(SOAPY_SDR_RX, 0, g);
        LOGD("Gain %s = %.1f dB", g.c_str(), v);
    }

    try {
        // UI -> dB
        double gainDb = gain / 10.0;

        // Désactiver AGC (important pour RTL-SDR)
        sdrDevice->setGainMode(SOAPY_SDR_RX, 0, false);

        const std::string mainGain = findMainRxGain(sdrDevice, 0);

        if (!mainGain.empty()) {
            sdrDevice->setGain(SOAPY_SDR_RX, 0, mainGain, gainDb);

            // TODO isolate this code only for Lime devices
            const std::string lnaGain = "LNA";
            if (contains(gains, lnaGain)) {
                auto lnaRange = sdrDevice->getGainRange(SOAPY_SDR_RX, 0, lnaGain);
                sdrDevice->setGain(SOAPY_SDR_RX, 0, lnaGain, lnaRange.maximum() - 12);
            }

            const std::string tiaGain = "TIA";
            if (contains(gains, tiaGain)) {
                auto tiaRange = sdrDevice->getGainRange(SOAPY_SDR_RX, 0, tiaGain);
                sdrDevice->setGain(SOAPY_SDR_RX, 0, tiaGain, tiaRange.maximum() - 3);
            }
            LOGD("Main RX Gain %s = %.1f dB", mainGain.c_str(), gainDb);
        } else {
            // Fallback global
            sdrDevice->setGain(SOAPY_SDR_RX, 0, gainDb);
            LOGD("Global RX Gain = %.1f dB", gainDb);
        }

    } catch (const std::exception &e) {
        LOGD("ERROR setting gain: %s", e.what());
    }

    isUpdatingConfiguration = false;
}

extern "C" JNIEXPORT void JNICALL
Java_fr_intuite_sdr_bridge_SDRBridge_setSamplesPerReading(JNIEnv *env,
                                                                            jobject obj,
                                                                            jint samplesPerReading) {
    isUpdatingConfiguration = true;
    BridgeConfig::getInstance().setSamplesPerReading(samplesPerReading);
    LOGD("New Samples Per Reading %d", samplesPerReading);
    isUpdatingConfiguration = false;
}

extern "C" JNIEXPORT void JNICALL
Java_fr_intuite_sdr_bridge_SDRBridge_setFrequencyFocusRange(JNIEnv *env,
                                                                              jobject obj,
                                                                              jint frequencyFocusRange) {
    isUpdatingConfiguration = true;
    BridgeConfig::getInstance().setSamplesPerReading(frequencyFocusRange);
    LOGD("New Frequency Focus Range %ld", frequencyFocusRange);
    isUpdatingConfiguration = false;
}

extern "C" JNIEXPORT void JNICALL
Java_fr_intuite_sdr_bridge_SDRBridge_setRefreshFFTMs(JNIEnv *env, jobject obj,
                                                                       jlong refreshFFTMs) {
    isUpdatingConfiguration = true;
    BridgeConfig::getInstance().setRefreshFFTMs(refreshFFTMs);
    LOGD("New Refresh FFT period in ms %ld", refreshFFTMs);
    isUpdatingConfiguration = false;
}

extern "C" JNIEXPORT void JNICALL
Java_fr_intuite_sdr_bridge_SDRBridge_setRefreshPeakMs(JNIEnv *env, jobject obj,
                                                                        jlong refreshPeakMs) {
    isUpdatingConfiguration = true;
    BridgeConfig::getInstance().setRefreshPeakMs(refreshPeakMs);
    LOGD("New Refresh Peak period in ms %ld", refreshPeakMs);
    isUpdatingConfiguration = false;
}

extern "C" JNIEXPORT void JNICALL
Java_fr_intuite_sdr_bridge_SDRBridge_setRefreshSignalStrengthMs(JNIEnv *env,
                                                                                  jobject obj,
                                                                                  jlong refreshSignalStrengthMs) {
    isUpdatingConfiguration = true;
    BridgeConfig::getInstance().setRefreshSignalStrengthMs(refreshSignalStrengthMs);
    LOGD("New Refresh Signal Strength %ld", refreshSignalStrengthMs);
    isUpdatingConfiguration = false;
}





extern "C" JNIEXPORT void JNICALL
Java_fr_intuite_sdr_bridge_SDRBridge_setSoundMode(JNIEnv *env, jobject obj,
                                                                    jint soundMode) {
    isUpdatingConfiguration = true;
    BridgeConfig::getInstance().setSoundMode(soundMode);
    isUpdatingConfiguration = false;
}

extern "C" JNIEXPORT jboolean JNICALL
Java_fr_intuite_sdr_bridge_SDRBridge_initConfig(
        JNIEnv *env,
        jobject /* this */,
        jlong centerFrequency,
        jlong sampleRate,
        jint samplesPerReading,
        jint freqFocusRangeKhz,
        jint gain,
        jlong refreshFFTMs,
        jlong refreshPeakMs,
        jlong refreshSignalStrengthMs,
        jint soundMode
) {
    BridgeConfig &prefs = BridgeConfig::getInstance();
// Initialize BridgeConfig
    prefs.initialize(
            centerFrequency,
            sampleRate,
            samplesPerReading,
            freqFocusRangeKhz,
            gain,
            refreshFFTMs,
            refreshPeakMs,
            refreshSignalStrengthMs,
            soundMode
    );

    // Configure the FFTProcessor
    FftProcessorConfig fftConfig;
    fftConfig.centerFrequency = centerFrequency;
    fftConfig.sampleRate = sampleRate;
    fftConfig.samplesPerReading = samplesPerReading;
    fftConfig.freqFocusRangeKhz = freqFocusRangeKhz;
    // Add other relevant configuration parameters here if needed by FFTProcessor
    fftProcessor.configure(fftConfig);

    return true;
}

extern "C" JNIEXPORT jintArray JNICALL
Java_fr_intuite_sdr_bridge_SDRBridge_getTunerGains(JNIEnv *env, jobject obj) {
    if (!sdrDevice) return nullptr;

    std::string selectedGain = findMainRxGain(sdrDevice);

    if (selectedGain.empty()) {
        LOGD("No tuner gain found");
        return nullptr;
    }

    auto range = sdrDevice->getGainRange(SOAPY_SDR_RX, 0, selectedGain);

    double min = range.minimum();
    double max = range.maximum();
    double step = range.step();

    if (step == 0.0) {
        // Continuous range → 20 UI steps
        step = (max - min) / 20.0;
    }

    std::vector<int> values;

// Generate steps
    for (double g = min; g < max; g += step) {
        values.push_back(static_cast<int>(std::round(g * 10.0)));
    }

// Ensure max is included
    int maxValue = static_cast<int>(std::round(max * 10.0));
    if (values.empty() || values.back() != maxValue) {
        values.push_back(maxValue);
    }

// (Optional but safe) remove duplicates caused by rounding
    values.erase(std::unique(values.begin(), values.end()), values.end());

    jintArray arr = env->NewIntArray(values.size());
    env->SetIntArrayRegion(arr, 0, values.size(), values.data());
    return arr;
}