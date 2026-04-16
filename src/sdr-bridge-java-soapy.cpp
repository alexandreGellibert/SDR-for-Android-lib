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

enum class Driver {
    RTLSDR,
    LIME,
    AIRSPY,
    AIRSPYHF
};

Driver driver_from_string(std::string_view key) {
    if (key == "rtlsdr"   || key == "RTLSDR")   return Driver::RTLSDR;
    if (key == "lime"     || key == "LIME")     return Driver::LIME;
    if (key == "airspy"   || key == "AIRSPY")   return Driver::AIRSPY;
    if (key == "airspyhf" || key == "AIRSPYHF") return Driver::AIRSPYHF;
    return Driver::RTLSDR;  // fallback
}

std::string_view driver_to_string(Driver d) {
    if (d == Driver::RTLSDR)    return "rtlsdr";
    if (d == Driver::LIME)      return "lime";
    if (d == Driver::AIRSPY)    return "airspy";
    if (d == Driver::AIRSPYHF)  return "airspyhf";
    return "rtlsdr";            // fallback
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
    jobject pulseCallbackObj = nullptr;
    jmethodID pulseCallbackMethod = nullptr;
    jobject maxBinCallbackObj = nullptr;
    jmethodID maxBinCallbackMethod = nullptr;
    jobject best1kHzCallbackObj = nullptr;
    jmethodID best1kHzCallbackMethod = nullptr;

    // Global JNI object for passing float arrays back to Java/Kotlin
    jfloatArray result = nullptr;
    jint resultSize = 0;

    jshortArray pcmArray = nullptr;
    jint pcmArraySize = 0;

    // Global state variables related to device and processing status
    struct RxChunk {
        std::vector<std::complex<float>> samples;
    };
    // SoapySDR device and stream
    SoapySDR::Device *sdrDevice;
    std::optional<Driver> deviceDriver;
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
 * Returns the number of bytes per sample (per complex or real element)
 * for the given SoapySDR stream format.
 *
 * @param format One of the SOAPY_SDR_xxx format strings (e.g. "CF32", "CS16", "CU8", "F32", etc.)
 * @return Number of bytes per sample, or 0 if the format is unknown/invalid.
 */
    size_t getBytesPerSample(const std::string& format) {
        // Most common formats first
        if (format == SOAPY_SDR_CF32) return 8;   // 2 × float (4 bytes each)
        if (format == SOAPY_SDR_CS16) return 4;   // 2 × int16_t (2 bytes each)
        if (format == SOAPY_SDR_CU8)  return 2;   // 2 × uint8_t (1 byte each)
        if (format == SOAPY_SDR_CF64) return 16;  // 2 × double (8 bytes each)

        // Less common / exotic formats
        if (format == SOAPY_SDR_CS32) return 8;   // 2 × int32_t
        if (format == SOAPY_SDR_CU32) return 8;   // 2 × uint32_t
        if (format == SOAPY_SDR_CS8)  return 2;   // 2 × int8_t
        if (format == SOAPY_SDR_CU16) return 4;   // 2 × uint16_t

        // 12-bit formats (packed, 3 bytes per complex sample)
        if (format == SOAPY_SDR_CS12 || format == SOAPY_SDR_CU12) {
            return 3;
        }

        // Very rare 4-bit packed formats (1 byte per complex sample)
        if (format == SOAPY_SDR_CS4 || format == SOAPY_SDR_CU4) {
            return 1;
        }

        // Real-only formats
        if (format == SOAPY_SDR_F64) return 8;    // double
        if (format == SOAPY_SDR_F32) return 4;    // float
        if (format == SOAPY_SDR_S32 ||
            format == SOAPY_SDR_U32) return 4;    // int32 / uint32
        if (format == SOAPY_SDR_S16 ||
            format == SOAPY_SDR_U16) return 2;    // int16 / uint16
        if (format == SOAPY_SDR_S8  ||
            format == SOAPY_SDR_U8)  return 1;    // int8 / uint8

        // Unknown / invalid format
        return 0;
    }

    /**
     * Calculates an optimal buffer length (in bytes) for USB transfers,
     * aiming for low latency without excessive overhead.
     * @param sampleRateHz The desired sample rate in Hz.
     * @param format The SoapySDR stream format (e.g., SOAPY_SDR_CF32).
     * @return The calculated optimal buffer length in bytes, clamped to reasonable min/max values.
     */
    size_t calculateOptimalBufflenInBytes(uint32_t sampleRateHz, const std::string &format){
        if (sampleRateHz <= 0.0) return 16384;  // fallback for invalid rate

        constexpr double targetFillTimeLow = 0.025;  // 25 ms — good compromise for tracking
        constexpr double targetFillTimeHigh = 0.050;  // 50 ms — more throughput at high rates

        double targetTime = (sampleRateHz < 500000.0) ? targetFillTimeLow : targetFillTimeHigh;

        // samples = rate × time
        double samplesPerBuffer = sampleRateHz * targetTime;

        size_t bytesPerSample = getBytesPerSample(format);
        if (bytesPerSample == 0) {
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
    size_t calculateOptimalBuffersNumber(uint32_t sampleRateHz) {
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

        size_t bufflen_bytes = -1;
        size_t buff_numbers = calculateOptimalBuffersNumber(currentSampleRate);

        // Almost all drivers understand "buffers"
        streamArgs["buffers"] = std::to_string(buff_numbers);

        // Driver-specific
        if (deviceDriver == Driver::RTLSDR) {
            bufflen_bytes = calculateOptimalBufflenInBytes(currentSampleRate, streamFormat);
            streamArgs["bufflen"] = std::to_string(bufflen_bytes);
        } else if (deviceDriver == Driver::LIME) {
            // Usually wants samples, not bytes
            bufflen_bytes = calculateOptimalBufflenInBytes(currentSampleRate, streamFormat);
            size_t bytes_per_sample = getBytesPerSample(streamFormat);
            if (bytes_per_sample == 0) bytes_per_sample = 1;
            size_t bufflen_samples = bufflen_bytes / bytes_per_sample;
            streamArgs["bufferLength"] = std::to_string(bufflen_samples);
        }
        // Airspy, AirspyHF → nothing specific needed, MTU usually 65536

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
            if (current_rx_MTU == 0 || current_rx_MTU > 262144) {
                current_rx_MTU = 65536;  // fallback if not supported or too large
            }

            LOGD("Stream (re)setup OK | bufflen_bytes=%d | buffers_number=%d | MTU=%d", bufflen_bytes, buff_numbers,
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
}

// Namespace used when different internall cpp needs access to a variable
namespace sdr_bridge_internal {
    std::atomic<bool> isCenterFrequencyChanged{false};   // or = false;
    JavaVM *gJavaVM = nullptr;
}

JavaVM *getJavaVM() {
    return sdr_bridge_internal::gJavaVM;
}

bool loadModule(const std::string &moduleName, const std::string &fileName) {
    try {
        std::string err = SoapySDR::loadModule(fileName);
        if (err.empty()) {
            LOGI("%s module loaded", moduleName.c_str());
            return true;
        } else {
            LOGE("Failed to load %s: %s", moduleName.c_str(), err.c_str());
        }
    } catch (const std::exception &e) {
        LOGE("Failed to load %s: %s", moduleName.c_str(), e.what());
    }
    return false;
}

// Cache JavaVM during JNI initialization
extern "C" JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM *vm, void *reserved) {
    sdr_bridge_internal::gJavaVM = vm;
    LOGD("JNI_OnLoad called, JavaVM cached");

    loadModule("SoapyRTLSDR", "libSoapyRTLSDR.so");
    loadModule("SoapyLimeSDR", "libSoapyLMS7.so");
    loadModule("SoapyAirspy", "libSoapyAirspy.so");
    loadModule("SoapyAirspyHF", "libSoapyAirspyHF.so");

    return JNI_VERSION_1_6;
}

extern "C" JNIEXPORT jboolean JNICALL
Java_fr_intuite_sdr_bridge_SDRBridge_initDongle(JNIEnv *env, jobject obj, jint fd,
                                                               jstring usbfsPath, jstring device) {

    initJavaVariables(env, obj);

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
        deviceDriver = driver_from_string(deviceCStr);
        if (!sdrDevice) {
            LOGE("SoapySDR init failed");
            return false;
        }
    } catch (const std::exception &e) {
        LOGE("SoapySDR init failed: %s", e.what());
        return false;
    }

    env->ReleaseStringUTFChars(usbfsPath, usbfsPathCStr);
    env->ReleaseStringUTFChars(device, deviceCStr);
    return true;
}

extern "C" JNIEXPORT jstring JNICALL
Java_fr_intuite_sdr_bridge_SDRBridge_getDriver(JNIEnv *env, jobject obj) {
    if (!sdrDevice || !deviceDriver.has_value()) {
        LOGD("Device not initialized for getDriver");
        return nullptr; // Return null string if device not initialized
    }
    try {
        return env->NewStringUTF(driver_to_string(*deviceDriver).data());
    } catch (const std::exception &e) {
        LOGE("Error getting driver key: %s", e.what());
        return nullptr; // Return null string on error
    }
}

const bool USB = true;


void soapyCallback(std::complex<float> *buf, uint32_t len) {
    if (!rx_running || isUpdatingConfiguration) return;

    JNIEnv *env = nullptr;
    bool didAttach = false;

    jint getEnvResult = sdr_bridge_internal::gJavaVM->GetEnv((void **) &env, JNI_VERSION_1_6);
    if (getEnvResult == JNI_EDETACHED) {
        if (sdr_bridge_internal::gJavaVM->AttachCurrentThread(&env, nullptr) == JNI_OK) {
            didAttach = true;
        } else {
            return;
        }
    } else if (getEnvResult != JNI_OK) {
        return;
    }

    uint32_t sampleRate = BridgeConfig::getInstance().getSampleRate();
    ssbProcessor.enqueueData(std::vector<std::complex<float>>(buf, buf + len), sampleRate);

    fftProcessor.process(buf, len);

    const std::vector<float> &power_shifted = fftProcessor.getPowerSpectrum();
    float peakDb = fftProcessor.getPeakDb();
    float peakNormalized = fftProcessor.getPeakNormalized();
    long trackingFrequency = fftProcessor.getTrackingFrequency();
    int signalStrengthIndexSent = fftProcessor.getSignalStrengthIndex();
    float maxBinSnrDb    = fftProcessor.getMaxBinSnrDb();
    float maxBinSnrSigma = fftProcessor.getMaxBinSnrSigma();
    float best1kHzSnrDb    = fftProcessor.getBest1kHzSnrDb();
    float best1kHzSnrSigma = fftProcessor.getBest1kHzSnrSigma();

    // Use a LOCAL array instead of the global shared one
    jfloatArray localResult = env->NewFloatArray(power_shifted.size());
    if (localResult == nullptr) {
        if (didAttach) sdr_bridge_internal::gJavaVM->DetachCurrentThread();
        return;
    }

    env->SetFloatArrayRegion(localResult, 0, power_shifted.size(), power_shifted.data());
    env->CallVoidMethod(fftCallbackObj, fftCallbackMethod, localResult);
    env->DeleteLocalRef(localResult);

    env->CallVoidMethod(strengthCallbackObj, strengthCallbackMethod, signalStrengthIndexSent);
    env->CallVoidMethod(peakCallbackObj, peakCallbackMethod, peakDb);
    env->CallVoidMethod(peakNormalizedCallbackObj, peakNormalizedCallbackMethod, peakNormalized);
    env->CallVoidMethod(peakFrequencyCallbackObj, peakFrequencyCallbackMethod, (jlong)trackingFrequency);
    if (maxBinCallbackObj  != nullptr) env->CallVoidMethod(maxBinCallbackObj,   maxBinCallbackMethod,   maxBinSnrDb,    maxBinSnrSigma);
    if (best1kHzCallbackObj != nullptr) env->CallVoidMethod(best1kHzCallbackObj, best1kHzCallbackMethod, best1kHzSnrDb, best1kHzSnrSigma);

    if (didAttach) {
        sdr_bridge_internal::gJavaVM->DetachCurrentThread();
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
        jobject pcmCallback,
        jobject audioPulseCallback,
        jobject maxBinCallback,
        jobject best1kHzCallback) {

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

    pulseCallbackObj = env->NewGlobalRef(audioPulseCallback);
    jclass pulseClass = env->GetObjectClass(audioPulseCallback);
    pulseCallbackMethod = env->GetMethodID(pulseClass, "invoke", "(FI)V");

    maxBinCallbackObj = env->NewGlobalRef(maxBinCallback);
    jclass maxBinClass = env->GetObjectClass(maxBinCallback);
    maxBinCallbackMethod = env->GetMethodID(maxBinClass, "invoke", "(FF)V");

    best1kHzCallbackObj = env->NewGlobalRef(best1kHzCallback);
    jclass best1kHzClass = env->GetObjectClass(best1kHzCallback);
    best1kHzCallbackMethod = env->GetMethodID(best1kHzClass, "invoke", "(FF)V");

    //THREAD for SSB PART
    // Start SSB worker thread using SSBProcessor
    // Define the C++ callback that handles the JNI part
    PcmDataCallback pcmDataToJavaCallback = [](const std::vector<int16_t>& pcm_data) {
        if (pcmCallbackObj == nullptr || pcmCallbackMethod == nullptr) return;
        if (pcm_data.empty()) return;

        JNIEnv* env = nullptr;
        bool attached = false;
        if (sdr_bridge_internal::gJavaVM->GetEnv(
                (void**)&env, JNI_VERSION_1_6) == JNI_EDETACHED) {
            if (sdr_bridge_internal::gJavaVM->AttachCurrentThread(
                    &env, nullptr) != JNI_OK) return;
            attached = true;
        }

        // Réutiliser le tableau global si la taille n'a pas changé
        const jint neededSize = static_cast<jint>(pcm_data.size());
        if (pcmArray == nullptr || pcmArraySize != neededSize) {
            if (pcmArray != nullptr) {
                env->DeleteGlobalRef(pcmArray);
            }
            jshortArray local = env->NewShortArray(neededSize);
            if (local == nullptr) {
                if (attached) sdr_bridge_internal::gJavaVM->DetachCurrentThread();
                return;
            }
            pcmArray = static_cast<jshortArray>(env->NewGlobalRef(local));
            env->DeleteLocalRef(local);
            pcmArraySize = neededSize;
        }

        env->SetShortArrayRegion(pcmArray, 0, neededSize,
                                 reinterpret_cast<const jshort*>(pcm_data.data()));
        env->CallVoidMethod(pcmCallbackObj, pcmCallbackMethod, pcmArray);

        if (env->ExceptionOccurred()) env->ExceptionClear();
        if (attached) sdr_bridge_internal::gJavaVM->DetachCurrentThread();
    };

    ssbProcessor.startProcessing(pcmDataToJavaCallback, [=](float strength, int level) {
        if (pulseCallbackObj == nullptr || pulseCallbackMethod == nullptr) return;

        JNIEnv* cbEnv = nullptr;
        bool att = false;
        if (sdr_bridge_internal::gJavaVM->GetEnv(
                (void**)&cbEnv, JNI_VERSION_1_6) == JNI_EDETACHED) {
            if (sdr_bridge_internal::gJavaVM->AttachCurrentThread(
                    &cbEnv, nullptr) != JNI_OK) return;
            att = true;
        }

        cbEnv->CallVoidMethod(pulseCallbackObj, pulseCallbackMethod,
                              static_cast<jfloat>(strength), static_cast<jint>(level));

        // Use cbEnv here — NOT env (env belongs to another thread: thread read())
        if (cbEnv->ExceptionOccurred()) {
            cbEnv->ExceptionClear();
        }

        if (att) sdr_bridge_internal::gJavaVM->DetachCurrentThread();
    });
    if (!setupOrUpdateRxStream(BridgeConfig::getInstance().getSampleRate())) {
        LOGD("setup SoapySDR Stream failed");
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
        deviceDriver.reset();
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
    if (maxBinCallbackObj != nullptr) {
        env->DeleteGlobalRef(maxBinCallbackObj);
        maxBinCallbackObj = nullptr;
    }
    if (best1kHzCallbackObj != nullptr) {
        env->DeleteGlobalRef(best1kHzCallbackObj);
        best1kHzCallbackObj = nullptr;
    }
    if (result != nullptr) {
        env->DeleteGlobalRef(result);
        result = nullptr;
    }
    if (pcmArray != nullptr) {
        env->DeleteGlobalRef(pcmArray);
        pcmArray = nullptr;
        pcmArraySize = 0;
    }
    LOGD("Device closed");
}

extern "C" JNIEXPORT void JNICALL
Java_fr_intuite_sdr_bridge_SDRBridge_setFrequency(JNIEnv *env, jobject obj, jlong frequency) {
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

extern "C" JNIEXPORT jlong JNICALL
Java_fr_intuite_sdr_bridge_SDRBridge_getFrequency(JNIEnv *env, jobject obj) {
    if (!sdrDevice) {
        LOGD("Device not initialized for getFrequency");
        return 0; // Default or error value
    }
    try {
        double freq = sdrDevice->getFrequency(SOAPY_SDR_RX, 0);
        return static_cast<jlong>(freq);
    } catch (const std::exception &e) {
        LOGE("Error getting frequency: %s", e.what());
        return 0; // Default or error value
    }
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

//        if (!setupOrUpdateRxStream(sampleRate)) {
//            LOGD("Could not adjust Stream Setup to new sample rate %lld Hz", sampleRate);
//        }
    }
    catch (const std::exception &e) {
        LOGD("ERROR: Failed to set sample rate %lld : %s",
             sampleRate, e.what());
    }

    isUpdatingConfiguration = false;
}

extern "C" JNIEXPORT jlong JNICALL
Java_fr_intuite_sdr_bridge_SDRBridge_getSampleRate(JNIEnv *env, jobject obj) {
    if (!sdrDevice) {
        LOGD("Device not initialized for getSampleRate");
        return 0;
    }
    try {
        double sampleRate = sdrDevice->getSampleRate(SOAPY_SDR_RX, 0);
        return std::llround(sampleRate);
    } catch (const std::exception &e) {
        LOGE("Error getting sample rate: %s", e.what());
        return 0;
    }
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

    try {
        // UI -> dB
        double gainDb = gain / 10.0;

        // Desactivate AGC (important for RTL-SDR)
        sdrDevice->setGainMode(SOAPY_SDR_RX, 0, false);
        sdrDevice->setGain(SOAPY_SDR_RX, 0, gainDb);
        LOGD("Global RX Gain = %.1f dB", gainDb);
    } catch (const std::exception &e) {
        LOGD("ERROR setting gain: %s", e.what());
    }

    isUpdatingConfiguration = false;
}

extern "C" JNIEXPORT jint JNICALL
Java_fr_intuite_sdr_bridge_SDRBridge_getGain(JNIEnv *env, jobject obj) {
    if (!sdrDevice) {
        LOGD("Device not initialized for getGain");
        return 0;
    }
    try {
        double gainDb = sdrDevice->getGain(SOAPY_SDR_RX, 0);
        return static_cast<jint>(std::round(gainDb * 10.0)); // Convert back to UI value
    } catch (const std::exception &e) {
        LOGE("Error getting gain: %s", e.what());
        return 0;
    }
}

extern "C" JNIEXPORT void JNICALL
Java_fr_intuite_sdr_bridge_SDRBridge_setSamplesPerReading(JNIEnv *env, jobject obj, jint samplesPerReading) {
    isUpdatingConfiguration = true;
    BridgeConfig::getInstance().setSamplesPerReading(samplesPerReading);
    LOGD("New Samples Per Reading %d", samplesPerReading);
    isUpdatingConfiguration = false;
}

extern "C" JNIEXPORT void JNICALL
Java_fr_intuite_sdr_bridge_SDRBridge_setFrequencyFocusRange(JNIEnv *env, jobject obj, jint frequencyFocusRange) {
    isUpdatingConfiguration = true;
    BridgeConfig::getInstance().setSamplesPerReading(frequencyFocusRange);
    LOGD("New Frequency Focus Range %ld", frequencyFocusRange);
    isUpdatingConfiguration = false;
}

extern "C" JNIEXPORT void JNICALL
Java_fr_intuite_sdr_bridge_SDRBridge_setRefreshFFTMs(JNIEnv *env, jobject obj, jlong refreshFFTMs) {
    isUpdatingConfiguration = true;
    BridgeConfig::getInstance().setRefreshFFTMs(refreshFFTMs);
    LOGD("New Refresh FFT period in ms %ld", refreshFFTMs);
    isUpdatingConfiguration = false;
}

extern "C" JNIEXPORT void JNICALL
Java_fr_intuite_sdr_bridge_SDRBridge_setRefreshPeakMs(JNIEnv *env, jobject obj, jlong refreshPeakMs) {
    isUpdatingConfiguration = true;
    BridgeConfig::getInstance().setRefreshPeakMs(refreshPeakMs);
    LOGD("New Refresh Peak period in ms %ld", refreshPeakMs);
    isUpdatingConfiguration = false;
}

extern "C" JNIEXPORT void JNICALL
Java_fr_intuite_sdr_bridge_SDRBridge_setRefreshSignalStrengthMs(JNIEnv *env, jobject obj, jlong refreshSignalStrengthMs) {
    isUpdatingConfiguration = true;
    BridgeConfig::getInstance().setRefreshSignalStrengthMs(refreshSignalStrengthMs);
    LOGD("New Refresh Signal Strength %ld", refreshSignalStrengthMs);
    isUpdatingConfiguration = false;
}

extern "C" JNIEXPORT void JNICALL
Java_fr_intuite_sdr_bridge_SDRBridge_setSoundMode(JNIEnv *env, jobject obj, jint soundMode) {
    isUpdatingConfiguration = true;
    BridgeConfig::getInstance().setSoundMode(soundMode);
    isUpdatingConfiguration = false;
}

extern "C" JNIEXPORT jboolean JNICALL
Java_fr_intuite_sdr_bridge_SDRBridge_applyConfig(
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

    if (sdrDevice) {
        try {
            if (sdrDevice->getFrequency(SOAPY_SDR_RX, 0) != static_cast<double>(centerFrequency)) {
                sdrDevice->setFrequency(SOAPY_SDR_RX, 0, static_cast<double>(centerFrequency));
            }

            if (sdrDevice->getGain(SOAPY_SDR_RX, 0) != gain) {
                sdrDevice->setGain(SOAPY_SDR_RX, 0, gain);
            }

            if (sdrDevice->getSampleRate(SOAPY_SDR_RX, 0) != static_cast<double>(sampleRate)) {
                sdrDevice->setSampleRate(SOAPY_SDR_RX, 0, static_cast<double>(sampleRate));
            }
        } catch (const std::exception &e) {
            LOGE("Error applying configuration on the SDR device: %s", e.what());
            return false;
        }

    }

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

extern "C" JNIEXPORT void JNICALL
Java_fr_intuite_sdr_bridge_SDRBridge_setPulseConfig(
        JNIEnv*, jobject,
        jfloat burstRatio,
        jfloat directFactor,
        jfloat mediumRatioDb,
        jfloat lowRatioDb,
        jfloat minDurationMs,
        jfloat maxDurationMs,
        jfloat shortWindowMs,
        jfloat longWindowMs,
        jfloat refractoryMs) {

    // Nouvelle implémentation PulseEngine : config interne, paramètres legacy ignorés.
    (void)burstRatio; (void)directFactor; (void)mediumRatioDb; (void)lowRatioDb;
    (void)minDurationMs; (void)maxDurationMs; (void)shortWindowMs;
    (void)longWindowMs; (void)refractoryMs;
    // ssbProcessor.setPulseConfig() — no-op, defaults suffice
}

extern "C" JNIEXPORT jfloat JNICALL
Java_fr_intuite_sdr_bridge_SDRBridge_getAmbientAudioEnergy(JNIEnv*, jobject) {
    return ssbProcessor.getAmbientEnergy();
}

extern "C" JNIEXPORT jfloat JNICALL
Java_fr_intuite_sdr_bridge_SDRBridge_getCurrentAudioRatio(JNIEnv*, jobject) {
    return ssbProcessor.getCurrentRatio();
}

extern "C" JNIEXPORT jintArray JNICALL
Java_fr_intuite_sdr_bridge_SDRBridge_getTunerGains(JNIEnv *env, jobject obj) {
    if (!sdrDevice) return nullptr;

    sdrDevice->listGains(SOAPY_SDR_RX, 0);

    auto range = sdrDevice->getGainRange(SOAPY_SDR_RX, 0);

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

extern "C" JNIEXPORT jobjectArray JNICALL
Java_fr_intuite_sdr_bridge_SDRBridge_getFrequencyRange(JNIEnv *env, jobject /*obj*/) {
    if (!sdrDevice) return nullptr;

    try {
        SoapySDR::RangeList ranges = sdrDevice->getFrequencyRange(SOAPY_SDR_RX, 0);
        if (ranges.empty()) {
            return nullptr;
        }

        jclass rangeClass = env->FindClass("fr/intuite/sdr/bridge/SDRRange");
        if (rangeClass == nullptr) {
            LOGE("Could not find class fr/intuite/sdr/bridge/SDRRange");
            return nullptr;
        }

        jmethodID rangeConstructor = env->GetMethodID(rangeClass, "<init>", "(JJD)V");
        if (rangeConstructor == nullptr) {
            LOGE("Could not find constructor for SDRRange");
            return nullptr;
        }

        jobjectArray rangeArray = env->NewObjectArray(ranges.size(), rangeClass, nullptr);
        if (rangeArray == nullptr) {
            LOGE("Could not create object array for SDRRange");
            return nullptr;
        }

        for (size_t i = 0; i < ranges.size(); ++i) {
            const auto& range = ranges[i];
            jobject rangeObj = env->NewObject(rangeClass, rangeConstructor,
                                              std::llround(range.minimum()),
                                              std::llround(range.maximum()),
                                              static_cast<jdouble>(range.step()));
            if (rangeObj == nullptr) {
                 LOGE("Failed to create SDRRange object");
                 return nullptr;
            }
            env->SetObjectArrayElement(rangeArray, i, rangeObj);
            env->DeleteLocalRef(rangeObj);
        }

        return rangeArray;

    } catch (const std::exception& e) {
        LOGE("Error getting frequency range: %s", e.what());
        return nullptr;
    }
}

extern "C" JNIEXPORT jlongArray JNICALL
Java_fr_intuite_sdr_bridge_SDRBridge_getSampleRatesList(JNIEnv *env, jobject /*obj*/) {
    if (!sdrDevice) return nullptr;

    try {
        std::vector<double> rates = sdrDevice->listSampleRates(SOAPY_SDR_RX, 0);
        if (rates.empty()) {
            return nullptr;
        }

        std::vector<jlong> longRates;
        longRates.reserve(rates.size());
        for (double rate : rates) {
            longRates.push_back(static_cast<jlong>(rate));
        }

        jlongArray result = env->NewLongArray(longRates.size());
        if (result == nullptr) {
            return nullptr; // out of memory error thrown
        }
        env->SetLongArrayRegion(result, 0, longRates.size(), longRates.data());
        return result;

    } catch (const std::exception& e) {
        LOGE("Error listing sample rates: %s", e.what());
        return nullptr;
    }
}