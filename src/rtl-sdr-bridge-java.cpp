#include <cmath>
#include <jni.h>
#include <__exception/exception.h>
#include <android/log.h>
#include <rtl-sdr-android.h>
#include <fftw3.h>
#include <cstdlib>
#include <__algorithm/max.h>
#include <vector>
#include <chrono>
#include <iostream>
#include <fstream>
#include <numeric>
#include <complex>
#include <SoapySDR/Formats.h>
#include "rtl-sdr-bridge-preferences.h"
#include "ssb/ssb_demod_opt.h"
#include <algorithm>
#include <stdexcept>
#include <cstring>
#include <atomic>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <android/log.h>

#include <SoapySDR/Modules.hpp>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Types.hpp>
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Formats.h>

#define LOG_TAG "SDR-Bridge"
#define BASENAME(file) (strrchr(file, '/') ? strrchr(file, '/') + 1 : (strrchr(file, '\\') ? strrchr(file, '\\') + 1 : file))

// --- JNI Globals ---
// Cache the JavaVM globally
static JavaVM *gJavaVM = nullptr;
static jobject gWrapperObj = nullptr;
static jclass javaWrapperClass = nullptr;
static jmethodID javaLogMethod = nullptr;
static std::mutex gJniMutex;

void initJavaVariables(JNIEnv *env, jobject thiz);
bool setupOrUpdateRxStream(uint32_t currentSampleRate);

static std::string findMainRxGain(
        SoapySDR::Device *device,
        size_t channel = 0
);

// Get the JavaVM (implementation depends on your setup)
JavaVM *getJavaVM() {
    return gJavaVM;
}

// Function to send logs to Java/Kotlin
void logToJava(const char *message) {
    if (gJavaVM == nullptr || !javaWrapperClass || !javaLogMethod) {
        __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "Cannot logToJava");
        return;
    }

    JNIEnv *env = nullptr;
    bool attached = false;

    // Check if the thread is already attached to the JVM
    jint res = gJavaVM->GetEnv((void **) &env, JNI_VERSION_1_6);
    if (res == JNI_EDETACHED) {
        // Thread is not attached, so attach it
        if (gJavaVM->AttachCurrentThread(&env, nullptr) == JNI_OK) {
            attached = true; // Mark that we attached the thread
        } else {
            __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "Failed to attach thread to JVM");
            return;
        }
    } else if (res != JNI_OK) {
        __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "Failed to get JNIEnv");
        return;
    }

    jstring jMessage = env->NewStringUTF(message);

    if (jMessage != nullptr) {
        env->CallStaticVoidMethod(javaWrapperClass, javaLogMethod, jMessage);
        if (env->ExceptionOccurred()) {
            __android_log_print(ANDROID_LOG_ERROR, LOG_TAG,
                                "Exception while calling Java Logs from C++");
            env->ExceptionDescribe();
            env->ExceptionClear();
        }
        env->DeleteLocalRef(jMessage);
    } else {
        __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "Failed to create Java string");
    }
    if (attached) {
        gJavaVM->DetachCurrentThread();
    }

}

#define LOGD(fmt, ...) do { \
    char buf[1024]; \
    snprintf(buf, sizeof(buf), "[%s:%d] " fmt, BASENAME(__FILE__), __LINE__, ##__VA_ARGS__); \
    if (strlen(buf) >= sizeof(buf) - 1) { \
        __android_log_print(ANDROID_LOG_WARN, LOG_TAG, "Log truncated"); \
    } \
    logToJava(buf); \
} while(0)

// Cache JavaVM during JNI initialization
extern "C" JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM *vm, void *reserved) {
    gJavaVM = vm; // Store the JavaVM pointer
    __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, "JNI_OnLoad called, JavaVM cached");

    try {
        std::string err = SoapySDR::loadModule("libSoapyRTLSDR.so");
        if (err.empty()) {
            __android_log_print(ANDROID_LOG_INFO, LOG_TAG, ("SoapyRTLSDR module loaded"));
        } else {
            __android_log_print(ANDROID_LOG_ERROR, LOG_TAG,
                                ("Failed to load SoapyRTLSDR: %s", err.c_str()));
        }
    } catch (const std::exception &e) {
        __android_log_print(ANDROID_LOG_ERROR, LOG_TAG,
                            ("Failed to load SoapyRTLSDR: %s", e.what()));
    }

    try {
        std::string err = SoapySDR::loadModule("libSoapyLMS7.so");
        if (err.empty()) {
            __android_log_print(ANDROID_LOG_INFO, LOG_TAG, ("SoapyLimeSDR module loaded"));
        } else {
            __android_log_print(ANDROID_LOG_ERROR, LOG_TAG,
                                ("Failed to load SoapyLimeSDR: %s", err.c_str()));
        }
    } catch (const std::exception &e) {
        __android_log_print(ANDROID_LOG_ERROR, LOG_TAG,
                            ("Failed to load SoapyLimeSDR: %s", e.what()));
    }

    return JNI_VERSION_1_6;
}

// --- Globals for SoapySDR ---
static SoapySDR::Device *sdrDevice = nullptr;
static SoapySDR::Stream *rxStream = nullptr;

struct RxChunk {
    std::vector<std::complex<float>> samples;
};
static std::mutex rx_mutex;
static std::condition_variable rx_cv;
// Put the Received Data in a Queue because SoapyRTLSDR reads are not continuous but bursty
static std::deque<RxChunk> rx_queue;
// Put the Received Data in a Buffer in case the received data are below the expected data on the UI (samplesPerReading)
static std::vector<std::complex<float>> accBuffer;

static constexpr size_t RX_QUEUE_MAX = 20;

static std::thread rx_reading_thread;
static std::thread rx_process_thread;

static std::atomic<bool> rx_running{false};
static std::atomic<bool> isUpdatingConfiguration(false);

size_t current_rx_MTU;


static jobject fftCallbackObj = nullptr; // Global reference to the callback
static jmethodID fftCallbackMethod = nullptr; // Method ID for the callback
static jobject strengthCallbackObj = nullptr; // Global reference to the callback
static jmethodID strengthCallbackMethod = nullptr; // Method ID for the callback
static jobject peakCallbackObj = nullptr; // Global reference to the callback
static jmethodID peakCallbackMethod = nullptr; // Method ID for the callback
static jobject peakNormalizedCallbackObj = nullptr; // Global reference to the callback
static jmethodID peakNormalizedCallbackMethod = nullptr; // Method ID for the callback
static jobject peakFrequencyCallbackObj = nullptr; // Global reference to the callback
static jmethodID peakFrequencyCallbackMethod = nullptr; // Method ID for the callback
static jobject pcmCallbackObj = nullptr;
static jmethodID pcmCallbackMethod = nullptr;

static bool isCenterFrequencyChanged = false;

extern "C" JNIEXPORT jboolean JNICALL
Java_fr_intuite_rtlsdrbridge_RtlSdrBridgeWrapper_nativeInitRTL(JNIEnv *env, jobject obj, jint fd,
                                                               jstring usbfsPath, jstring device) {

    initJavaVariables(env, obj);

    Preferences &prefs = Preferences::getInstance();
    if (!prefs.isInitialized()) {
        LOGD("Preferences and Settings not yet initialized");
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
        if (prefs.getGain() < 0) sdrDevice->setGainMode(SOAPY_SDR_RX, 0, true);
        else {
            sdrDevice->setGainMode(SOAPY_SDR_RX, 0, false);
            std::vector<std::string> gains = sdrDevice->listGains(SOAPY_SDR_RX, 0);
            if (!gains.empty()) sdrDevice->setGain(SOAPY_SDR_RX, 0, gains[0], prefs.getGain() / 10);
        }
    } catch (const std::exception &e) {
        LOGD("SoapySDR init failed: %s", e.what());
        return false;
    }

    env->ReleaseStringUTFChars(usbfsPath, usbfsPathCStr);
    env->ReleaseStringUTFChars(device, deviceCStr);
    return true;
}

void initJavaVariables(JNIEnv *env,
                       jobject thiz) {
    std::lock_guard<std::mutex> lock(gJniMutex);

    // Release previous refs if re-initialized
    if (gWrapperObj != nullptr) {
        env->DeleteGlobalRef(gWrapperObj);
        gWrapperObj = nullptr;
    }
    if (javaWrapperClass != nullptr) {
        env->DeleteGlobalRef(javaWrapperClass);
        javaWrapperClass = nullptr;
    }
    javaLogMethod = nullptr;

    // Cache new ones
    gWrapperObj = env->NewGlobalRef(thiz);

    jclass localCls = env->GetObjectClass(thiz);
    javaWrapperClass = (jclass) env->NewGlobalRef(localCls);

    javaLogMethod = env->GetStaticMethodID(
            javaWrapperClass,
            "logFromNative",
            "(Ljava/lang/String;)V"
    );
    if (!javaLogMethod) {
        __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "logFromNative not found!");
    }

    env->DeleteLocalRef(localCls);
}

// Global or static jfloatArray (ensure thread safety if needed)
static jfloatArray result = nullptr;
static jint resultSize = 0;

// Declaration des variables pour le son
//std::ofstream wavFile;
//std::vector<int16_t> wavBuffer;
//std::atomic<bool> stopFlag{false};
//std::mutex wavMutex;

//const int SAMPLE_RATE = 2400000; //CHECK if not already declared ???
const bool USB = true;

// Declaration of signal treatment variables
using namespace std::chrono;
// FFT and visiualization
static int integrationCount = 0;
static int integrationPeriod = 1; // Number of packets to integrate in temporal. 1 means no integration to be calculated and so far displayed
static int bufferIndex = 0;
static fftwf_complex *circularBuffer = nullptr;
static int currentSampCount = 0;
//Frequencey of interest management
static float frequence_of_interest = 432000000.0f;
static int index_f_interest = 5;
static float trackingFrequency = 0.0f;
static float lowTrackingFrequency = -3000.0f; //inf tol -3khz
static float upperTrackingFrequency = 1000.0f; //sup tol +1khz
static std::vector<float> maxPeakAndFrequency;
static std::chrono::steady_clock::time_point timeOfLastMaxPeak;
static std::chrono::steady_clock::time_point timeOfLastMaxPeakUpdate;
//Peak variables
static float refMagnitude = 50000.0f;
static float refPower = refMagnitude * refMagnitude;
//Signal strength variables
std::vector<time_point<high_resolution_clock>> signalTimeTable;
static std::vector<int> signalStrengthIndexBuffer;
static int signalStrengthIndexRemanance = 3;
static int indexsignalStrengthIndexBuffer = 0;
static int signalWeak = 1;
static int signalMedium = 5;
static int signalStrong = 20;
static int remananceWeak = 0;
static int remananceMedium = 0;
static int remananceStrong = 0;
//Noise evaluation
static std::vector<float> noisePercentileBuffer;
static std::vector<float> noiseMedianBuffer;
static int noiseBufferSize = 1;
static int indexNoiseBuffer = 0;

static float noiseSigmaMin = 100;
static float noiseSigmaMax = 0;
static float peakNormalizedMax = 0;

//Level 1 variables
static std::vector<float> peakBuffer;
static std::vector<float> peakNormalizedBuffer;
static int peakRemanance = 5;
static int indexPeakBuffer = 0;

static int confirmation = 1;
static int peakConfirmed = 0;

static std::vector<float> thresholdBuffer(confirmation + 1, 0.0f);
static int indexThreshold = 0;

static float maxThreshold = 0;
static float maxThresholdConfirmed = 0;

static float lvl1Ratio = 0.0f;
static int loopNB = 0;
static int lvl1NB = 0;

//SSB part
static bool sensitivityBoost = true;
static float gain = 1.2f;
static std::vector<int16_t> pcm;
static int mode = 1;
static bool pulse = false;


//THREAD PART
// For SSB processing thread
struct SSB_Data {
    std::vector<std::complex<float>> iq;
    uint32_t sampleRate;
};

static std::queue<SSB_Data> ssb_queue;
static std::mutex ssb_mutex;
static std::condition_variable ssb_cv;
static std::thread ssb_worker;
static std::atomic<bool> ssb_worker_running(false);

void ssb_processing_thread() {
    JNIEnv *env;
    JavaVM *javaVM = getJavaVM();
    if (javaVM == nullptr) {
        __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "SSB Thread: JavaVM is NULL.");
        return;
    }

    // Attach the current thread to the JVM
    jint res = javaVM->AttachCurrentThread(&env, nullptr);
    if (res != JNI_OK) {
        __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "SSB Thread: Failed to attach to JVM.");
        return;
    }

    while (true) {
        std::unique_lock<std::mutex> lock(ssb_mutex);
        ssb_cv.wait(lock, [] { return !ssb_queue.empty() || !ssb_worker_running; });

        if (!ssb_worker_running && ssb_queue.empty()) {
            break; // Exit if worker is stopped and queue is empty
        }

        if (ssb_queue.empty()) {
            continue; // Spurious wakeup
        }

        SSB_Data data = ssb_queue.front();
        ssb_queue.pop();
        lock.unlock();

        mode = Preferences::getInstance().getSoundMode();

        processSSB_opt(data.iq, data.sampleRate, USB, pcm, pulse, mode);

        if (pcmCallbackObj != nullptr && !pcm.empty()) {
            jshortArray pcmArray = env->NewShortArray(pcm.size());
            if (pcmArray != nullptr) {
                env->SetShortArrayRegion(pcmArray, 0, pcm.size(), pcm.data());
                env->CallVoidMethod(pcmCallbackObj, pcmCallbackMethod, pcmArray);
                env->DeleteLocalRef(pcmArray);
            }
        }

        // TO DO - DETECT PULS IN SOUND
        //int signalEvalPulse = 0 ;
        //if (pulse){
        //    signalEvalPulse = 2 ;
        //}
        //env->CallVoidMethod(strengthCallbackObj, strengthCallbackMethod, signalEvalPulse);
    }

    // Detach the current thread from the JVM
    javaVM->DetachCurrentThread();
    __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, "SSB Thread: Detached from JVM and finished.");
}


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

    uint32_t centerFrequency = Preferences::getInstance().getCenterFrequency();
    uint32_t sampleRate = Preferences::getInstance().getSampleRate();
    //-----
    //0. THREAD SSB PART
    //-----
    // Enqueue data for SSB processing in a separate thread
    {

        std::vector<std::complex<float>> iq_copy(buf, buf + len);

        std::lock_guard<std::mutex> lock(ssb_mutex);

        if (!ssb_queue.empty()) {
            std::queue<SSB_Data>().swap(ssb_queue);
        }

        ssb_queue.push({
            std::move(iq_copy),
            sampleRate
        });
    }
    ssb_cv.notify_one();

    // Traitement direct (sans thread)
    //std::vector<int16_t> pcm;
    //processSSB(buffer, len, sampleRate, USB, pcm, 5.0f);

    // Traitement direct (sans thread) Version Mistral AI de publi
    //if (pcmCallbackObj != nullptr && !pcm.empty()) {
    //    jshortArray pcmArray = env->NewShortArray(pcm.size());
    //    env->SetShortArrayRegion(pcmArray, 0, pcm.size(), reinterpret_cast<const jshort*>(pcm.data()));
    //    env->CallVoidMethod(pcmCallbackObj, pcmCallbackMethod, pcmArray);
    //    env->DeleteLocalRef(pcmArray);
    //}

    //Traitement direct (sans thread) Vieille version d'envoi
    //if (pcmCallbackObj != nullptr && !pcm.empty()) {
    //    jshortArray pcmArray = env->NewShortArray(pcm.size());
    //    if (pcmArray != nullptr) {
    //        env->SetShortArrayRegion(pcmArray, 0, pcm.size(), pcm.data());
    //        env->CallVoidMethod(pcmCallbackObj, pcmCallbackMethod, pcmArray);
    //        env->DeleteLocalRef(pcmArray);
    //    }
    //}

    //-----
    //1. Convert I/Q and prepare tables that depend on sampCount and store in circular buffer
    //-----

    uint32_t sampCount = len;

    fftwf_complex *signal = fftwf_alloc_complex(sampCount);
    fftwf_complex *fft_signal = fftwf_alloc_complex(sampCount);

    if (currentSampCount != sampCount) {
        fftwf_free(circularBuffer);
        currentSampCount = sampCount;
        circularBuffer = fftwf_alloc_complex(currentSampCount * integrationPeriod);
    }

    // ---- I/Q COPY ----
    for (uint32_t i = 0; i < sampCount; i++) {
        signal[i][0] = buf[i].real();
        signal[i][1] = buf[i].imag();

        circularBuffer[(bufferIndex * sampCount) + i][0] = signal[i][0];
        circularBuffer[(bufferIndex * sampCount) + i][1] = signal[i][1];
    }

    bufferIndex = (bufferIndex + 1) % integrationPeriod;
    integrationCount++;

    // Integrate the buffer
    //fftwf_complex *signal_integrated = fftwf_alloc_complex(sampCount);
    //fftwf_complex *fft_signal_integrated = fftwf_alloc_complex(sampCount);

    // Remettre le tableau à zéro
    //for (int i = 0; i < sampCount; ++i) {
    //    signal_integrated[i][0] = 0.0f; // Partie réelle
    //    signal_integrated[i][1] = 0.0f; // Partie imaginaire
    //}

    //-----
    //2. INTEGRATE signal and produce signal integrated
    //-----

    // Perform integration
    //TO DO : integrate on timelapse and not on number of loop : if perf is different chat happens ?
    //if (integrationCount >= integrationPeriod) {
    //    for (int i = 0; i < sampCount; i++) {
    //        for (int j = 0; j < integrationPeriod; j++) {
    //            signal_integrated[i][0] += circularBuffer[j * sampCount + i][0];
    //            signal_integrated[i][1] += circularBuffer[j * sampCount + i][1];
    //        }
    //        //LOGD("passe dans la boucle integration");
    //        signal_integrated[i][0] /= integrationPeriod;
    //        signal_integrated[i][1] /= integrationPeriod;
    //    }
    //}

    //-----
    //3. PERFORM FFT on signal and integrated signal
    //-----

    // Perform FFT on signal
    fftwf_plan plan1 = fftwf_plan_dft_1d(sampCount, signal, fft_signal, FFTW_FORWARD,
                                         FFTW_ESTIMATE);
    fftwf_execute(plan1);
    // AND clean up
    fftwf_destroy_plan(plan1);


    // Perform FFT on signal integrated
    //fftwf_plan plan2 = fftwf_plan_dft_1d(sampCount, signal_integrated, fft_signal_integrated,
    //                                     FFTW_FORWARD, FFTW_ESTIMATE);
    //fftwf_execute(plan2);
    // AND clean up
    //fftwf_destroy_plan(plan2);

    //-----
    //4. COMPUTE POWER SPECTRUM on both inst and integrated
    //-----

    //Define power inst and power integrated
    float power[sampCount];
    float power_shifted[sampCount];
    //float power_integrated[sampCount];
    //float power_integrated_shifted[sampCount];

    //Compute power spectrum on signal AND on signal integrated : power = real^2 + imag^2
    for (int i = 0; i < sampCount; i++) {
        power[i] = fft_signal[i][0] * fft_signal[i][0] + fft_signal[i][1] * fft_signal[i][1];
    }
    //for (int i = 0; i < sampCount; i++) {
    //    power_integrated[i] = fft_signal_integrated[i][0] * fft_signal_integrated[i][0] +
    //                          fft_signal_integrated[i][1] * fft_signal_integrated[i][1];
    //}
    //FREE
    fftwf_free(signal);
    //fftwf_free(signal_integrated);
    fftwf_free(fft_signal);
    //fftwf_free(fft_signal_integrated);

    // Shift the power spectrum to center it (FFT shift)
    int half = sampCount / 2;
    for (int i = 0; i < half; i++) {
        power_shifted[i] = power[i + half]; // Negative frequencies
        power_shifted[i + half] = power[i]; // Positive frequencies
    }
    //for (int i = 0; i < half; i++) {
    //    power_integrated_shifted[i] = power_integrated[i + half]; // Negative frequencies
    //    power_integrated_shifted[i + half] = power_integrated[i]; // Positive frequencies
    //}

    if (result == nullptr || resultSize != sampCount) {
        if (result != nullptr) {
            env->DeleteGlobalRef(result);
            result = nullptr;
        }
        jfloatArray local = env->NewFloatArray(sampCount);
        if (local == nullptr) {
            LOGD("Failed to allocate float array (%d)", sampCount);
            return;
        }

        // Make it a global reference to persist across calls
        result = (jfloatArray) env->NewGlobalRef(local);
        env->DeleteLocalRef(local);

        resultSize = sampCount;
    }

    //-----
    //5. SEND POWER INTEGRATED to JAVA
    //-----

    //NEXT : give power in db
    env->SetFloatArrayRegion(result, 0, sampCount, power_shifted);
    // Call the Kotlin fftCallback
    env->CallVoidMethod(fftCallbackObj, fftCallbackMethod, result);

    //-----
    //6. SIGNAL STRENGTH EVALUATION on BANDWITH
    //-----

    //-----
    // 6.1 BANDWITH definition for PEAK EVALUATION
    //-----
    // Define subrange = wide window = WW - manageable through UI
    float freqPerBin = static_cast<float>(sampleRate) / sampCount;  // Hz per bin
    float lowerWWBound = centerFrequency - Preferences::getInstance().getFreqFocusRangeKhz() *
                                           1000.0f;  // fc - 5 kHz
    float upperWWBound = centerFrequency + Preferences::getInstance().getFreqFocusRangeKhz() *
                                           1000.0f;  // fc + 5 kHz

    // Find bin indices for the subrange (after shifting)
    int lowerWWIndex = static_cast<int>((lowerWWBound - (centerFrequency - sampleRate / 2)) /
                                        freqPerBin);
    int upperWWIndex = static_cast<int>((upperWWBound - (centerFrequency - sampleRate / 2)) /
                                        freqPerBin);
    lowerWWIndex = std::max(0, lowerWWIndex);
    upperWWIndex = sampCount - 1 > upperWWIndex ? upperWWIndex : sampCount - 1;
    int WW_size = upperWWIndex - lowerWWIndex + 1;

    //-----
    // 6.2 INITIALISATION of variables for SIGNAL PEAK MEASURE
    //-----
    //Init of variables for peak evaluation
    float totalPower = 0.0f;
    float total_integrated_Power = 0.0f;
    float dB = -130.0f; //temp value for max research
    float peakDb = -130.0f; //peak on integrated signal
    float peakNormalized = 0.0f; //peak normalized
    int index_peak = 0;
    float WW_average_power = -130.0f; //TO DO remonter en static si on veut moyenner un peu ???
    float WW_average_integrated_power = -130.0f;
    float power_shifted_DB[WW_size];
    //Prepare peak buffer for remanance - target of remanance is to display peak on screen and keep highest value a bit stuck so yhat the eye get it
    if (peakBuffer.empty()) {
        peakBuffer.assign(peakRemanance, -130.0);
    }
    //Prepare peak NORMALIZED for remanance
    if (peakNormalizedBuffer.empty()) {
        peakNormalizedBuffer.assign(peakRemanance, 0.0);
    }


    //-----
    //6.3 SIGNAL PEAK MEASURE - look for signal peak (in case of signal strong enough)
    //-----

    //Run in a loop on subrange to find peak in subrange
    for (int i = lowerWWIndex; i <= upperWWIndex; i++) {
        dB = 10 * log10(power_shifted[i] / refPower);
        if (dB > peakDb) {
            peakDb = dB;
            index_peak = i - lowerWWIndex;
        }

        totalPower += 10 * log10(power_shifted[i] / refPower);
        power_shifted_DB[i - lowerWWIndex] =
                10 * log10(power_shifted[i] / refPower);
        totalPower += 10 * log10(power_shifted[i] / refPower);
    }
    WW_average_power = (totalPower / WW_size); // average defined on signal not integrated
    //WW_average_integrated_power = (total_integrated_Power / WW_size);

    //Stock PEAK in buffer for remanance
    peakBuffer[indexPeakBuffer] = peakDb;

    //-----
    //6.4 NOISE EVALUATION - statistical data collection for Peak normalization and trigger def
    //-----

    //Do not take all the data to have some out of peak values

    int indexOfMedian = static_cast<int>(std::floor(
            static_cast<double>(WW_size) * 0.5)); //Middle ;)
    //LOGD("index = %d", index) ;
    std::sort(power_shifted_DB, power_shifted_DB + WW_size);
    //float noisePercentile = power_shifted_DB[indexOfPercentile] ;
    float noiseMedian = power_shifted_DB[indexOfMedian];
    float gapTable[WW_size];
    for (int i = 0; i < WW_size; i++) {
        gapTable[i] = std::fabs(power_shifted_DB[i] - noiseMedian);
    }
    std::sort(gapTable, gapTable + WW_size);

    //LOGD("valeur percentilebuffer = %f", percentileBuffer[indexPercentileBuffer]);

    float noiseSigma = 1.4816 * gapTable[indexOfMedian];

    //-----
    //6.5 SIGNAL PEAK NORMALIZED MEASURE -
    //-----
    //Peak normalized up of noise mediane
    peakNormalized = peakDb - noiseMedian;
    //Stock PEAK normalized in buffer for remanance
    peakNormalizedBuffer[indexPeakBuffer] = peakNormalized;
    //increment index
    indexPeakBuffer = (indexPeakBuffer + 1) % peakRemanance;

    //TEMP CODE FOR NOISe ANALYSIS
    //if (noiseSigma<noiseSigmaMin){
    //    noiseSigmaMin = noiseSigma ;
    //}
    //if (noiseSigma>noiseSigmaMax){
    //    noiseSigmaMax = noiseSigma ;
    //}

    // PUT THRESHOLD CALCULATED IN BUFFER
    thresholdBuffer[indexThreshold] = peakNormalized / noiseSigma;

    // CALCULATE peakNormMax
    if (peakNormalized > peakNormalizedMax) {
        peakNormalizedMax = peakNormalized;
    }

    // CALCULATE MAX THRESHOLD
    if (thresholdBuffer[indexThreshold] > maxThreshold) {
        maxThreshold = thresholdBuffer[indexThreshold];
    }

    // CALCULATE MAX CONFIRMED THRESHOLD
    float minVal = *min_element(thresholdBuffer.begin(), thresholdBuffer.end());
    if (minVal > maxThresholdConfirmed) {
        maxThresholdConfirmed = minVal;
    }

    indexThreshold = (indexThreshold + 1) % (confirmation + 1);

    //CHECK
    if (integrationCount % 300 == 0) {
        LOGD("le seuil max est = %f", maxThreshold);
        LOGD("le seuil max CONFIRMED est = %f", maxThresholdConfirmed);
        maxThreshold = 0;
        maxThresholdConfirmed = 0;
    }

    //-----
    //6.6 INIT of tracking frequency, and peaks and its frequencies
    //-----
    if (trackingFrequency == 0.0f) {
        trackingFrequency = centerFrequency;
    }
    if (isCenterFrequencyChanged) {
        trackingFrequency = centerFrequency;
        isCenterFrequencyChanged = false;
    }
    if (maxPeakAndFrequency.empty()) {
        maxPeakAndFrequency = {-130.0f, static_cast<float>(centerFrequency)};
    }

    //-----
    //6.7 DEFINITION OF SIGNAL DETECTED - signalStrengthIndex get value 0 : no signal ; 1 ; weak signal may be no signal, remanance short ; medium signal, remanance normal ; strong signal, remanance normal.
    //-----
    //init of Signal Strength
    int signalEval = 0;
    loopNB = loopNB + 1;


    //First condition for signal strong
    if (peakDb > noiseMedian + 4.0 * noiseSigma) {
        if (peakConfirmed >= confirmation) {
            if (peakDb > maxPeakAndFrequency[0]) {
                maxPeakAndFrequency[0] = peakDb;
                maxPeakAndFrequency[1] = index_peak * freqPerBin +
                                         lowerWWBound; //définir les règles de l'autocalibration !!!
                timeOfLastMaxPeak = std::chrono::steady_clock::now();
                LOGD("Level 1 déclenche. frequence = %f", trackingFrequency);
            }
            signalEval = signalStrong + 1;
            LOGD("Level 1 déclenche. peak = %f", peakDb);
            lvl1NB = lvl1NB + 1;
            lvl1Ratio = float(lvl1NB) / float(loopNB);
            //LOGD("Level 1 déclenche. valeur noiseMedian = %f", noiseMedian);
            //LOGD("Level 1 déclenche. valeur noiseSigma = %f", noiseSigma);
        } else {
            peakConfirmed = peakConfirmed + 1;
            //LOGD("peak confirmed = %d", peakConfirmed);
        }
    } else {
        peakConfirmed = 0;
        //LOGD("peak confirmed = %d", peakConfirmed);
    }


    if (integrationCount % 300 == 0) {
        LOGD("le ratio de déclenchement est = %f", lvl1Ratio);
    }

    auto maintenant = std::chrono::steady_clock::now();
    auto delaiDepuisPeak = std::chrono::duration_cast<std::chrono::milliseconds>(
            maintenant - timeOfLastMaxPeak);
    //auto delai2 = std::chrono::duration_cast<std::chrono::milliseconds>(timeOfLastMaxPeak - timeOfLastMaxPeakUpdate);
    auto delaiDepuisPeak_ms = delaiDepuisPeak.count();

    //PB : déclenche alors que pas de peak !!!
    if (timeOfLastMaxPeakUpdate < timeOfLastMaxPeak && delaiDepuisPeak_ms > 300) {
        trackingFrequency = maxPeakAndFrequency[1];
        timeOfLastMaxPeakUpdate = std::chrono::steady_clock::now();
        maxPeakAndFrequency[0] = -130.0f;
        LOGD("MAJ frequence = %f", trackingFrequency);
    }

    //tableau des occurences des signaux avec valeur et timestamp. weak si repet faible. medium si repet faible et régulier ou si repet fort. fort si repet fort et régulier.
    //A optimiser, pas besoin de passer partout
    if (signalTimeTable.size() == 0) {
        signalTimeTable.push_back(high_resolution_clock::now());
        signalTimeTable.push_back(high_resolution_clock::now());
    }

    //signal strength calculation
    int signalStrengthIndex = 0;

    if (remananceStrong > 0) {
        signalStrengthIndex = 3;
        remananceStrong = std::max(0, remananceStrong - 1);
        remananceMedium = std::max(0, remananceMedium - 1);
        remananceWeak = std::max(0, remananceWeak - 1);
    } else {
        // Calculer les durées
        auto duration1 = high_resolution_clock::now() - signalTimeTable[0];
        auto duration2 = signalTimeTable[0] - signalTimeTable[1];

        // Convertir les durées en un type numérique (par exemple, en millisecondes)
        auto duration1_ms = duration_cast<milliseconds>(duration1).count();
        auto duration2_ms = duration_cast<milliseconds>(duration2).count();

        // Calculer la valeur absolue de la différence
        long long abs_diff = std::abs(duration1_ms - duration2_ms);

        if (signalEval >= signalStrong ||
            signalMedium <= signalEval && signalEval < signalStrong && abs_diff < 300) {
            signalStrengthIndex = 3;
            remananceStrong = 3;
            if (duration1_ms > 666) {
                signalTimeTable.push_back(high_resolution_clock::now());
            }
        } else {
            if (remananceMedium > 0) {
                signalStrengthIndex = 2;
                remananceMedium = std::max(0, remananceMedium - 1);
                remananceWeak = std::max(0, remananceWeak - 1);
            } else {

                // Calculer les durées
                auto duration1 = high_resolution_clock::now() - signalTimeTable[0];
                auto duration2 = signalTimeTable[0] - signalTimeTable[1];

                // Convertir les durées en un type numérique (par exemple, en millisecondes)
                auto duration1_ms = duration_cast<milliseconds>(duration1).count();
                auto duration2_ms = duration_cast<milliseconds>(duration2).count();

                // Calculer la valeur absolue de la différence
                long long abs_diff = std::abs(duration1_ms - duration2_ms);

                //TO DO rajouter duration1_ms > 300 pour pas chercher la confirmation
                if (signalEval >= signalMedium ||
                    signalWeak <= signalEval && signalEval < signalMedium && abs_diff < 300) {
                    signalStrengthIndex = 2;
                    remananceMedium = 2;
                    if (duration1_ms > 666) {
                        signalTimeTable.push_back(high_resolution_clock::now());
                    }
                } else {
                    if (remananceWeak > 0) {
                        signalStrengthIndex = 1;
                        remananceWeak = std::max(0, remananceWeak - 1);

                    } else {
                        if (signalEval >= signalWeak) {
                            remananceWeak = 1;
                            signalStrengthIndex = 1;
                            if (duration1_ms > 666) {
                                signalTimeTable.push_back(high_resolution_clock::now());
                            }
                        } else {
                            remananceStrong = std::max(0, remananceStrong - 1);
                            remananceMedium = std::max(0, remananceMedium - 1);
                            remananceWeak = std::max(0, remananceWeak - 1);
                        }

                    }
                }
            }
        }
    }

    //Initialization of signalStrengthIndex table
    if (signalStrengthIndexBuffer.empty()) {
        signalStrengthIndexBuffer.assign(signalStrengthIndexRemanance, 0);
    }
    //Fulfill of table
    signalStrengthIndexBuffer[indexsignalStrengthIndexBuffer] = signalStrengthIndex;
    indexsignalStrengthIndexBuffer =
            (indexsignalStrengthIndexBuffer + 1) % signalStrengthIndexRemanance;

    //Calculate strenghtIndex to be sent
    auto signalStrengthIndexMaxIter = std::max_element(signalStrengthIndexBuffer.begin(),
                                                       signalStrengthIndexBuffer.end());
    int signalStrengthIndexSent = *signalStrengthIndexMaxIter;
    //Calculate if we have not quickly some stronger signal before sending "Signal probable"
    int sum = std::accumulate(signalStrengthIndexBuffer.begin(), signalStrengthIndexBuffer.end(),
                              0);
    double signalStrengthIndexMean = static_cast<double>(sum) / signalStrengthIndexBuffer.size();
    if (*signalStrengthIndexMaxIter == 1 && signalStrengthIndexMean < 0.8) {
        int signalStrengthIndexSent = 0;
    }

    //if (integrationCount%100==0){
    //    LOGD("time tous les 100 tours");
    //}

    // Send signal typologyto Java
    env->CallVoidMethod(strengthCallbackObj, strengthCallbackMethod, signalStrengthIndexSent);
    // Stick the peak if goes down
    auto peakRemanantMaxIter = std::max_element(peakBuffer.begin(), peakBuffer.end());
    float peakRemanantMax = *peakRemanantMaxIter;
    // IDEM for peakNormalized
    auto peakNormalizedMaxIter = std::max_element(peakNormalizedBuffer.begin(),
                                                  peakNormalizedBuffer.end());
    float peakNormalizedMax = *peakNormalizedMaxIter;
    // Send peak of signal to Java
    env->CallVoidMethod(peakCallbackObj, peakCallbackMethod, peakRemanantMax);
    // Send normalized peak of signal to Java
    env->CallVoidMethod(peakNormalizedCallbackObj, peakNormalizedCallbackMethod, peakNormalizedMax);
    // Send new frequency tracking to Java
    env->CallVoidMethod(peakFrequencyCallbackObj, peakFrequencyCallbackMethod,
                        static_cast<long>(std::round(trackingFrequency)));
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
                    Preferences::getInstance().getSamplesPerReading();

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
                while (accBuffer.size() >= samplesPerReading_to_process)
                {
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
                Preferences::getInstance().getSampleRate();
        size_t uiSamples =
                Preferences::getInstance().getSamplesPerReading();

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
Java_fr_intuite_rtlsdrbridge_RtlSdrBridgeWrapper_nativeReadAsync(
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
    // Start SSB worker thread if not already running
    if (!ssb_worker_running) {
        ssb_worker_running = true;
        ssb_worker = std::thread(ssb_processing_thread);
    }

    if (!setupOrUpdateRxStream(Preferences::getInstance().getSampleRate())) {
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
Java_fr_intuite_rtlsdrbridge_RtlSdrBridgeWrapper_nativeCancelAsync(JNIEnv *env, jobject thiz) {
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
    if (ssb_worker_running) {
        ssb_worker_running = false;
        ssb_cv.notify_one();
        if (ssb_worker.joinable()) {
            ssb_worker.join();
        }
    }
}

// Close: Cleanup device
extern "C" JNIEXPORT void JNICALL
Java_fr_intuite_rtlsdrbridge_RtlSdrBridgeWrapper_nativeCloseRTL(JNIEnv *env, jobject obj) {
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
    if (ssb_worker_running) {
        ssb_worker_running = false;
        ssb_cv.notify_one();
        if (ssb_worker.joinable()) {
            ssb_worker.join();
        }
    }

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
Java_fr_intuite_rtlsdrbridge_RtlSdrBridgeWrapper_nativeSetFrequency(JNIEnv *env, jobject obj,
                                                                    jlong frequency) {
    isUpdatingConfiguration = true;
    Preferences::getInstance().setCenterFrequency(frequency);

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
        isCenterFrequencyChanged = true;
    }
    catch (const std::exception &e) {
        LOGD("ERROR: Failed to set frequency %lld : %s",
             frequency, e.what());
    }

    isUpdatingConfiguration = false;
}

extern "C" JNIEXPORT void JNICALL
Java_fr_intuite_rtlsdrbridge_RtlSdrBridgeWrapper_nativeSetSampleRate(JNIEnv *env, jobject obj,
                                                                     jlong sampleRate) {
    isUpdatingConfiguration = true;
    Preferences::getInstance().setSampleRate(sampleRate);

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
Java_fr_intuite_rtlsdrbridge_RtlSdrBridgeWrapper_nativeSetGain(JNIEnv *env, jobject obj,
                                                               jint gain) {
    isUpdatingConfiguration = true;
    Preferences::getInstance().setGain(gain);

    if (!sdrDevice) {
        isUpdatingConfiguration = false;
        return;
    }

    auto gains = sdrDevice->listGains(SOAPY_SDR_RX, 0);

    for (const auto &g : gains)
    {
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
            sdrDevice->setGain(SOAPY_SDR_RX, 0, "PGA", 20);
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
Java_fr_intuite_rtlsdrbridge_RtlSdrBridgeWrapper_nativeSetSamplesPerReading(JNIEnv *env,
                                                                            jobject obj,
                                                                            jint samplesPerReading) {
    isUpdatingConfiguration = true;
    Preferences::getInstance().setSamplesPerReading(samplesPerReading);
    LOGD("New Samples Per Reading %d", samplesPerReading);
    isUpdatingConfiguration = false;
}

extern "C" JNIEXPORT void JNICALL
Java_fr_intuite_rtlsdrbridge_RtlSdrBridgeWrapper_nativeSetFrequencyFocusRange(JNIEnv *env,
                                                                              jobject obj,
                                                                              jint frequencyFocusRange) {
    isUpdatingConfiguration = true;
    Preferences::getInstance().setSamplesPerReading(frequencyFocusRange);
    LOGD("New Frequency Focus Range %ld", frequencyFocusRange);
    isUpdatingConfiguration = false;
}

extern "C" JNIEXPORT void JNICALL
Java_fr_intuite_rtlsdrbridge_RtlSdrBridgeWrapper_nativeSetRefreshFFTMs(JNIEnv *env, jobject obj,
                                                                       jlong refreshFFTMs) {
    isUpdatingConfiguration = true;
    Preferences::getInstance().setRefreshFFTMs(refreshFFTMs);
    LOGD("New Refresh FFT period in ms %ld", refreshFFTMs);
    isUpdatingConfiguration = false;
}

extern "C" JNIEXPORT void JNICALL
Java_fr_intuite_rtlsdrbridge_RtlSdrBridgeWrapper_nativeSetRefreshPeakMs(JNIEnv *env, jobject obj,
                                                                        jlong refreshPeakMs) {
    isUpdatingConfiguration = true;
    Preferences::getInstance().setRefreshPeakMs(refreshPeakMs);
    LOGD("New Refresh Peak period in ms %ld", refreshPeakMs);
    isUpdatingConfiguration = false;
}

extern "C" JNIEXPORT void JNICALL
Java_fr_intuite_rtlsdrbridge_RtlSdrBridgeWrapper_nativeSetRefreshSignalStrengthMs(JNIEnv *env,
                                                                                  jobject obj,
                                                                                  jlong refreshSignalStrengthMs) {
    isUpdatingConfiguration = true;
    Preferences::getInstance().setRefreshSignalStrengthMs(refreshSignalStrengthMs);
    LOGD("New Refresh Signal Strength %ld", refreshSignalStrengthMs);
    isUpdatingConfiguration = false;
}





extern "C" JNIEXPORT void JNICALL
Java_fr_intuite_rtlsdrbridge_RtlSdrBridgeWrapper_nativeSetSoundMode(JNIEnv *env, jobject obj,
                                                                    jint soundMode) {
    isUpdatingConfiguration = true;
    Preferences::getInstance().setSoundMode(soundMode);
    isUpdatingConfiguration = false;
}

extern "C" JNIEXPORT jboolean JNICALL
Java_fr_intuite_rtlsdrbridge_RtlSdrBridgeWrapper_nativeInitParameters(
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
    Preferences &prefs = Preferences::getInstance();
// Initialize Preferences
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
    return true;
}

extern "C" JNIEXPORT jintArray JNICALL
Java_fr_intuite_rtlsdrbridge_RtlSdrBridgeWrapper_nativeGetTunerGains(JNIEnv *env, jobject obj) {
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

static std::string findMainRxGain(
        SoapySDR::Device *device,
        size_t channel
) {
    if (!device) return "";

    const auto gains = device->listGains(SOAPY_SDR_RX, channel);

    // Only interested in Main Gain Settings
    // TUNER: Case RTL-SDR, only this gain is of interest
    // LNA: Lime SDR TODO handle cases for Gain TIA and PGA?
    // RF: Airspy
    // HackRF: VGA
    static const std::vector<std::string> preferred = {
            "TUNER",
            "LNA",
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

size_t calculateOptimalBufflen(uint32_t sampleRateHz, const std::string& format);
size_t calculateOptimalBuffers(uint32_t sampleRateHz);

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

        LOGD("Stream (re)setup OK | bufflen=%zu | buffers=%d | MTU=%d", bufflen, numBuffers, current_rx_MTU);

        return true;
    }
    catch (const std::exception& e) {
        // Log exception
        if (rxStream != nullptr) {
            sdrDevice->closeStream(rxStream);
            rxStream = nullptr;
        }
        return false;
    }
}

// Returns optimal bufflen in bytes (multiple of 512, clamped). Size of each individual USB transfer
// Target: ~20–50 ms of data per USB buffer for low latency without too much overhead
size_t calculateOptimalBufflen(uint32_t sampleRateHz, const std::string& format) {
    if (sampleRateHz <= 0.0) return 16384;  // fallback for invalid rate

    constexpr double targetFillTimeLow  = 0.025;  // 25 ms — good compromise for tracking
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
    bufflen = std::max<size_t>(4096,   bufflen);   // min ~10 ms @ 100 kS/s
    bufflen = std::min<size_t>(262144, bufflen);   // don't exceed typical default

    return bufflen;
}

// Returns number of ring buffers (more = more headroom, but higher worst-case latency)
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
