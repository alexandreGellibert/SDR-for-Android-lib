#ifndef SDR_LOGGER_H
#define SDR_LOGGER_H

#include <jni.h>
#include <android/log.h> // Still needed for ANDROID_LOG_... constants and __android_log_print signature
#include <mutex>
#include <cstring>
#include <functional> // For std::function

// Utility for getting basename of a file
#define BASENAME(file) (strrchr(file, '/') ? strrchr(file, '/') + 1 : (strrchr(file, '\\') ? strrchr(file, '\\') + 1 : file))

// Function to send logs to Java/Kotlin AND native logcat
void unifiedLog(android_LogPriority priority, const char* file, int line, const char *fmt, ...);

// Logging Macros
#define LOG_TAG "SDR-Bridge" // Define LOG_TAG here for use by __android_log_print

#define LOGD(fmt, ...) unifiedLog(ANDROID_LOG_DEBUG, __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define LOGI(fmt, ...) unifiedLog(ANDROID_LOG_INFO, __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...) unifiedLog(ANDROID_LOG_WARN, __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define LOGE(fmt, ...) unifiedLog(ANDROID_LOG_ERROR, __FILE__, __LINE__, fmt, ##__VA_ARGS__)

// Function to initialize JNI-related logging components
void initLoggerJni(JNIEnv *env, jobject thiz);

#endif // SDR_LOGGER_H