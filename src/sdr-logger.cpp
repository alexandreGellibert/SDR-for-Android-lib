#include "sdr-logger.h"
#include "sdr-bridge-internal.h"
#include <cstdio> // For vsnprintf
#include <cstdarg> // For va_list
#include <vector>

// --- JNI Globals for Logging ---
namespace {
    jobject gWrapperObj = nullptr;
    jclass javaWrapperClass = nullptr;
    jmethodID javaLogMethod = nullptr;
}

// Internal helper to get JNIEnv, attach if necessary
JNIEnv* getJniEnv(bool* attached) {
    *attached = false;
    JNIEnv *env = nullptr;
    if (sdr_bridge_internal::gJavaVM == nullptr) {
        __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "getJniEnv: gJavaVM is NULL.");
        return nullptr;
    }
    jint res = sdr_bridge_internal::gJavaVM->GetEnv((void **) &env, JNI_VERSION_1_6);
    if (res == JNI_EDETACHED) {
        if (sdr_bridge_internal::gJavaVM->AttachCurrentThread(&env, nullptr) == JNI_OK) {
            *attached = true;
        } else {
            __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "getJniEnv: Failed to attach thread to JVM.");
            return nullptr; // Failed to attach
        }
    } else if (res != JNI_OK) {
        __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "getJniEnv: Failed to get JNIEnv.");
        return nullptr; // Failed to get env
    }
    return env;
}

void logToJavaInternal(const char *message) {
    if (sdr_bridge_internal::gJavaVM == nullptr || gWrapperObj == nullptr || javaWrapperClass == nullptr || javaLogMethod == nullptr) {
        __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, "logToJavaInternal: JNI logging not fully initialized.");
        return;
    }

    bool attached;
    JNIEnv *env = getJniEnv(&attached);
    if (env == nullptr) {
        return; // Error logged in getJniEnv
    }

    jstring jMessage = env->NewStringUTF(message);
    if (jMessage != nullptr) {
        env->CallStaticVoidMethod(javaWrapperClass, javaLogMethod, jMessage);
        if (env->ExceptionOccurred()) {
            __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "Exception during Java log callback.");
            env->ExceptionDescribe();
            env->ExceptionClear();
        }
        env->DeleteLocalRef(jMessage);
    } else {
        __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "logToJavaInternal: Failed to create Java string.");
    }

    if (attached) {
        sdr_bridge_internal::gJavaVM->DetachCurrentThread();
    }
}

// Unified logging function
void unifiedLog(android_LogPriority priority, const char* file, int line, const char *fmt, ...) {

    char buffer[1024];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    // Send to native logcat
    __android_log_print(priority, LOG_TAG, "[%s:%d] %s", BASENAME(file), line, buffer);

    // Send to Java
    logToJavaInternal(buffer);
}

// Function to initialize JNI-related logging components
void initLoggerJni(JNIEnv *env, jobject thiz) {

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
        LOGE("logFromNative not found!");
    }

    env->DeleteLocalRef(localCls);
    LOGI("JNI Logger Initialized (C++ side)."); // Use the new macro for internal logging
}