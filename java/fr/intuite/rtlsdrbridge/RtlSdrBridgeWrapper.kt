package fr.intuite.rtlsdrbridge

import com.google.gson.Gson
import com.google.gson.reflect.TypeToken
import java.io.InputStreamReader

object RtlSdrBridgeWrapper {

    // LogListener interface for handling native log messages
    fun interface LogListener {
        fun onLog(message: String)
    }

    private var logListener: LogListener? = null
    private var compatibleDevices: List<DeviceCategory> = emptyList()

    init {
        System.loadLibrary("rtl-sdr-bridge-android-lib")
        loadCompatibleDevices()
    }

    // Set the LogListener
    fun setLogListener(listener: LogListener?) {
        logListener = listener
    }

    // Static method called from C++ to forward log messages
    @JvmStatic
    fun logFromNative(message: String) {
        logListener?.onLog(message) ?: android.util.Log.d("RtlSdrBridge", "C++: $message")
    }

    fun isDeviceSupported(vendorId: Int, productId: Int): DeviceCategory? {
        val vendorIdHex = String.format("0x%04x", vendorId)
        val productIdHex = String.format("0x%04x", productId)

        return compatibleDevices.find {
            it.vendorId.equals(vendorIdHex, ignoreCase = true) &&
                    it.productId.equals(productIdHex, ignoreCase = true)
        }
    }

    private fun loadCompatibleDevices() {
        try {
            val inputStream = RtlSdrBridgeWrapper::class.java.classLoader?.getResourceAsStream("assets/profiles/compatible_devices.json")
            inputStream?.use { stream ->
                InputStreamReader(stream).use { reader ->
                    val listType = object : TypeToken<List<DeviceCategory>>() {}.type
                    compatibleDevices = Gson().fromJson(reader, listType)
                    logFromNative("Loaded ${compatibleDevices.size} compatible devices.")
                }
            }
        } catch (e: Exception) {
            logFromNative("Error loading compatible devices from JSON: ${e.message}")
        }
    }

    external fun nativeInitParameters(
        centerFrequency: Long,
        sampleRate: Long,
        samplesPerReading: Int,
        freqFocusRangeKhz: Int,
        gain: Int,
        refreshFFTMs: Long,
        refreshPeakMs: Long,
        refreshSignalStrengthMs: Long,
        soundMode: Int
    ): Boolean

    external fun nativeInitRTL(
        fd: Int,
        path: String?,
        driver: String
    ): Boolean

    external fun nativeReadAsync(
        fftCallback: (FloatArray) -> Unit,
        signalStrengthCallback: (Int) -> Unit,
        peakCallback: (Float) -> Unit,
        peakNormalizedCallback: (Float) -> Unit,
        peakFrequencyCallback: (Long) -> Unit,
        pcmCallback: (ShortArray) -> Unit
    )

    external fun nativeCancelAsync()

    external fun nativeCloseRTL()

    external fun nativeSetFrequency(frequency: Long)

    external fun nativeSetSampleRate(sampleRate: Long)

    external fun nativeSetGain(gain: Int)

    external fun nativeSetSamplesPerReading(samplesPerReading: Int)

    external fun nativeSetFrequencyFocusRange(frequencyFocusRange: Int)

    external fun nativeSetRefreshFFTMs(refreshFFTMs: Long)

    external fun nativeSetRefreshPeakMs(refreshPeakMs: Long)

    external fun nativeSetRefreshSignalStrengthMs(refreshSignalStrengthMs: Long)

    external fun nativeSetSoundMode(soundMode: Int)

    external fun nativeGetTunerGains(): IntArray?
}