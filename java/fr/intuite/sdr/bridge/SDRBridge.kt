package fr.intuite.sdr.bridge

import com.google.gson.Gson
import com.google.gson.reflect.TypeToken
import fr.intuite.sdr.DeviceCategory
import java.io.InputStreamReader

object SDRBridge  {

    // LogListener interface for handling native log messages
    fun interface LogListener {
        fun onLog(message: String)
    }

    private var logListener: LogListener? = null
    private var compatibleDevices: List<DeviceCategory> = emptyList()

    init {
        System.loadLibrary("sdr-bridge-java-soapy-lib")
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
            val inputStream = SDRBridge::class.java.classLoader?.getResourceAsStream("assets/profiles/compatible_devices.json")
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

    external fun initConfig(
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

    external fun initDongle(
        fd: Int,
        path: String?,
        driver: String
    ): Boolean

    external fun read(
        fftCallback: (FloatArray) -> Unit,
        signalStrengthCallback: (Int) -> Unit,
        peakCallback: (Float) -> Unit,
        peakNormalizedCallback: (Float) -> Unit,
        peakFrequencyCallback: (Long) -> Unit,
        pcmCallback: (ShortArray) -> Unit
    )

    external fun stopReading()

    external fun close()

    /**
     * FREQUENCY
     */
    external fun setFrequency(frequency: Long)
    external fun setFrequencyFocusRange(frequencyFocusRange: Int)

    /**
     * GAIN
     */

    external fun setGain(gain: Int)
    external fun getTunerGains(): IntArray?


    /**
     * SAMPLE RATE
     */
    external fun setSampleRate(sampleRate: Long)
    external fun setSamplesPerReading(samplesPerReading: Int)


    /**
     * REFRESH RATES
     */
    external fun setRefreshFFTMs(refreshFFTMs: Long)
    external fun setRefreshPeakMs(refreshPeakMs: Long)
    external fun setRefreshSignalStrengthMs(refreshSignalStrengthMs: Long)

    /**
     * SOUND
     */
    external fun setSoundMode(soundMode: Int)
}