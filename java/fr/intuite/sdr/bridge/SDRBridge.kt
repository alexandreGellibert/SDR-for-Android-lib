package fr.intuite.sdr.bridge

import com.google.gson.Gson
import com.google.gson.reflect.TypeToken
import fr.intuite.sdr.DeviceCategory
import java.io.InputStreamReader

data class SDRRange(val minimum: Long, val maximum: Long, val step: Double)

enum class Driver(val key: String) {
    RTLSDR("rtlsdr"),
    LIME("lime"),
    AIRSPY("airspy"),
    AIRSPYHF("airspyhf");

    companion object {
        fun fromString(key: String?): Driver =
            entries.find { it.key.equals(key, ignoreCase = true) } ?: RTLSDR
    }
}

data class SDRConfig(
    val centerFrequency: Long = 430000000L,
    // Samples per reading: Multiple of 512 is recommended
    val samplesPerReading: Int = 16384,
    // Default sample rate should depend on the connected device
    val sampleRate: Long = 2500000,
    // Default gain should depend on the connected device
    val gain: Int = 10,
    val freqFocusRangeKhz: Int = 5,
    val refreshFFTMs: Long = 50L,
    val refreshPeakMs: Long = 200L,
    val refreshSignalStrengthMs: Long = 30L,
    // Sound mode: 0 mute, 1 normal, 2 loud
    val soundMode: Int = 1
)

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

    external fun setPulseConfig(
        burstRatio:    Float,
        minDurationMs: Float,
        maxDurationMs: Float,
        shortWindowMs: Float,
        longWindowMs:  Float,
        refractoryMs:  Float
    )

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

    fun initDongle(fd: Int, path: String?, driver: Driver): Boolean {
        return initDongle(fd, path, driver.key)
    }

    fun applyConfig(sdrConfig: SDRConfig): Boolean{
        return applyConfig(
            sdrConfig.centerFrequency,
            sdrConfig.sampleRate,
            sdrConfig.samplesPerReading,
            sdrConfig.freqFocusRangeKhz,
            sdrConfig.gain,
            sdrConfig.refreshFFTMs,
            sdrConfig.refreshPeakMs,
            sdrConfig.refreshSignalStrengthMs,
            sdrConfig.soundMode
        )
    }

    external fun initDongle(
        fd: Int,
        path: String?,
        driver: String
    ): Boolean

    external fun getDriver(): String?

    external fun applyConfig(centerFrequency: Long,
                             sampleRate: Long,
                             samplesPerReading: Int,
                             freqFocusRangeKhz: Int,
                             gain: Int,
                             refreshFFTMs: Long,
                             refreshPeakMs: Long,
                             refreshSignalStrengthMs: Long,
                             soundMode: Int
    ): Boolean

    external fun read(
        fftCallback: (FloatArray) -> Unit,
        signalStrengthCallback: (Int) -> Unit,
        peakCallback: (Float) -> Unit,
        peakNormalizedCallback: (Float) -> Unit,
        peakFrequencyCallback: (Long) -> Unit,
        pcmCallback: (ShortArray) -> Unit,
        audioPulseCallback: (Float) -> Unit
    )

    external fun stopReading()

    external fun close()

    /**
     * FREQUENCY
     */
    external fun setFrequency(frequency: Long)
    external fun setFrequencyFocusRange(frequencyFocusRange: Int)
    external fun getFrequency(): Long

    /**
     * Returns the valid frequency ranges for the device.
     * @return An array of SDRRange objects, each representing a frequency range, or null if not supported.
     */
    external fun getFrequencyRange(): Array<SDRRange>?


    /**
     * GAIN
     */
    external fun setGain(gain: Int)
    external fun getTunerGains(): IntArray?
    external fun getGain(): Int


    /**
     * SAMPLE RATE
     */
    external fun setSampleRate(sampleRate: Long)
    external fun setSamplesPerReading(samplesPerReading: Int)
    external fun getSampleRate(): Long

    /**
     * Returns a list of supported sample rates for the device.
     * @return An array of supported sample rates, or null if not supported.
     */
    external fun getSampleRatesList(): LongArray?


    /**
     * REFRESH RATES
     */
    external fun setRefreshFFTMs(refreshFFTMs: Long)
    external fun setRefreshPeakMs(refreshPeakMs: Long)
    external fun setRefreshSignalStrengthMs(refreshSignalStrengthMs: Long)

    /**
     * SOUND
     */
    /**
     * Sound mode: 0 mute, 1 normal, 2 loud
     */
    external fun setSoundMode(soundMode: Int)

    external fun getAmbientAudioEnergy(): Float

    external fun getCurrentAudioRatio(): Float
}