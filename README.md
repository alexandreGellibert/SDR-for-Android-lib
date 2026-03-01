# SDR Bridge for Android

A lightweight **C++ library with JNI/Kotlin bindings** to connect SDR devices on Android using SoapySDR.

![License](https://img.shields.io/github/license/alexandreGellibert/SDR-for-Android-lib)
![Issues](https://img.shields.io/github/issues/alexandreGellibert/SDR-for-Android-lib)
![Stars](https://img.shields.io/github/stars/alexandreGellibert/SDR-for-Android-lib)

---

## ✅ Currently Supports

* **RTL-SDR** (well tested)
* **LimeSDR Mini 2.0** (tested – more devices welcome!)
* **Airspy** (basic support)

The high-level bridge is written in **Kotlin** (with a Java-compatible API), and the core logic is in **C++** using SoapySDR + basic DSP (FFT, SSB demodulation, etc.).

---

## ⚠️ WORK IN PROGRESS – BREAKING CHANGES EXPECTED

This library is under active development.

* ❌ No backward compatibility guaranteed (for now)
* ✅ If you want production stability → **please fork**
* 🤝 Pull requests are very welcome (bug fixes, new devices, improvements, etc.)

---

# 🎯 Goals

* Enable easy SDR dongle integration in Android apps (USB OTG)
* Perform lightweight DSP in native code:

  * FFT computation
  * Basic demodulation (SSB)
  * Peak detection
  * Signal strength estimation
* Deliver processed data via simple callbacks
  (no heavy processing on Java/Kotlin side)

Fork, experiment, and enjoy!

---

# 🛠 Requirements & Tested Setup

* Kotlin / Java **17**
* Android NDK + CMake **3.22.1+**
* Android API level supporting USB host mode (typically **21+**)

---

# 📦 Cloning the Repository (IMPORTANT – Submodules)

This project uses **Git submodules** for several dependencies
(SoapySDR drivers, libusb, rtl-sdr, etc.), many of which are forked for Android compatibility.

A plain `git clone` **will NOT fetch submodules** → build will fail.

---

## ✅ Recommended (Clone Everything in One Command)

```bash
git clone --recurse-submodules https://github.com/alexandreGellibert/SDR-for-Android-lib.git
```

### Faster Variant (Parallel Fetching)

```bash
git clone --recurse-submodules -j8 https://github.com/alexandreGellibert/SDR-for-Android-lib.git
```

---

## If You Already Cloned Without Submodules

```bash
cd SDR-for-Android-lib
git submodule update --init --recursive
```

This fetches all nested submodules too.

---

## ❓ Why Forks?

Several upstream libraries required Android-specific patches
(CMake adjustments, NDK compatibility, etc.).

👉 Always use the forks listed in `.gitmodules` for a working build.

---

# 🚀 Installation

## Option 1 – Recommended: Add as Git Submodule

In your Android project root:

```bash
git submodule add https://github.com/alexandreGellibert/SDR-for-Android-lib.git SDR-for-Android-lib
git submodule update --init --recursive
```

### settings.gradle(.kts)

```kotlin
include(":app", ":SDR-for-Android-lib")
```

### app/build.gradle(.kts)

```kotlin
dependencies {
    implementation(project(":SDR-for-Android-lib"))
}
```

Sync Gradle.

---

## Option 2 – Simple Copy

```bash
git clone --recurse-submodules https://github.com/alexandreGellibert/SDR-for-Android-lib.git
```

Move or symlink into your project, then include as above.

---

# 📡 Usage

Main entry point:

```
fr.intuite.sdr.bridge.SDRBridge
```

## Basic Lifecycle

1. `initDongle(fd, path, driver)`
2. `applyConfig(...)`
3. `read(...)` with callbacks
4. Control methods:

   * `setFrequency()`
   * `setGain()`
   * etc.
5. `stopReading()`
6. `close()`

---

## Kotlin Example

```kotlin
import fr.intuite.sdr.bridge.SDRBridge

if (SDRBridge.initDongle(fd, path, "rtlsdr")) {

    val config = SDRConfig(
        centerFrequency = 100_000_000L,
        sampleRate = 2_000_000L,
        samplesPerReading = 16384,
        freqFocusRangeKhz = 200,
        gain = 20,
        refreshFFTMs = 100L,
        refreshPeakMs = 500L,
        refreshSignalStrengthMs = 1000L,
        soundMode = 0
    )

    SDRBridge.applyConfig(config)
        
    SDRBridge.read(
        fftCallback = { /* FFT data */ },
        signalStrengthCallback = { /* Signal strength */ },
        peakCallback = { /* Peak raw */ },
        peakNormalizedCallback = { /* Peak normalized */ },
        peakFrequencyCallback = { /* Peak frequency */ },
        pcmCallback = { /* Audio/Sound PCM */ }
    )
}
```

See full API in `SDRBridge.kt`.

---

# 📚 Dependencies & Credits

This project stands on the shoulders of excellent open-source work.
Many are included as forked submodules for Android/NDK compatibility.

### SoapySDR & Drivers

* SoapySDR
* SoapyRTLSDR
* SoapyAirspy
* etc.

### Other Dependencies

* RTL-SDR: osmocom/rtl-sdr
* libusb: libusb/libusb
* FFTW3: FFTW/fftw3
* LimeSuite (LimeSDR): myriadrf/LimeSuite

See all submodules in `.gitmodules` and forks at:

[https://github.com/alexandreGellibert?tab=repositories](https://github.com/alexandreGellibert?tab=repositories)

Huge thanks to all maintainers ❤️

---

# 🤝 Contributing

* Found a bug? → Open an issue
* New device support? → PR adding Soapy driver + test
* Better DSP / Android integration? → Welcome!

---

# 📄 License

MIT License — see the `LICENSE` file for details.

---
