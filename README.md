# SDR Bridge for Android lib

A C++ library for SDR devices with JNI bindings for Android.

So far, the library is compatible with:
* RTL-SDR
* Lime dongles (tested only on Lime Mini 2.0) - feel free to test more and report
* Airspy

The Bridge is written in Kotlin but can be adapted for Java without too much trouble.

> **⚠️ WORK IN PROGRESS**
> 
> This library is under active development. Breaking changes may occur at any time.
> If you want to use it in production, **please fork it**.
> 
> **No backward compatibility is guaranteed during the development phase.**
> 
> **Any PR is welcome to fix bugs or add new compatibility devices with SoapySDR as a base**

## Objectives

Use a SDR dongle in the context of an Android app.

This library does also basic DSP and SSB computation (FFT and sound) too before sending the results through Callback to the app.
Feel free to fork and enjoy :)
We're also happy to receive any PR or request for improvement.

## Versions

This library was tested with the following setup:
* Kotlin/Java: 17
* CMake: 3.22.1

## How to install
1. Create an Android Project
2. Add SDR-for-Android as a dependence in your Android Project

app/build.gradle.kts
```
implementation(project(":SDR-for-Android-lib"))
project(":SDR-for-Android-lib")
```
settings.gradle.kts
```
include(":app", ":SDR-for-Android-lib")
```

## How to use

SDRBridge is your main entry point from Android/Kotlin to C++.

initConf: to initialize parameters (gain, frequency, sample rate etc.)
initDongle: to initialize connection with SDR dongle. Must be already authorized from Android USB Manager (you get a File Descriptor and a Path)
read: to start reading streams asynchronously

Those are the main methods. You can also pause/resume and close at the end of the streaming session (or in case of disconnection)

Once it starts reading, it will callback the Kotlin/Java through 4 callbacks:
1. fftCallback: (FloatArray) -> Unit = {} : A Float Array of the FFT of the sample processed
2. signalStrengthCallback: (Int) -> Unit  = {} : An indication if signal was found or not (0 no signal to 3 good signal)
3. peakCallback: (Float) -> Unit = {} : The strength of the signal at its Peak
4. peakFrequencyCallback: (Long) -> Unit = {} : The frequency of the signal at its Peak

```java
    import fr.intuite.sdr.bridge.SDRBridge;

    private val sdrBridge = SDRBridge

    sdrBridge.initConf()
            
    success = sdrBridge.initDongle(deviceFileDescriptor, devicePath)
            
    if (success) {
        sdrBridge.read(fftCallback, signalStrengthCallback, peakCallback, peakFrequencyCallback)
    }
```

## Dependencies
Thanks for all the great open-source projects that were used in this project.
Most of these libraries were forked in submodules.
Please use the fork if you want to make it work straight away.

FFTW3 https://github.com/FFTW/fftw3

RTL-SDR https://github.com/osmocom/rtl-sdr.git

libusb  https://github.com/libusb/libusb.git

SoapySDR 
https://github.com/pothosware/SoapySDR
https://github.com/pothosware/SoapyAirspy
https://github.com/pothosware/SoapyRTLSDR

Lime Suite
https://github.com/myriadrf/LimeSuite

Lime Suite
https://github.com/myriadrf/LimeSuite

See our forks to make it compatible with Android
https://github.com/alexandreGellibert?tab=repositories
