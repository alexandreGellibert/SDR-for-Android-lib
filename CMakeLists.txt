cmake_minimum_required(VERSION 3.22.1)

project("rtl-sdr-bridge-android-lib")

set(ANDROID_JNI_LIBS_DIR
        ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}
)

set(BUILD_SHARED_LIBS ON CACHE BOOL "" FORCE)

# Force 16 KB segment alignment
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,max-page-size=0x4000")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,max-page-size=0x4000")

# =======================
# SoapySDR configuration
# =======================

set(ENABLE_APPS OFF CACHE BOOL "" FORCE)
set(ENABLE_UTILS OFF CACHE BOOL "" FORCE)
set(ENABLE_EXAMPLES OFF CACHE BOOL "" FORCE)
set(ENABLE_TESTING OFF CACHE BOOL "" FORCE)
set(ENABLE_GUI OFF CACHE BOOL "" FORCE)

# =======================
# LimeSuite configuration
# =======================
# DISABLED
set(ENABLE_GUI OFF CACHE BOOL "" FORCE)
set(ENABLE_DESKTOP OFF CACHE BOOL "" FORCE)
set(ENABLE_PYTHON OFF CACHE BOOL "" FORCE)
set(ENABLE_EXAMPLES OFF CACHE BOOL "" FORCE)

set(ENABLE_PCIE_XILLYBUS OFF CACHE BOOL "" FORCE)
set(ENABLE_LIME_UTIL OFF CACHE BOOL "" FORCE)
set(ENABLE_LIMERFE OFF CACHE BOOL "" FORCE)
set(ENABLE_STREAM OFF CACHE BOOL "" FORCE)
set(ENABLE_REMOTE OFF CACHE BOOL "" FORCE)

#ENABLED
set(ENABLE_FTDI ON CACHE BOOL "" FORCE)
#set(ENABLE_FX3 ON CACHE BOOL "" FORCE)
set(ENABLE_SOAPY_LMS7 ON CACHE BOOL "" FORCE)


# Build libusb as a shared library
add_subdirectory(libusb)

# =======================
# SoapySDR
# =======================
add_subdirectory(SoapySDR)
add_subdirectory(SoapyRTLSDR)

# LimeSuite
add_subdirectory(LimeSuite)

# fftw3f (static)
add_library(fftw3f STATIC IMPORTED)
set_target_properties(fftw3f PROPERTIES
        IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/libs/${ANDROID_ABI}/libfftw3f.a"
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/include/fftw3"
)

# rtl-sdr
add_library(rtlsdr SHARED
        rtl-sdr/src/librtlsdr.c
        rtl-sdr/src/tuner_e4k.c
        rtl-sdr/src/tuner_fc0012.c
        rtl-sdr/src/tuner_fc0013.c
        rtl-sdr/src/tuner_fc2580.c
        rtl-sdr/src/tuner_r82xx.c
        rtl-sdr/src/convenience/convenience.c
)


# App shared library
add_library(${CMAKE_PROJECT_NAME} SHARED
        # List C/C++ source files with relative paths to this CMakeLists.txt.
        src/rtl-sdr-bridge-java.cpp
        # src/ssb_demod.cpp
        # src/ssb_demod_high.cpp
        # src/ssb_demod_boost.cpp
        src/ssb_demod_opt.cpp
        #src/ssb_demod_opt+boost.cpp
)

target_include_directories(rtlsdr
        PUBLIC
        ${CMAKE_SOURCE_DIR}/rtl-sdr/include
        PRIVATE
        ${CMAKE_SOURCE_DIR}/libusb/libusb
        ${CMAKE_SOURCE_DIR}/libusb/android
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/libusb/libusb
        ${CMAKE_SOURCE_DIR}/libusb/android
        ${CMAKE_SOURCE_DIR}/SoapySDR/include
        ${CMAKE_SOURCE_DIR}
)

target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -w)

target_link_libraries(rtlsdr
        PRIVATE
        libusb
        log
        android
)

target_link_libraries(${CMAKE_PROJECT_NAME}
        PRIVATE
        android
        log
        fftw3f
        rtlsdr
        SoapySDR
)