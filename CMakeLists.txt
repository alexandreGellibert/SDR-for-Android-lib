cmake_minimum_required(VERSION 3.22.1)

project("sdr-bridge-java-soapy-lib")

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
#FTDI to be used for Lime Mini 1.0, 2.0.
set(ENABLE_FTDI ON CACHE BOOL "" FORCE)
#FX3 to be used for LimeSDR USB (not tested)
set(ENABLE_FX3 OFF CACHE BOOL "" FORCE)
set(ENABLE_SOAPY_LMS7 ON CACHE BOOL "" FORCE)


add_subdirectory(third_party/libusb)
# RTL-SDR
add_subdirectory(third_party/rtl-sdr)
#
## Airspy
add_subdirectory(third_party/libairspy)

# =======================
# SoapySDR
# =======================
add_subdirectory(third_party/SoapySDR)
add_subdirectory(third_party/SoapyRTLSDR)
add_subdirectory(third_party/SoapyAirspy)
# LimeSuite
add_subdirectory(third_party/LimeSuite)


# fftw3f (static)
add_library(fftw3f STATIC IMPORTED)
set_target_properties(fftw3f PROPERTIES
        IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/third_party/fftw3/static_lib/${ANDROID_ABI}/libfftw3f.a"
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/third_party/fftw3/include"
)

# App shared library
add_library(${CMAKE_PROJECT_NAME} SHARED
        # List C/C++ source files with relative paths to this CMakeLists.txt.
        src/sdr-bridge-java-soapy.cpp
        src/sdr-logger.cpp
        src/dsp/fft_process.cpp
        src/ssb/ssb_demod_opt.cpp
        src/ssb/ssb_processor.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/third_party/SoapySDR/include
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/src/dsp
        ${CMAKE_SOURCE_DIR}/src/bridge
)

target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -w)

target_link_libraries(${CMAKE_PROJECT_NAME}
        PRIVATE
        android
        log
        fftw3f
        SoapySDR
)