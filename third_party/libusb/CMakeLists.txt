cmake_minimum_required(VERSION 3.10)

project(libusb C)

# Force 16 KB segment alignment (Android 16KB page requirement)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,max-page-size=0x4000")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,max-page-size=0x4000")

# Core sources
set(LIBUSB_CORE
        libusb/core.c
        libusb/descriptor.c
        libusb/hotplug.c
        libusb/io.c
        libusb/strerror.c
        libusb/sync.c
)

# Linux backend (used for Android)
set(LIBUSB_LINUX
        libusb/os/linux_usbfs.c
        libusb/os/linux_netlink.c
        libusb/os/threads_posix.c
        libusb/os/events_posix.c
)

add_library(libusb SHARED
        ${LIBUSB_CORE}
        ${LIBUSB_LINUX}
)

target_include_directories(libusb PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/android
)
target_include_directories(libusb
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/libusb
)

target_link_libraries(libusb
        log
        android
)

target_compile_definitions(libusb PRIVATE
        ANDROID
        HAVE_POLL_H
        HAVE_SYS_TIME_H
)
