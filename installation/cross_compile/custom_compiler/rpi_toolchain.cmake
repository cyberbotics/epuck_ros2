set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_LIBRARY_ARCHITECTURE arm-linux-gnueabihf)
set(CMAKE_CROSSCOMPILING 1)

set(TOOLCHAIN_PATH /home/develop/x-tools/armv6-rpi-linux-gnueabi)
set(SYSROOT_PATH /home/develop/sysroot)

set(CMAKE_C_COMPILER ${TOOLCHAIN_PATH}/bin/armv6-rpi-linux-gnueabi-gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PATH}/bin/armv6-rpi-linux-gnueabi-g++)

link_directories(
    "${SYSROOT_PATH}/lib/"
    "${SYSROOT_PATH}/usr/lib/"
    "${SYSROOT_PATH}/lib/arm-linux-gnueabihf/"
    "${SYSROOT_PATH}/usr/lib/arm-linux-gnueabihf/"
)

include_directories(
    "${SYSROOT_PATH}/usr/include/arm-linux-gnueabihf"
)

set(CMAKE_C_FLAGS "")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -nostdlib")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -nodefaultlibs")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -nostartfiles")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-use-cxa-atexit")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=arm1176jzf-s")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mfpu=vfp")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mfloat-abi=hard")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -marm")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${INCLUDE_DIRS}")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wl,-dynamic-linker ${OBJECT_LIBS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}" CACHE STRING "Flags for Raspberry Pi Zero")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS}" CACHE STRING "Flags for Raspberry Pi Zero")

set(CMAKE_SYSROOT ${SYSROOT_PATH})
set(CMAKE_FIND_ROOT_PATH ${SYSROOT_PATH}/usr/lib)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(PYTHON_SOABI cpython-37m-arm-linux-gnueabihf)
