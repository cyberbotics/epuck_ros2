set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_LIBRARY_ARCHITECTURE arm-linux-gnueabihf)

set(CMAKE_C_COMPILER /home/develop/x-tools/armv6-rpi-linux-gnueabi/bin/armv6-rpi-linux-gnueabi-gcc)
set(CMAKE_CXX_COMPILER /home/develop/x-tools/armv6-rpi-linux-gnueabi/bin/armv6-rpi-linux-gnueabi-g++)

set(CMAKE_SYSROOT /home/develop/sysroot)
set(CMAKE_FIND_ROOT_PATH /home/develop/sysroot)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

set(PYTHON_SOABI cpython-37m-arm-linux-gnueabihf)