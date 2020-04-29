set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_LIBRARY_ARCHITECTURE arm-linux-gnueabihf)
set(CMAKE_CROSSCOMPILING 1)

set(TOOLCHAIN_PATH /home/develop/cross-pi-gcc)
set(SYSROOT_PATH /home/develop/sysroot)

set(CMAKE_C_COMPILER ${TOOLCHAIN_PATH}/bin/arm-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PATH}/bin/arm-linux-gnueabihf-g++)

set(LIB_DIRS 
    "${LIB_DIRS} ${SYSROOT_PATH}/lib/"
    "${LIB_DIRS} ${SYSROOT_PATH}/usr/lib/"
    "${LIB_DIRS} ${SYSROOT_PATH}/lib/arm-linux-gnueabihf/"
    "${LIB_DIRS} ${SYSROOT_PATH}/usr/lib/arm-linux-gnueabihf/"
)

set(CMAKE_C_FLAGS "")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -I${SYSROOT_PATH}/usr/include/arm-linux-gnueabihf/")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -I${SYSROOT_PATH}/usr/include/")
foreach(LIB ${LIB_DIRS})
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -L${LIB} -Wl,-rpath-link,${LIB}")
endforeach()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}" CACHE STRING "Flags for Raspberry Pi Zero")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS}" CACHE STRING "Flags for Raspberry Pi Zero")

set(CMAKE_SYSROOT ${SYSROOT_PATH})
set(CMAKE_FIND_ROOT_PATH ${SYSROOT_PATH}/usr/lib)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(PYTHON_SOABI cpython-37m-arm-linux-gnueabihf)
