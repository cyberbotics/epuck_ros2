wcolcon() {
    export TOOLCHAIN_PREFIX=arm-linux-gnueabih
    touch src/eclipse-cyclonedds/COLCON_IGNORE

    colcon \
        build \
        --merge-install \
        --cmake-force-configure \
        --cmake-args \
        -DCMAKE_TOOLCHAIN_FILE=`pwd`/rpi_toolchain.cmake \
        -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON \
        -DTHIRDPARTY=ON \
        -DBUILD_TESTING:BOOL=ON
}
