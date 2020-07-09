cross-colcon-build() {
    export C_INCLUDE_PATH=/home/develop/rootfs/usr/include
    export CPLUS_INCLUDE_PATH=/home/develop/rootfs/usr/include
    colcon build \
        --merge-install \
        --packages-up-to ros2topic \
        --cmake-force-configure \
        --cmake-args \
        -DCMAKE_TOOLCHAIN_FILE=/home/develop/rpi_toolchain.cmake \
        -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON \
        -DTHIRDPARTY=ON \
        -DBUILD_TESTING:BOOL=OFF
}

cross-initialize() {
    mkdir -p /home/develop/ros2_ws/src
    cd /home/develop/ros2_ws
    wget https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos
    vcs import /home/develop/ros2_ws/src < ros2.repos
}
