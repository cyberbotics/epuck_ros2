simple() {
    colcon build \
    --packages-select fastcdr \
    --executor sequential \
    --merge-install \
    --cmake-clean-cache \
    --cmake-force-configure \
    --cmake-args \
        -DCMAKE_TOOLCHAIN_FILE=/home/develop/ros2_ws/rpi_toolchain.cmake \
        -DTHIRDPARTY=ON \
        -DBUILD_TESTING:BOOL=OFF
}


build() {
    colcon build \
    --packages-ignore cyclonedds \
    --executor sequential \
    --merge-install \
    --cmake-clean-cache \
    --cmake-force-configure \
    --cmake-args \
        -DCMAKE_TOOLCHAIN_FILE=/home/develop/ros2_ws/rpi_toolchain.cmake \
        -DTHIRDPARTY=ON \
        -DBUILD_TESTING:BOOL=OFF
}