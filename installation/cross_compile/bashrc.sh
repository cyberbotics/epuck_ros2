export ROS_DISTRO=foxy

cross-colcon-build() {
    export C_INCLUDE_PATH="/home/develop/rootfs/usr/include"
    export CPLUS_INCLUDE_PATH="/home/develop/rootfs/usr/include"

    colcon build \
        $@ \
        --merge-install \
        --cmake-force-configure \
        --cmake-args \
        -DCMAKE_TOOLCHAIN_FILE=/home/develop/toolchain.cmake \
        -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON \
        -DTHIRDPARTY=ON \
        -DBUILD_TESTING:BOOL=OFF
}

cross-initialize() {
    mkdir -p /home/develop/ros2_ws/src
    cd /home/develop/ros2_ws
    wget https://raw.githubusercontent.com/ros2/ros2/${ROS_DISTRO}/ros2.repos
    vcs import /home/develop/ros2_ws/src < ros2.repos
}

cross-generator() {
    export ROS_PACKAGE_PATH=/home/develop/ros2_ws/src
    rosinstall_generator $1 --deps --exclude RPP --rosdistro ${ROS_DISTRO} > /tmp/rospkgs.repos
    vcs import ${ROS_PACKAGE_PATH} < /tmp/rospkgs.repos
}
