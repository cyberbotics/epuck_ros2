FROM ubuntu:18.04

# Install dependencies
RUN apt-get update
RUN apt-get install -y \
    wget \
    tar

# User
RUN useradd -m develop
RUN echo "develop:develop" | chpasswd
RUN usermod -aG sudo develop
USER develop
WORKDIR /home/develop

# ROS2 tools
USER root
RUN apt-get install -y \
    python3-pip \
    git \
    cmake
USER develop
RUN pip3 install \
    rosinstall_generator \
    colcon-common-extensions \
    vcstool
ENV PATH=/home/develop/.local/bin/:$PATH

# ROS2 workspace
RUN mkdir -p /home/develop/ros2_ws/src
WORKDIR /home/develop/ros2_ws
ENV ROS_PACKAGE_PATH=/home/develop/ros2_ws/src
RUN rosinstall_generator ros_core --deps --exclude RPP --rosdistro eloquent > /tmp/rospkgs.repos
RUN vcs import ${ROS_PACKAGE_PATH} < /tmp/rospkgs.repos

# Toolchain
WORKDIR /home/develop
RUN wget -O raspi-toolchain.tar.gz https://www.dropbox.com/s/j3xg8sjn4qohwkj/raspi-toolchain.tar.gz?dl=0
RUN tar -xvf raspi-toolchain.tar.gz && rm raspi-toolchain.tar.gz

# Compile
WORKDIR /home/develop/ros2_ws
COPY rpi_toolchain.cmake rpi_toolchain.cmake
RUN mkdir /home/develop/sysroot
COPY test.c test.c