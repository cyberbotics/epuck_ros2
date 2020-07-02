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
    cmake \
    gcc-6 \
    g++-6 \
    g++-6-arm-linux-gnueabihf
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-6 60 --slave /usr/bin/g++ g++ /usr/bin/g++-6  

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
RUN wget https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos
RUN vcs import ${ROS_PACKAGE_PATH} < ros2.repos

# Compile
WORKDIR /home/develop/ros2_ws
COPY rpi_toolchain.cmake rpi_toolchain.cmake
COPY bashrc.sh bashrc.sh
RUN cat bashrc.sh >> $HOME/.bashrc