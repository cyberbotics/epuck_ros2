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

# Prepare workspace
WORKDIR /home/develop
COPY rpi_toolchain.cmake rpi_toolchain.cmake
COPY bashrc.sh bashrc.sh
RUN cat bashrc.sh >> $HOME/.bashrc
WORKDIR /home/develop/ros2_ws
