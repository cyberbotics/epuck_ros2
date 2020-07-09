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
RUN apt-get update
RUN apt-get install -y \
    python3-pip \
    git \
    cmake \
    gcc-6 \
    g++-6 \
    g++-6-arm-linux-gnueabihf \
    qemu-user-static \
    python3-numpy
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-6 60 --slave /usr/bin/g++ g++ /usr/bin/g++-6  

# Cross-compilation tweaks
RUN mkdir -p /usr/lib/arm-linux-gnueabih
RUN ln -s /home/develop/rootfs/lib/arm-linux-gnueabih/libz.so.1 /usr/lib/arm-linux-gnueabih/libz.so
RUN ln -s /home/develop/rootfs/lib/arm-linux-gnueabih/libpcre.so.3 /usr/lib/arm-linux-gnueabih/libpcre.so

# ROS2 developmnet dependencies
USER develop
RUN pip3 install \
    rosinstall_generator \
    colcon-common-extensions \
    vcstool \
    lark-parser
ENV PATH=/home/develop/.local/bin/:$PATH

USER root
RUN wget https://github.com/Pro/raspi-toolchain/releases/latest/download/raspi-toolchain.tar.gz
RUN tar xfz raspi-toolchain.tar.gz --strip-components=1 -C /opt

# Prepare workspace
USER develop
WORKDIR /home/develop
COPY rpi_toolchain.cmake rpi_toolchain.cmake
COPY bashrc.sh bashrc.sh
RUN cat bashrc.sh >> $HOME/.bashrc
WORKDIR /home/develop/ros2_ws
