FROM ubuntu:18.04

# Install dependencies
RUN apt-get update && apt-get install -y \
    wget \
    tar \
    python3-pip \
    git \
    cmake \
    qemu-user-static \
    python3-numpy \
    sshfs \
    rsync \
    && rm -rf /var/lib/apt/lists/*
RUN echo user_allow_other >> /etc/fuse.conf

# Add user
RUN useradd -m develop
RUN echo "develop:develop" | chpasswd
RUN usermod -aG sudo develop

# ROS2 developmnet dependencies
USER develop
RUN pip3 install --no-cache-dir \
    rosinstall_generator \
    colcon-common-extensions \
    vcstool \
    lark-parser
ENV PATH=/home/develop/.local/bin/:$PATH

# Install compiler
USER root
WORKDIR /tmp
RUN wget https://github.com/Pro/raspi-toolchain/releases/latest/download/raspi-toolchain.tar.gz \
    && tar xfz raspi-toolchain.tar.gz --strip-components=1 -C /opt \
    && rm raspi-toolchain.tar.gz

# Prepare workspace
USER develop
WORKDIR /home/develop
COPY toolchain.cmake toolchain.cmake
COPY bashrc.sh bashrc.sh
RUN cat bashrc.sh >> $HOME/.bashrc
WORKDIR /home/develop/ros2_ws
