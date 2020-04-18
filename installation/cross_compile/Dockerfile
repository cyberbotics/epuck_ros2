FROM ubuntu:18.04

# Install dependencies
RUN apt-get update
RUN apt-get install -y \
    wget \
    build-essential \
    autoconf \
    git \
    bison \
    cvs \
    flex \
    gperf \
    texinfo \
    automake \
    libtool \
    libtool-bin \
    make \
    tar \
    xz-utils \
    unzip \
    help2man \
    gawk \
    libncurses5-dev \
    python3-dev \
    python3-distutils

# Add non-root user
RUN useradd -m develop
RUN echo "develop:develop" | chpasswd
RUN usermod -aG sudo develop
USER develop
WORKDIR /home/develop

# Build crosstool-ng (a tool for building toolchain)
RUN git clone --single-branch --depth 1 https://github.com/crosstool-ng/crosstool-ng.git
WORKDIR /home/develop/crosstool-ng
RUN ./bootstrap
RUN ./configure --enable-local
RUN make -j8
ENV PATH=$PATH:/home/develop/crosstool-ng

# Build the toolchain
WORKDIR /home/develop
RUN mkdir /home/develop/toolchain
WORKDIR /home/develop/toolchain
COPY crosstool.config crosstool.config
ENV DEFCONFIG=/home/develop/toolchain/crosstool.config
RUN ct-ng defconfig
RUN ct-ng build || { cat build.log && false; } && rm -rf .build
ENV TOOLCHAIN_PATH=/home/develop/x-tools/armv6-rpi-linux-gnueabi
ENV PATH=${TOOLCHAIN_PATH}/bin:$PATH

# ROS2 tools
USER root
RUN apt-get install -y \
    python3-pip \
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

# ROS2 compilation
COPY rpi_toolchain.cmake rpi_toolchain.cmake
RUN mkdir /home/develop/sysroot
COPY test.c test.c

