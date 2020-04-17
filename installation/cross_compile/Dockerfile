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

# Add user
RUN useradd -m develop
RUN echo "develop:develop" | chpasswd
RUN usermod -aG sudo develop
USER develop
WORKDIR /home/develop

# Build crosstool-ng (tool for building toolchain)
RUN git clone --single-branch --depth 1 https://github.com/crosstool-ng/crosstool-ng.git
WORKDIR /home/develop/crosstool-ng
RUN ./bootstrap
RUN ./configure --enable-local
RUN make -j8
ENV PATH=$PATH:/home/develop/crosstool-ng

# Build a toolchain
WORKDIR /home/develop
RUN mkdir /home/develop/toolchain && mkdir /home/develop/src
WORKDIR /home/develop/toolchain
COPY crosstool.config crosstool.config
ENV DEFCONFIG=/home/develop/toolchain/crosstool.config
RUN ct-ng defconfig
RUN ct-ng build || { cat build.log && false; } && rm -rf .build
ENV TOOLCHAIN_PATH=/home/develop/x-tools/armv6-rpi-linux-gnueabi
ENV PATH=${TOOLCHAIN_PATH}/bin:$PATH

# TODO