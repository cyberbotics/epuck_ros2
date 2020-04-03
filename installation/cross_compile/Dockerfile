FROM ubuntu:18.04

# Install dependencies
RUN apt-get update
RUN apt-get install -y \
    git \
    bison \
    cvs \
    flex \
    gperf \
    texinfo \
    automake \
    libtool \
    make \
    tar \
    xz-utils \
    unzip \
    help2man \
    gawk \
    libtool-bin \
    libncurses-dev

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
RUN ./configure --prefix=/home/develop/.local
RUN make -j8
RUN make install
ENV PATH=/home/develop/.local/bin:$PATH

# Build a toolchain
WORKDIR /home/develop
RUN mkdir /home/develop/toolchain && mkdir /home/develop/src
WORKDIR /home/develop/toolchain
COPY crosstool.config .config

RUN ct-ng build --help
RUN ct-ng build || { cat build.log && false; } && rm -rf .build
# ENV TOOLCHAIN_PATH=/home/develop/x-tools/armv6-rpi-linux-gnueabi
# ENV PATH=${TOOLCHAIN_PATH}/bin:$PATH
# WORKDIR /home/develop