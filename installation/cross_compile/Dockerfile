FROM ubuntu:18.04

# Build crosstool-ng (tool for building toolchain)
RUN apt-get update
RUN apt-get install -y git bison cvs flex gperf texinfo automake libtool make tar xz-utils
WORKDIR /home/develop
RUN git clone --single-branch --depth 1 https://github.com/crosstool-ng/crosstool-ng.git
WORKDIR /home/develop/crosstool-ng
RUN ./bootstrap
RUN apt-get install -y unzip help2man gawk libtool-bin libncurses-dev
RUN ./configure --prefix=/home/develop/.local
RUN make -j6
RUN make install
ENV PATH=/home/develop/.local/bin:$PATH

# Build a toolchain
WORKDIR /home/develop
RUN mkdir /home/develop/RPi3 && mkdir /home/develop/src
WORKDIR /home/develop/RPi3
COPY crosstool.config .config

# RUN ct-ng build || { cat build.log && false; } && rm -rf .build
# ENV TOOLCHAIN_PATH=/home/develop/x-tools/armv6-rpi-linux-gnueabi
# ENV PATH=${TOOLCHAIN_PATH}/bin:$PATH
# WORKDIR /home/develop