#!/bin/bash
#
# This script installs ROS2 dependencies, and downloads and compiles ROS2 base for Raspberry Pi Zero.
# Warning: The compilation process is very long and it is prone to erros as Raspberry Pi Zero is not a part of tier 1/2 support.
#

if [ ! $ROS_DISTRO ]; then
  export ROS_DISTRO=foxy
fi

# Set Locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add the ROS2 apt repository
sudo apt update
sudo apt install -y curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install development tools and ROS tools
sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-pip \
  wget

# Install some pip packages
pip3 install -U \
  argcomplete \
  setuptools \
  vcstool \
  colcon-common-extensions \
  rosinstall_generator
sudo -H pip3 install rosdep
export PATH=$HOME/.local/bin/:$PATH

# Install Fast-RTPS dependencies
sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev

# Install CycloneDDS dependencies
sudo apt install --no-install-recommends -y \
  libcunit1-dev

# Get ROS2 Code
mkdir -p $HOME/ros2/src
cd $HOME/ros2
rosinstall_generator --deps --rosdistro ${ROS_DISTRO} ros_base >ros2.repos
vcs import $HOME/ros2/src <ros2.repos

# Install external dependencies
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 rti-connext-dds-5.3.1 urdfdom_headers"

# Build
export CFLAGS='-latomic'
colcon build \
  --cmake-force-configure \
  --cmake-args \
    -DBUILD_TESTING:BOOL=OFF

# E-Puck configuration
cd /tmp
wget https://raw.githubusercontent.com/cyberbotics/epuck_ros2/master/installation/compile/pi-puck-v4_0.dtbo
wget https://raw.githubusercontent.com/cyberbotics/epuck_ros2/master/installation/compile/config.txt
sudo cp config.txt /boot/config.txt
sudo rm /boot/overlays/pi-puck-*
sudo cp pi-puck-*.dtbo /boot/overlays/
sudo sh -c 'echo "i2c-dev" >> /etc/modules'
sync
sudo reboot
