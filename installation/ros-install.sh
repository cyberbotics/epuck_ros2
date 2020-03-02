# Set Locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add the ROS 2 apt repository
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install development tools and ROS tools
sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-pip \
  python-rosdep \
  python3-vcstool \
  wget \
  python-rosinstall-generator
# install some pip packages needed for testing
python3 -m pip install -U \
  argcomplete \
  flake8 \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  pytest-cov \
  pytest-runner \
  setuptools
# install Fast-RTPS dependencies
sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev
# install CycloneDDS dependencies
sudo apt install --no-install-recommends -y \
  libcunit1-dev

# Get ROS2 Code
rosinstall_generator --deps --rosdistro eloquent ros_base > ros2.repos
vcs import src < ros2.repos

# Install external dependencies
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro eloquent -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 rti-connext-dds-5.3.1 urdfdom_headers"

# EPuck dependencies
pip3 install smbus2
wget https://raw.githubusercontent.com/gctronic/Pi-puck/master/system/pi-puck-v4_0.dtbo
wget https://raw.githubusercontent.com/gctronic/Pi-puck/master/system/config.txt
wget https://raw.githubusercontent.com/gctronic/Pi-puck/master/system/update.sh
sh update.sh
