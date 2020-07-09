# Cross-Compilation
Since processing power of Raspberry Pi Zero module is insufficient for "quick" compilation we made tools to help you compile ROS2 on your PC for the Raspberry Pi Zero.
Therefore, this tutorial will give you instructions on how to use the tools to cross-compile ROS2 for Raspberry Pi Zero (could be also extended to other Raspberry Pi versions).

## Prerequisites
Please make sure everything is ready for you proceed:
- Raspberry Pi OS ready on your Raspberry Pi ([tutorial](https://www.raspberrypi.org/documentation/installation/installing-images/)).
- SSH access to Raspberry Pi ([tutorial](../README.md#wifi-and-ssh)).
- It is preferable to have Ubuntu installed on your PC.
- Docker on your PC ([tutorial](https://docs.docker.com/get-docker/)).
- `sshfs` package on your PC (`sudo apt install sshfs`).

## Raspberry Pi Preparation

ROS2 requires the following packages to be installed on your Raspberry Pi:
```bash
sudo apt update
sudo apt install \
    liblog4cxx-dev \
    python3-numpy \
    python3-dev
```

## Your PC Preparation

Now, as your Raspberry Pi is equiped with ROS2 dependencies you can synchronize the [sysroot](https://wiki.dlang.org/GDC/Cross_Compiler/Existing_Sysroot#:~:text=A%20sysroot%20is%20a%20folder,sysroot%2Fusr%2Finclude'.).
This will allow the cross-compiler to use header files and libraries from your Raspberry Pi.
There are two ways to get the sysroot on your PC.

The first way, you can synchronize the content of the sysroot:
```bash
rsync -rLR --safe-links pi@raspberrypi.local:/{lib,usr,opt/vc/lib} ./ros2-raspbian-rootfs
```

> Initially, `rsync` will take more time to perform synchronization, but cross-compilation process will be faster after (as the cross-compiler doesn't have to ask for a file every time).


The second way, you can use `sshfs` tool to mount the sysroot:
```bash
sshfs -o follow_symlinks,allow_other -o cache_timeout=115200 pi@raspberrypi.local:/ ./ros2-raspbian-rootfs
```
you may need:
```
sudo echo 'user_allow_other' >> /etc/fuse.conf
```

## ROS2 Base Cross-Compilation on Your PC

When the sysroot is ready you can start the cross-compilation.
Run:
```bash
./start_docker.sh
```
to start the Docker with the cross-compilation support.

In the Docker, we prepare a few commands with prefix `cross-*` to bootstrap your development.
For example, you can use `cross-initialize` to download ROS2 source code or `cross-colcon-build` to build it.
These commands are simple bash functions located in `.bashrc`.
You can see how the commands are implemented by typing e.g. `type cross-initialize` and change them according to your needs.
Therefore, in the Docker container type:
```
cross-initialize
```
to download ROS2 source code and:
```
cross-colcon-build
```
to compile it.

## Using the Cross-Compiled ROS2 on your Raspberry Pi

To use the cross-compiled ROS2 on your Raspberry Pi you have to copy `./ros2_ws/install` to the Raspberry Pi or to mount it, e.g.:
```
mkdir ros2
sshfs [pc_username]@[pc_address]:[path_to_this_folder]/ros2_ws/install ros2
source ros2/local_setup.bash
```

## Cross-Compiling Custom ROS2 Packages

With these tools, you can compile custom ROS2 packages as well.
It is enough to put a source of the package to `./ros2_ws/src` and inside of the Docker run `cross-colcon-build`.
