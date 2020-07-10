# Cross-Compilation
Since processing power of the Raspberry Pi Zero module is insufficient for "quick" compilation we made tools to help you compile ROS2 on your PC for the Raspberry Pi Zero.
Therefore, this tutorial will give you instructions on how to use the tools to cross-compile ROS2 for Raspberry Pi Zero (could be also extended to other Raspberry Pi versions).

## Prerequisites
Please make sure everything is ready for you to proceed:
- Raspberry Pi OS ready on your Raspberry Pi ([tutorial](https://www.raspberrypi.org/documentation/installation/installing-images/)).
- SSH access to Raspberry Pi ([tutorial](../README.md#wifi-and-ssh)).
- Docker on your PC ([tutorial](https://docs.docker.com/get-docker/)).
- Optional SSH server on your PC (`sudo apt install openssh-server`).

## Raspberry Pi Preparation

ROS2 requires the following packages to be installed on your Raspberry Pi:
```bash
sudo apt update

# Compilation dependencies
sudo apt install \
    liblog4cxx-dev

# Runtime dependencies
sudo apt install \
    python3-numpy \
    python3-dev \
    python3-netifaces \
    python3-yaml
```

## ROS2 Cross-Compilation on Your PC

Now we can start a process of ROS2 cross-compilation.
The supplied Docker image is equipped with most of the tools you need, so start it:
```bash
./start_docker.sh
```
This command will build and run the Docker container, and allocate a pseudo-TTY.
It means that you can use it as an another operating system.
From now on, most of the tasks you will be able to do from the Docker container.


### Rootfs Preparation

As your Raspberry Pi is equipped with ROS2 dependencies you have to synchronize the [rootfs](https://wiki.dlang.org/GDC/Cross_Compiler/Existing_Sysroot#:~:text=A%20sysroot%20is%20a%20folder,sysroot%2Fusr%2Finclude'.).
This will allow the cross-compiler to use header files and libraries from your Raspberry Pi.
There are two ways to get the rootfs inside your Docker.

The first way, you can synchronize the content of the rootfs:
```bash
rsync -rLR --safe-links pi@[raspberry_pi_ip]:/{lib,usr,opt/vc/lib} /home/develop/rootfs
```

> Initially, `rsync` will take more time to perform synchronization, but the cross-compilation process will be faster after (as the cross-compiler doesn't have to transfer a file from Raspberry Pi every time).


The second way, you can use `sshfs` tool to mount the rootfs:
```bash
sshfs -o follow_symlinks,allow_other -o cache_timeout=115200 pi@[raspberry_pi_ip]:/ /home/develop/rootfs
```

### Compilation Commands 

In the Docker, we prepare a few commands with prefix `cross-*` to bootstrap your development.
For example, you can use `cross-initialize` to download ROS2 source code or `cross-colcon-build` to build it.
These commands are simple bash functions located in `.bashrc`.
You can see how the commands are implemented by typing e.g. `type cross-initialize` and change them according to your needs.
Therefore, in the Docker container type:
```bash
cross-initialize
```
to download ROS2 source code and:
```bash
cross-colcon-build --packages-up-to ros2topic
```
to compile it.

> Flag `--packages-up-to ros2topic` will compile `ros2topic` and all it's recursive dependencies.
Also, note that it can happen you need to run the command twice to compile `fastrtps` package.

## Using the Cross-Compiled ROS2 on your Raspberry Pi

To use the cross-compiled ROS2 on your Raspberry Pi you have to copy `./ros2_ws/install` to the Raspberry Pi.
Alternatevelly, you can mount it by running the following commands on Raspberry Pi:
```bash
mkdir ros2
sshfs [pc_username]@[pc_address]:[path_to_this_folder]/ros2_ws/install ros2
source ros2/local_setup.bash
```
But in that case, make sure your PC has SSH server installed and configured:
```bash
sudo apt install openssh-server
sudo systemctl start sshd
```

## Cross-Compiling Custom ROS2 Packages

With these tools, you can compile custom ROS2 packages as well.
It is enough to put a source code of the package to `./ros2_ws/src` and inside of the Docker run `cross-colcon-build`.
For example, to compile `epuck_ros2` package execute the following:

```bash
git clone --recurse-submodules https://github.com/cyberbotics/epuck_ros2.git src/epuck_ros2
cross-colcon-build --packages-up-to epuck_ros2_driver
```

> You can use `--packages-select epuck_ros2_driver` flag to compile `epuck_ros2_driver` package only.

### Missing Dependencies

Sometimes, your package will depend on ROS2 packages that are not a part of ROS2 base.
In that case, you can use `cross-generator` to download it.
For example, in case of `epuck_ros2` package, dependency `camera_info_manager` is missing, so you can download it as:
```bash
cross-generator camera_info_manager
```
