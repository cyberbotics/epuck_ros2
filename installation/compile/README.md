## Compile ROS2 on Raspberry Pi
In certain use cases, you may prefer to install ROS2, `epuck_ros2`, configure the device tree and install dependencies manually.
This is way more complicated and time consuming than using the provided image, but it is the only way to install a ROS2 version other than the ones provided by us.
Please note that neither ARMv6 architecture, neither Debian (Raspberry Pi OS is based on Debian) are supported by ROS2 and therefore, you may need to manually solve the compilation issues.

After the Raspberry Pi OS is booted and you have an SSH connection please execute the following command:
```
curl https://raw.githubusercontent.com/cyberbotics/epuck_ros2/master/installation/compile/compile.sh | sh
```
> Note that this script is in very early stage and it may fail.
In case of failure try to understand which package has caused the error and check if the community on GitHub has the solution.
Also, there are around 200 ROS2 packages and it will take a long time to compile.

The script will install dependencies, download ROS2 locally, compile and configure the device tree necessary for Raspberry Pi to communicate with the e-puck2 base.