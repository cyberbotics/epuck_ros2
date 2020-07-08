## Prepare Raspberry Pi

After the Raspberry OS is ready please install ROS2 dependencies:
```bash
sudo apt install \
    liblog4cxx-dev \
    python3-numpy \
    python3-dev
```

Now, as your Raspberry Pi is equiped with ROS2 dependencies you can synchronize the sysroot:
```bash
rsync -rLR --safe-links pi@raspberrypi.local:/{lib,usr,opt/vc/lib} ./ros2-raspbian-rootfs
```
