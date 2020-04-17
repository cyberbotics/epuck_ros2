## Prepare your system
```
scp pi@raspberrypi.local:/usr/lib/arm-linux-gnueabihf/{libz.so,libpcre.so} /usr/lib/arm-linux-gnueabihf
```

```
rsync -v --copy-unsafe-links --progress -r pi@raspberrypi.local:/{lib,usr,opt/vc/lib} $HOME/rpi
```
or
```
sudo sshfs -o follow_symlinks,allow_other pi@raspberrypi.local:/ $HOME/rpi
```
you may need:
```
sudo echo 'user_allow_other' >> /etc/fuse.conf
```

## Docker
You can build docker as:
```
docker build . -t armv6-ros2-toolchain
```
and run tty as:
```
docker run -it -v $HOME/rpi:/home/develop/sysroot armv6-ros2-toolchain /bin/bash
```

## Commands
```
colcon build \
  --executor sequential \
  --merge-install \
  --cmake-clean-cache \
  --cmake-force-configure \
  --cmake-args \
    -DCMAKE_TOOLCHAIN_FILE=/home/develop/ros2_ws/rpi_toolchain.cmake \
    -DTHIRDPARTY=ON \
    -DBUILD_TESTING:BOOL=OFF
```