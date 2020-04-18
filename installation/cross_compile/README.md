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
  --packages-select fastcdr \
  --executor sequential \
  --merge-install \
  --cmake-clean-cache \
  --cmake-force-configure \
  --cmake-args \
    -DCMAKE_TOOLCHAIN_FILE=/home/develop/ros2_ws/rpi_toolchain.cmake \
    -DTHIRDPARTY=ON \
    -DBUILD_TESTING:BOOL=OFF
```

```
cmake . -DCMAKE_TOOLCHAIN_FILE=/home/develop/ros2_ws/rpi_toolchain.cmake
```


```
armv6-rpi-linux-gnueabi-gcc test.c --sysroot=/home/develop/sysroot/ \
  -I/home/develop/sysroot/usr/include/arm-linux-gnueabihf \
  -nostdlib -mcpu=arm1176jzf-s -mfpu=vfp -mfloat-abi=hard -marm \
  -Wl,-t  \
  /home/develop/sysroot/usr/lib/arm-linux-gnueabihf/crt1.o \
  /home/develop/sysroot/usr/lib/arm-linux-gnueabihf/crti.o \
  /home/develop/sysroot/usr/lib/gcc/arm-linux-gnueabihf/8/crtbegin.o \
  /home/develop/sysroot/usr/lib/gcc/arm-linux-gnueabihf/8/crtend.o \
  /home/develop/sysroot/lib/arm-linux-gnueabihf/libgcc_s.so.1 \
  /home/develop/sysroot/lib/arm-linux-gnueabihf/libc.so.6 \
  /home/develop/sysroot/usr/lib/arm-linux-gnueabihf/libc_nonshared.a \
  /home/develop/sysroot/lib/arm-linux-gnueabihf/ld-linux-armhf.so.3 \
  /home/develop/sysroot/usr/lib/arm-linux-gnueabihf/crtn.o \
  /home/develop/sysroot/lib/arm-linux-gnueabihf/libm.so.6
```