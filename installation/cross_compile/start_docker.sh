#!/bin/bash

set -e

if [ ! -d "./ros2_ws" ]; then
  mkdir -p ./ros2_ws
  chmod 777 ./ros2_ws
fi
touch ./ros2_ws/.dockerignore

if [ ! -d "./rootfs" ]; then
  mkdir -p ./rootfs
  chmod 777 ./rootfs
fi

echo "Building docker image..."
docker build -t rpi_cross_compile -f Dockerfile .
docker run -it \
  --device /dev/fuse \
  --cap-add SYS_ADMIN \
  --security-opt apparmor:unconfined \
  -v $PWD/rootfs:/home/develop/rootfs \
  -v $PWD/ros2_ws:/home/develop/ros2_ws \
  rpi_cross_compile \
  /bin/bash
