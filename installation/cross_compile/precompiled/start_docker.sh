#!/bin/bash

if [ ! -d "./ros2_ws" ]; then
    mkdir -p ./ros2_ws
fi

echo "Building docker image..."
docker build -t rpi_cross_compile -f Dockerfile .
docker run -it \
  -v $PWD/ros2-raspbian-rootfs:/home/develop/rootfs \
  -v $PWD/ros2_ws:/home/develop/ros2_ws \
  rpi_cross_compile \
  /bin/bash
