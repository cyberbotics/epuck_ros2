# E-Puck Camera

This ROS2 driver takes images from camera device located on e-puck2 robot. It uses V4L2 driver for extracting the images from the camera and Multimedia Abstraction Layer to perform fast JPEG conversion by utilizing onboard GPU.

## Parameters
- `quality` Quality of JPEG conversion (default 8)
- `interval` Interval of image acquisition in milliseconds (default 80) 
- `width` Width of the output RGB image, height is automatically calculated (default 640, valid values [640, 320, 256, 128, 64])

## Topics
- `/image_raw [sensor_msgs/msg/Image]` Raw RGB images
- `/image_raw/compressed [sensor_msgs/msg/CompressedImage]` JPEG compressed images

There a few things to note here, mainly related to Raspberry Pi Zero computational power and WiFi bandwidth.
- Images will be published only if there is at least one subscriber to avoid unnecessary computation.
- Topic `/image_raw/compressed` is suitable for offboard processing since the data transmission rate is significantly lower.
- Topic `/image_raw` is suitable for onboard processing since no conversion is needed (much lower CPU utilization).
- If you have multiple nodes subscribed to `/image_raw/compressed` it would be handy to have an additional node republishing images to reduce network utilisation, e.g. `ros2 run image_transport republish compressed in/compressed:=image_raw/compressed raw out:=image_raw/uncompressed`.
