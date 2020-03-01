#include "epuck_ros2_camera/pipuck_image.h"

#include "epuck_ros2_camera/pipuck_jpeg.h"

void pipuck_image_init(pipuck_image_t* image) {
    image->width = 640;
    image->height = 480;
    image->quality = 10;
    image->size = image->width * image->height * 3;
    image->encoding = PIPUCK_IMAGE_ENCODING_BGR24;
}