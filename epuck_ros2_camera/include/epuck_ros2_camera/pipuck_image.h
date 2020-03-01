#ifndef PIPUCK_IMAGE_H
#define PIPUCK_IMAGE_H

// #include <interface/mmal/mmal.h>
// #define PIPUCK_IMAGE_ENCODING_BGR24 MMAL_ENCODING_BGR24
// #define PIPUCK_IMAGE_ENCODING_YUYV MMAL_ENCODING_YUYV

#define PIPUCK_IMAGE_ENCODING_BGR24 1
#define PIPUCK_IMAGE_ENCODING_YUYV 2

typedef struct _pipuck_image_t {
    int width;
    int height;
    int size;
    char* data;
    int encoding;
    int quality;
} pipuck_image_t;

void pipuck_image_init(pipuck_image_t* image);

#endif