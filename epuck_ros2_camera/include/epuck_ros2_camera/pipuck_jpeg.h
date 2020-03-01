#ifndef MMAL_JPEG_H
#define MMAL_JPEG_H

#include "pipuck_image.h"
#include <inttypes.h>

void pipuck_jpeg_init(pipuck_image_t* input_image, pipuck_image_t* output_image);
int pipuck_jpeg_encode(pipuck_image_t *input_image, pipuck_image_t *output_image);
void pipuck_jpeg_deinit();

#endif