#ifndef PIPUCK_V4L2_H
#define PIPUCK_V4L2_H

#include "pipuck_image.h"

void pipuck_v4l2_init();
void pipuck_v4l2_deinit();
void pipuck_v4l2_capture(pipuck_image_t* image);

#endif