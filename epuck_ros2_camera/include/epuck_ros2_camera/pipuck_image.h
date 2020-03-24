// Copyright 1996-2020 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PIPUCK_IMAGE_H
#define PIPUCK_IMAGE_H

#include <interface/mmal/mmal.h>

#define PIPUCK_IMAGE_ENCODING_BGR24 MMAL_ENCODING_BGR24
#define PIPUCK_IMAGE_ENCODING_YUYV MMAL_ENCODING_YUYV
#define PIPUCK_IMAGE_ENCODING_JPEG MMAL_ENCODING_JPEG
#define PIPUCK_IMAGE_ENCODING_RGB24 MMAL_ENCODING_RGB24

typedef struct _pipuck_image_t {
  int width;
  int height;
  int size;
  char *data;
  int encoding;
  int quality;
} pipuck_image_t;

void pipuck_image_init(pipuck_image_t *image);

#endif
