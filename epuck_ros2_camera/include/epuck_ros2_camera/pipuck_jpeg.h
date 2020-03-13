// Copyright 2020 Cyberbotics
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

#ifndef MMAL_JPEG_H
#define MMAL_JPEG_H

#include "epuck_ros2_camera/pipuck_image.h"
#include <inttypes.h>

void pipuck_jpeg_init(pipuck_image_t* input_image, pipuck_image_t* output_image);
int pipuck_jpeg_encode(pipuck_image_t *input_image, pipuck_image_t *output_image);
void pipuck_jpeg_deinit();

#endif