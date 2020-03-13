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

#include "epuck_ros2_camera/pipuck_image.h"

#include "epuck_ros2_camera/pipuck_jpeg.h"

void pipuck_image_init(pipuck_image_t* image) {
    image->width = 640;
    image->height = 480;
    image->quality = 10;
    image->size = image->width * image->height * 3;
    image->encoding = PIPUCK_IMAGE_ENCODING_BGR24;
}