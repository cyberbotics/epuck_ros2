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

#ifndef _PIPUCK_MMAL_H_
#define _PIPUCK_MMAL_H_

#include "epuck_ros2_camera/pipuck_image.h"

#include <interface/mmal/mmal.h>
#include <interface/vcos/vcos.h>

typedef struct pipuck_mmal_internal_t_ {
    MMAL_COMPONENT_T *encoder;
    MMAL_POOL_T *pool_in;
    MMAL_POOL_T *pool_out;
    VCOS_SEMAPHORE_T semaphore;
    MMAL_QUEUE_T *queue;
} pipuck_mmal_internal_t;

typedef struct pipuck_mmal_t_ {
    pipuck_image_t input;
    pipuck_image_t output;
    char component[25];
    pipuck_mmal_internal_t internal;
} pipuck_mmal_t;

void pipuck_mmal_init(pipuck_mmal_t* pipuck_mmal);
void pipuck_mmal_create(pipuck_mmal_t *pipuck_mmal);
void pipuck_mmal_convert(pipuck_mmal_t* pipuck_mmal);
void pipuck_mmal_deinit(pipuck_mmal_t* pipuck_mmal);

#endif
