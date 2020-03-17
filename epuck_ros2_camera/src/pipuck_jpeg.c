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

#include "epuck_ros2_camera/pipuck_jpeg.h"

#include <assert.h>
#include <bcm_host.h>
#include <interface/mmal/mmal.h>
#include <interface/mmal/mmal_buffer.h>
#include <interface/mmal/mmal_logging.h>
#include <interface/mmal/util/mmal_connection.h>
#include <interface/mmal/util/mmal_default_components.h>
#include <interface/mmal/util/mmal_util.h>
#include <interface/mmal/util/mmal_util_params.h>
#include <interface/vcos/vcos.h>
#include <stdio.h>

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))

static MMAL_COMPONENT_T *encoder;
static MMAL_STATUS_T status;
static MMAL_POOL_T *pool_in;
static MMAL_POOL_T *pool_out;
static VCOS_SEMAPHORE_T semaphore;
static VCOS_STATUS_T vcos_status;
static MMAL_BUFFER_HEADER_T *buffer;
static MMAL_QUEUE_T *queue;
static MMAL_BOOL_T eos;
static int sent_bytes;

static void release_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);
static void put_to_queue_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);

void put_to_queue_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
  mmal_queue_put(queue, buffer);
  vcos_semaphore_post(&semaphore);
}

void release_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
  mmal_buffer_header_release(buffer);
  vcos_semaphore_post(&semaphore);
}

void pipuck_jpeg_init(pipuck_image_t *input_image, pipuck_image_t *output_image) {
  bcm_host_init();
  vcos_init();

  vcos_semaphore_create(&semaphore, "sync", 1);

  status = mmal_component_create(MMAL_COMPONENT_DEFAULT_IMAGE_ENCODER, &encoder);
  assert(status == MMAL_SUCCESS);
  status = mmal_port_enable(encoder->control, release_buffer_callback);
  assert(status == MMAL_SUCCESS);

  // Configure encoder input
  encoder->input[0]->format->encoding = input_image->encoding;
  encoder->input[0]->format->es->video.crop.width = input_image->width;
  encoder->input[0]->format->es->video.crop.height = input_image->height;
  encoder->input[0]->format->es->video.width = VCOS_ALIGN_UP(input_image->width, 32);
  encoder->input[0]->format->es->video.height = VCOS_ALIGN_UP(input_image->height, 16);
  encoder->input[0]->format->es->video.frame_rate.num = 0;
  encoder->input[0]->format->es->video.frame_rate.den = 1;
  encoder->input[0]->format->es->video.par.num = 1;
  encoder->input[0]->format->es->video.par.den = 1;
  status = mmal_port_format_commit(encoder->input[0]);
  assert(status == MMAL_SUCCESS);

  // Configure encoder output
  mmal_format_copy(encoder->output[0]->format, encoder->input[0]->format);
  encoder->output[0]->format->encoding = MMAL_ENCODING_JPEG;
  status = mmal_port_format_commit(encoder->output[0]);
  assert(status == MMAL_SUCCESS);
  status = mmal_port_parameter_set_uint32(encoder->output[0], MMAL_PARAMETER_JPEG_Q_FACTOR, output_image->quality);
  assert(status == MMAL_SUCCESS);

  // Configure buffers
  encoder->input[0]->buffer_num = encoder->input[0]->buffer_num_recommended;
  encoder->input[0]->buffer_size = encoder->input[0]->buffer_size_recommended;
  encoder->output[0]->buffer_num = encoder->output[0]->buffer_num_recommended;
  encoder->output[0]->buffer_size = encoder->output[0]->buffer_size_recommended;
  printf("There are %d input buffers with size %d and %d output buffers with size %d\n", encoder->input[0]->buffer_num,
         encoder->input[0]->buffer_size, encoder->output[0]->buffer_num, encoder->output[0]->buffer_size);

  // Enable ports
  status = mmal_port_enable(encoder->input[0], release_buffer_callback);
  assert(status == MMAL_SUCCESS);
  status = mmal_port_enable(encoder->output[0], put_to_queue_callback);
  assert(status == MMAL_SUCCESS);

  // Create a buffer pool
  queue = mmal_queue_create();
  pool_in = mmal_port_pool_create(encoder->input[0], encoder->input[0]->buffer_num, encoder->input[0]->buffer_size);
  assert(pool_in != 0);
  pool_out = mmal_port_pool_create(encoder->output[0], encoder->output[0]->buffer_num, encoder->output[0]->buffer_size);
  assert(pool_out != 0);

  // Start component
  status = mmal_component_enable(encoder);
  assert(status == MMAL_SUCCESS);
}

void pipuck_jpeg_deinit() {
  mmal_port_disable(encoder->input[0]);
  mmal_port_disable(encoder->output[0]);
  mmal_component_disable(encoder);
  mmal_queue_destroy(queue);
  mmal_port_pool_destroy(encoder->input[0], pool_in);
  mmal_port_pool_destroy(encoder->output[0], pool_out);
  vcos_semaphore_delete(&semaphore);
}

int pipuck_jpeg_encode(pipuck_image_t *input_image, pipuck_image_t *output_image) {
  eos = MMAL_FALSE;
  sent_bytes = 0;
  output_image->size = 0;

  while (eos == MMAL_FALSE) {
    vcos_status = vcos_semaphore_wait_timeout(&semaphore, 2000);
    assert(vcos_status == VCOS_SUCCESS);

    // Input frames
    if ((buffer = mmal_queue_get(pool_in->queue)) != NULL) {
      buffer->length = MAX(0, MIN(buffer->alloc_size, input_image->size - sent_bytes));
      buffer->offset = 0;
      buffer->pts = MMAL_TIME_UNKNOWN;
      buffer->dts = MMAL_TIME_UNKNOWN;
      // printf("Flags %d; EOS %d; Length: %d\n", buffer->flags, eos, buffer->length);
      buffer->data = input_image->data + sent_bytes;
      sent_bytes += buffer->length;
      status = mmal_port_send_buffer(encoder->input[0], buffer);
      assert(status == MMAL_SUCCESS);
    }

    // Output frames
    while ((buffer = mmal_queue_get(queue)) != NULL) {
      eos = buffer->flags & MMAL_BUFFER_HEADER_FLAG_FRAME_END;
      memcpy(output_image->data + output_image->size, buffer->data, buffer->length);
      output_image->size += buffer->length;
      mmal_buffer_header_release(buffer);
    }

    // Send empty buffers to the output port of the decoder
    while ((buffer = mmal_queue_get(pool_out->queue)) != NULL) {
      status = mmal_port_send_buffer(encoder->output[0], buffer);
      assert(status == MMAL_SUCCESS);
    }
  }
}
