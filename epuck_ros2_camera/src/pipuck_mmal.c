// Copyright 1996-2020 Cyberbotics
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

#include "epuck_ros2_camera/pipuck_mmal.h"

#include <assert.h>
#include <bcm_host.h>
#include <interface/mmal/mmal_buffer.h>
#include <interface/mmal/mmal_logging.h>
#include <interface/mmal/util/mmal_connection.h>
#include <interface/mmal/util/mmal_default_components.h>
#include <interface/mmal/util/mmal_util.h>
#include <interface/mmal/util/mmal_util_params.h>

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))

static void release_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);
static void put_to_queue_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);

void put_to_queue_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
  pipuck_mmal_t *pipuck_mmal = (pipuck_mmal_t *)port->userdata;

  mmal_queue_put(pipuck_mmal->internal.queue, buffer);
  vcos_semaphore_post(&(pipuck_mmal->internal.semaphore));
}

void release_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
  pipuck_mmal_t *pipuck_mmal = (pipuck_mmal_t *)port->userdata;

  mmal_buffer_header_release(buffer);
  vcos_semaphore_post(&(pipuck_mmal->internal.semaphore));
}

void pipuck_mmal_create(pipuck_mmal_t *pipuck_mmal) {
  pipuck_image_t *input_image = &(pipuck_mmal->input);
  pipuck_image_t *output_image = &(pipuck_mmal->output);

  // Set defaults
  input_image->width = 640;
  input_image->height = 480;
  input_image->size = input_image->width * input_image->height * 2;
  input_image->encoding = MMAL_ENCODING_YUYV;
  output_image->width = 640;
  output_image->height = 480;
  output_image->quality = 10;
  output_image->encoding = MMAL_ENCODING_RGB24;
  strcpy(pipuck_mmal->component, "vc.ril.isp");
}

void pipuck_mmal_init(pipuck_mmal_t *pipuck_mmal) {
  MMAL_STATUS_T status;
  pipuck_image_t *input_image = &(pipuck_mmal->input);
  pipuck_image_t *output_image = &(pipuck_mmal->output);
  pipuck_mmal_internal_t *context = &(pipuck_mmal->internal);

  // Initialize core components
  bcm_host_init();
  vcos_init();
  vcos_semaphore_create(&(pipuck_mmal->internal.semaphore), "sync", 1);
  status = mmal_component_create(pipuck_mmal->component, &(context->encoder));
  assert(status == MMAL_SUCCESS);
  context->encoder->control->userdata = (void *)pipuck_mmal;
  status = mmal_port_enable(context->encoder->control, release_buffer_callback);
  assert(status == MMAL_SUCCESS);

  // Configure encoder input
  context->encoder->input[0]->format->encoding = input_image->encoding;
  context->encoder->input[0]->format->es->video.crop.width = input_image->width;
  context->encoder->input[0]->format->es->video.crop.height = input_image->height;
  context->encoder->input[0]->format->es->video.width = VCOS_ALIGN_UP(input_image->width, 32);
  context->encoder->input[0]->format->es->video.height = VCOS_ALIGN_UP(input_image->height, 16);
  context->encoder->input[0]->format->es->video.frame_rate.num = 0;
  context->encoder->input[0]->format->es->video.frame_rate.den = 1;
  context->encoder->input[0]->format->es->video.par.num = 1;
  context->encoder->input[0]->format->es->video.par.den = 1;
  status = mmal_port_format_commit(context->encoder->input[0]);
  assert(status == MMAL_SUCCESS);

  // Configure encoder output
  mmal_format_copy(context->encoder->output[0]->format, context->encoder->input[0]->format);
  context->encoder->output[0]->format->encoding = output_image->encoding;
  context->encoder->output[0]->format->es->video.crop.width = output_image->width;
  context->encoder->output[0]->format->es->video.crop.height = output_image->height;
  context->encoder->output[0]->format->es->video.width = VCOS_ALIGN_UP(output_image->width, 32);
  context->encoder->output[0]->format->es->video.height = VCOS_ALIGN_UP(output_image->height, 16);
  status = mmal_port_format_commit(context->encoder->output[0]);
  assert(status == MMAL_SUCCESS);
  if (output_image->encoding == MMAL_ENCODING_JPEG) {
    status = mmal_port_parameter_set_uint32(context->encoder->output[0], MMAL_PARAMETER_JPEG_Q_FACTOR, output_image->quality);
    assert(status == MMAL_SUCCESS);
  }

  // Configure buffers
  context->encoder->input[0]->buffer_num = context->encoder->input[0]->buffer_num_recommended;
  context->encoder->input[0]->buffer_size = context->encoder->input[0]->buffer_size_recommended;
  context->encoder->output[0]->buffer_num = context->encoder->output[0]->buffer_num_recommended;
  context->encoder->output[0]->buffer_size = context->encoder->output[0]->buffer_size_recommended;

  // Enable ports
  context->encoder->input[0]->userdata = (void *)pipuck_mmal;
  status = mmal_port_enable(context->encoder->input[0], release_buffer_callback);
  assert(status == MMAL_SUCCESS);
  context->encoder->output[0]->userdata = (void *)pipuck_mmal;
  status = mmal_port_enable(context->encoder->output[0], put_to_queue_callback);
  assert(status == MMAL_SUCCESS);

  // Create a buffer pool
  context->queue = mmal_queue_create();
  pipuck_mmal->internal.pool_in = mmal_port_pool_create(context->encoder->input[0], context->encoder->input[0]->buffer_num,
                                                        context->encoder->input[0]->buffer_size);
  assert(context->pool_in != 0);
  context->pool_out = mmal_port_pool_create(context->encoder->output[0], context->encoder->output[0]->buffer_num,
                                            context->encoder->output[0]->buffer_size);
  assert(context->pool_out != 0);

  // Start component
  status = mmal_component_enable(context->encoder);
  assert(status == MMAL_SUCCESS);
}

void pipuck_mmal_deinit(pipuck_mmal_t *pipuck_mmal) {
  mmal_port_disable(pipuck_mmal->internal.encoder->input[0]);
  mmal_port_disable(pipuck_mmal->internal.encoder->output[0]);
  mmal_component_disable(pipuck_mmal->internal.encoder);
  mmal_queue_destroy(pipuck_mmal->internal.queue);
  mmal_port_pool_destroy(pipuck_mmal->internal.encoder->input[0], pipuck_mmal->internal.pool_in);
  mmal_port_pool_destroy(pipuck_mmal->internal.encoder->output[0], pipuck_mmal->internal.pool_out);
  vcos_semaphore_delete(&(pipuck_mmal->internal.semaphore));
}

int pipuck_mmal_convert(pipuck_mmal_t *pipuck_mmal) {
  MMAL_BOOL_T eos = MMAL_FALSE;
  int sent_bytes = 0;
  MMAL_BUFFER_HEADER_T *buffer;
  MMAL_STATUS_T status;

  pipuck_image_t *output_image = &(pipuck_mmal->output);
  pipuck_image_t *input_image = &(pipuck_mmal->input);
  pipuck_mmal_internal_t *context = &(pipuck_mmal->internal);

  output_image->size = 0;

  while (eos == MMAL_FALSE) {
    VCOS_STATUS_T vcos_status = vcos_semaphore_wait_timeout(&(context->semaphore), 2000);
    assert(vcos_status == VCOS_SUCCESS);

    // Input frames
    if ((buffer = mmal_queue_get(context->pool_in->queue)) != NULL) {
      buffer->length = MAX(0, MIN(buffer->alloc_size, input_image->size - sent_bytes));
      buffer->offset = 0;
      buffer->pts = MMAL_TIME_UNKNOWN;
      buffer->dts = MMAL_TIME_UNKNOWN;
      buffer->data = input_image->data + sent_bytes;
      sent_bytes += buffer->length;
      status = mmal_port_send_buffer(context->encoder->input[0], buffer);
      assert(status == MMAL_SUCCESS);
    }

    // Output frames
    while ((buffer = mmal_queue_get(context->queue)) != NULL) {
      eos = buffer->flags & MMAL_BUFFER_HEADER_FLAG_FRAME_END;
      memcpy(output_image->data + output_image->size, buffer->data, buffer->length);
      output_image->size += buffer->length;
      mmal_buffer_header_release(buffer);
    }

    // Send empty buffers to the output port of the decoder
    while ((buffer = mmal_queue_get(context->pool_out->queue)) != NULL) {
      status = mmal_port_send_buffer(context->encoder->output[0], buffer);
      assert(status == MMAL_SUCCESS);
    }
  }

  return 0;
}
