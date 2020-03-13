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

#include "epuck_ros2_camera/pipuck_v4l2.h"

#include <assert.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/v4l2-common.h>
#include <linux/v4l2-controls.h>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <string.h>

static int f;
static int status;
static char * buffer;
static struct v4l2_buffer bufferinfo;

void pipuck_v4l2_init()
{
  f = open("/dev/video0", O_RDWR);
  assert(f != 0);

  // Set device in capture mode
  struct v4l2_capability capability;
  status = ioctl(f, VIDIOC_QUERYCAP, &capability);
  assert(status >= 0);

  // Set image format
  struct v4l2_format image_format;
  image_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  image_format.fmt.pix.width = 640;
  image_format.fmt.pix.height = 480;
  image_format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV422P;
  image_format.fmt.pix.field = V4L2_FIELD_NONE;
  status = ioctl(f, VIDIOC_S_FMT, &image_format);
  assert(status >= 0);

  // Request buffers
  struct v4l2_requestbuffers requestbuffers = {0};
  requestbuffers.count = 1;
  requestbuffers.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  requestbuffers.memory = V4L2_MEMORY_MMAP;
  status = ioctl(f, VIDIOC_REQBUFS, &requestbuffers);
  assert(status >= 0);

  // Query the buffer
  struct v4l2_buffer querybuffer = {0};
  querybuffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  querybuffer.memory = V4L2_MEMORY_MMAP;
  querybuffer.index = 0;
  status = ioctl(f, VIDIOC_QUERYBUF, &querybuffer);
  assert(status >= 0);

  // Get buffer pointer
  buffer = mmap(NULL, querybuffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, f,
      querybuffer.m.offset);
  memset(buffer, 0, querybuffer.length);

  // Capture
  memset(&bufferinfo, 0, sizeof(bufferinfo));
  bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  bufferinfo.memory = V4L2_MEMORY_MMAP;
  bufferinfo.index = 0;
}

void pipuck_v4l2_capture(pipuck_image_t * image)
{
  status = ioctl(f, VIDIOC_STREAMON, &bufferinfo.type);
  assert(status >= 0);

  // Queue the buffer
  status = ioctl(f, VIDIOC_QBUF, &bufferinfo);
  assert(status >= 0);

  // Dequeue the buffer
  status = ioctl(f, VIDIOC_DQBUF, &bufferinfo);
  assert(status >= 0);

  image->size = bufferinfo.bytesused;
  image->data = buffer;
}

void pipuck_v4l2_deinit()
{
  status = ioctl(f, VIDIOC_STREAMOFF, &bufferinfo.type);
  assert(status >= 0);
}
