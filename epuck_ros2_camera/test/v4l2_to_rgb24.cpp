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

// This is an example of using pi-puck image related libraries (specifically
// created for this project) to read image directly from v4l2 kernel module
// and to convert it to JPEG by utilising onboard GPU.

// You can use this fantastic online tool to explore the image data:
// https://rawpixels.net/

extern "C" {
#include "epuck_ros2_camera/pipuck_image.h"
#include "epuck_ros2_camera/pipuck_jpeg.h"
#include "epuck_ros2_camera/pipuck_ov7670.h"
#include "epuck_ros2_camera/pipuck_v4l2.h"
}

#include <chrono>
#include <fstream>
#include <iostream>

void convert(pipuck_mmal_t* pipuck_mmal, std::string output_filename) {
  std::ofstream file;
  file.open(output_filename, std::ios::out | std::ios::binary);

  auto tick = std::chrono::high_resolution_clock::now();
  pipuck_mmal_convert(pipuck_mmal);
  auto tock = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(tock - tick).count();

  std::cout << "Time taken for conversion: " << duration / 1000 << "ms" << std::endl;
  std::cout << "Image size: " << pipuck_mmal->output.size << " bytes" << std::endl;

  file.write(pipuck_mmal->output.data, pipuck_mmal->output.size);
  file.close();

  std::cout << "Compressed image written to " << output_filename << std::endl;
}

void capture(pipuck_image_t *input_image, std::string output_filename) {
  std::ofstream file;

  file.open(output_filename, std::ios::out | std::ios::binary);

  // Capture the image
  auto tick = std::chrono::high_resolution_clock::now();
  pipuck_v4l2_capture(input_image);
  auto tock = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(tock - tick).count();
  std::cout << "Time taken to capture the image: " << duration / 1000 << "ms" << std::endl;
  std::cout << "Captured image size: " << input_image->size << " bytes" << std::endl;

  file.write(input_image->data, input_image->size);
  file.close();
  std::cout << "Raw image written to " << output_filename << std::endl;
}

int main() {
  pipuck_mmal_t pipuck_mmal;
  // char output_image_buffer[500 * 1024];
  char output_image_buffer[640 * 480 * 4];

  pipuck_mmal_create(&pipuck_mmal);
  pipuck_mmal.output.data = output_image_buffer;

  pipuck_ov7670_init();
  pipuck_v4l2_init();
  pipuck_mmal_init(&pipuck_mmal);

  capture(&(pipuck_mmal.input), "image1.yuv422");
  convert(&pipuck_mmal, "image1.rgb24");

  capture(&(pipuck_mmal.input), "image2.yuv422");
  convert(&pipuck_mmal, "image2.rgb24");

  pipuck_v4l2_deinit();
  pipuck_mmal_deinit(&pipuck_mmal);

  return 0;
}
