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
// created for this project) to convert OpenCV image (`cv::Mat`) to JPEG by
// utilising onboard GPU.

extern "C" {
#include "epuck_ros2_camera/pipuck_image.h"
#include "epuck_ros2_camera/pipuck_mmal.h"
}

#include <chrono>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

void convert(pipuck_mmal_t *pipuck_mmal, std::string output_filename) {
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
int main() {
  pipuck_mmal_t pipuck_mmal_jpeg;
  char output_buffer[640 * 480 * 3];

  // MMAL Init: JPEG
  pipuck_mmal_create(&pipuck_mmal_jpeg);
  strcpy(pipuck_mmal_jpeg.component, "vc.ril.image_encode");
  pipuck_mmal_jpeg.input.encoding = MMAL_ENCODING_BGR24;
  pipuck_mmal_jpeg.output.data = output_buffer;
  pipuck_mmal_jpeg.output.encoding = MMAL_ENCODING_JPEG;
  pipuck_mmal_init(&pipuck_mmal_jpeg);
  std::cout << "JPEG convertor is initialized" << std::endl;

  // Read image
  cv::Mat cv_image = cv::imread("lena.jpg");
  cv::Mat cv_image_resized;
  cv::resize(cv_image, cv_image_resized, cv::Size(pipuck_mmal_jpeg.input.width, pipuck_mmal_jpeg.input.height));

  // Convert the image
  pipuck_mmal_jpeg.input.data = (char *)cv_image_resized.data;
  convert(&pipuck_mmal_jpeg, "image1.jpg");
  convert(&pipuck_mmal_jpeg, "image2.jpg");

  pipuck_mmal_deinit(&pipuck_mmal_jpeg);
}
