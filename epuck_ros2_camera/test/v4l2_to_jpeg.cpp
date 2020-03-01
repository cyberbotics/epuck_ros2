#include <chrono>
#include <fstream>
#include <iostream>

extern "C" {
  #include "epuck_ros2_camera/pipuck_image.h"
  #include "epuck_ros2_camera/pipuck_jpeg.h"
  #include "epuck_ros2_camera/pipuck_ov7670.h"
  #include "epuck_ros2_camera/pipuck_v4l2.h"
}

void convert(pipuck_image_t* input_image, pipuck_image_t* output_image, std::string output_filename) {
  std::ofstream file;
  file.open(output_filename, std::ios::out | std::ios::binary);

  auto tick = std::chrono::high_resolution_clock::now();
  pipuck_jpeg_encode(input_image, output_image);
  auto tock = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::microseconds>(tock - tick)
          .count();

  std::cout << "Time taken for conversion: " << duration / 1000 << "ms"
            << std::endl;
  std::cout << "Image size: " << output_image->size << " bytes" << std::endl;

  file.write(output_image->data, output_image->size);
  file.close();

  std::cout << "Compressed image written to " << output_filename << std::endl;
}

void capture(pipuck_image_t* input_image, std::string output_filename) {
  std::ofstream file;

  file.open(output_filename, std::ios::out | std::ios::binary);

  // Capture the image
  auto tick = std::chrono::high_resolution_clock::now();
  pipuck_v4l2_capture(input_image);
  auto tock = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::microseconds>(tock - tick)
          .count();
  std::cout << "Time taken to capture the image: " << duration / 1000 << "ms"
            << std::endl;
  std::cout << "Captured image size: " << input_image->size << " bytes" << std::endl;

  file.write(input_image->data, input_image->size);
  file.close();
  std::cout << "Raw image written to " << output_filename << std::endl;
}

int main() {
  pipuck_image_t input_image;
  pipuck_image_t output_image;
  char output_image_buffer[500 * 1024];

  pipuck_image_init(&input_image);
  pipuck_image_init(&output_image);

  output_image.quality = 10;
  output_image.data = output_image_buffer;
  input_image.encoding = PIPUCK_IMAGE_ENCODING_YUYV;

  pipuck_ov7670_init();
  pipuck_v4l2_init();
  pipuck_jpeg_init(&input_image, &output_image);
  
  capture(&input_image, "image1.raw");
  convert(&input_image, &output_image, "image1.jpg");

  capture(&input_image, "image2.raw");
  convert(&input_image, &output_image, "image2.jpg");

  pipuck_v4l2_deinit();
  pipuck_jpeg_deinit();

  return 0;
}