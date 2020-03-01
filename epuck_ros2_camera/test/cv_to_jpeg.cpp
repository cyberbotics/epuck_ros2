#include <chrono>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

extern "C" {
    #include "pipuck_image.h"
    #include "pipuck_jpeg.h"
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
int main() {
    // Init pipuck images
    pipuck_image_t input_image;
    pipuck_image_t output_image;
    char output_image_buffer[500 * 1024];
    
    pipuck_image_init(&input_image);
    pipuck_image_init(&output_image);
    
    output_image.quality = 10;
    output_image.data = output_image_buffer;
    input_image.encoding = PIPUCK_IMAGE_ENCODING_BGR24;

    // Read image
    cv::Mat cv_image = cv::imread("lena.jpg");
    cv::Mat cv_image_resized;
    cv::resize(cv_image, cv_image_resized, cv::Size(input_image.width, input_image.height));
    
    // Convert the image
    input_image.data = (char *)cv_image_resized.data;
    pipuck_jpeg_init(&input_image, &output_image);

    convert(&input_image, &output_image, "image1.jpg");
    convert(&input_image, &output_image, "image2.jpg");

    pipuck_jpeg_deinit();
}