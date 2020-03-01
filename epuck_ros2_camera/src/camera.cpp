#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

extern "C" {
#include "epuck_ros2_camera/pipuck_image.h"
#include "epuck_ros2_camera/pipuck_v4l2.h"
#include "epuck_ros2_camera/pipuck_jpeg.h"
#include "epuck_ros2_camera/pipuck_ov7670.h"
}

using namespace std::chrono_literals;

class CameraPublisher : public rclcpp::Node {
public:
  CameraPublisher() : Node("camera_publisher"), count(0) {
    pipuck_image_init(&captured_image);
    pipuck_image_init(&compressed_image);

    compressed_image.quality = 10;
    compressed_image.data = compressed_image_buffer;
    captured_image.encoding = PIPUCK_IMAGE_ENCODING_YUYV;

    pipuck_ov7670_init();
    pipuck_v4l2_init();
    pipuck_jpeg_init(&captured_image, &compressed_image);

    publisher = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        "image_raw/compressed", 1);
    timer = this->create_wall_timer(
        100ms, std::bind(&CameraPublisher::timer_callback, this));
  }

  ~CameraPublisher() {
    pipuck_v4l2_deinit();
    pipuck_jpeg_deinit();
  }

private:
  void timer_callback() {
    pipuck_v4l2_capture(&captured_image);
    pipuck_jpeg_encode(&captured_image, &compressed_image);

    auto message = sensor_msgs::msg::CompressedImage();
    message.format = "jpeg";
    message.header.stamp = now();
    message.header.frame_id = "pipuck_image";
    message.data.assign(compressed_image.data,
                        compressed_image.data + compressed_image.size);

    RCLCPP_INFO(this->get_logger(), "Publishing: %d", count);
    publisher->publish(message);
    count++;
  }
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher;
  size_t count;
  pipuck_image_t captured_image;
  pipuck_image_t compressed_image;
  char compressed_image_buffer[500 * 1024];
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraPublisher>());
  rclcpp::shutdown();
  return 0;
}