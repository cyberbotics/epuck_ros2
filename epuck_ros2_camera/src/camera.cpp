#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

extern "C" {
#include "epuck_ros2_camera/pipuck_image.h"
#include "epuck_ros2_camera/pipuck_v4l2.h"
}

using namespace std::chrono_literals;

class CameraPublisher : public rclcpp::Node {
public:
  CameraPublisher() : Node("camera_publisher"), count(0) {
    pipuck_image_init(&captured_image);
    pipuck_v4l2_init();

    publisher = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        "image_raw/compressed", 1);
    timer = this->create_wall_timer(
        500ms, std::bind(&CameraPublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    pipuck_v4l2_capture(&captured_image);

    auto message = sensor_msgs::msg::CompressedImage();
    message.format = "jpeg";
    message.data.assign(captured_image.data,
                        captured_image.data + captured_image.size);

    RCLCPP_INFO(this->get_logger(), "Publishing: %d", count);
    publisher->publish(message);
    count++;
  }
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher;
  size_t count;
  pipuck_image_t captured_image;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraPublisher>());
  rclcpp::shutdown();

  pipuck_v4l2_deinit();

  return 0;
}