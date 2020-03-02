#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

extern "C"
{
#include "epuck_ros2_camera/pipuck_image.h"
#include "epuck_ros2_camera/pipuck_v4l2.h"
#include "epuck_ros2_camera/pipuck_jpeg.h"
#include "epuck_ros2_camera/pipuck_ov7670.h"
}

using namespace std::chrono_literals;

class CameraPublisher : public rclcpp::Node
{
public:
  CameraPublisher() : Node("camera_publisher"), v4l2_initialized(false), jpeg_initialized(false)
  {
    auto quality = declare_parameter<int>("quality", 8);
    auto interval = declare_parameter<int>("interval", 80);

    pipuck_image_init(&captured_image);
    pipuck_image_init(&compressed_image);

    compressed_image.quality = quality;
    compressed_image.data = compressed_image_buffer;
    captured_image.encoding = PIPUCK_IMAGE_ENCODING_YUYV;

    pipuck_ov7670_init();

    callback_handler = this->add_on_set_parameters_callback(std::bind(&CameraPublisher::param_change_callback, this, std::placeholders::_1));

    publisher_compressed = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        "image_raw/compressed", 0);
    publisher_raw = this->create_publisher<sensor_msgs::msg::Image>(
        "image_raw", 0);
    timer = this->create_wall_timer(
        std::chrono::milliseconds(interval), std::bind(&CameraPublisher::timer_callback, this));
  }

  ~CameraPublisher()
  {
    pipuck_v4l2_deinit();
    pipuck_jpeg_deinit();
  }

private:
  rcl_interfaces::msg::SetParametersResult param_change_callback(std::vector<rclcpp::Parameter> parameters)
  {
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    for (auto parameter : parameters)
    {
      if (parameter.get_name() == "quality")
      {
        if (jpeg_initialized == true)
        {
          pipuck_jpeg_deinit();
        }
        compressed_image.quality = parameter.as_int();
        if (jpeg_initialized == true)
        {
          pipuck_jpeg_init(&captured_image, &compressed_image);
        }
      }
      else if (parameter.get_name() == "interval")
      {
        timer->cancel();
        timer = this->create_wall_timer(
            std::chrono::milliseconds(parameter.as_int()), std::bind(&CameraPublisher::timer_callback, this));
      }

      RCLCPP_INFO(
          this->get_logger(),
          "Parameter '%s' has changed to %s",
          parameter.get_name().c_str(),
          parameter.value_to_string().c_str());
    }

    return result;
  }

  void timer_callback()
  {
    if (publisher_compressed->get_subscription_count() > 0 || publisher_raw->get_subscription_count())
    {
      if (v4l2_initialized == false)
      {
        pipuck_v4l2_init();
        v4l2_initialized = true;
      }
      pipuck_v4l2_capture(&captured_image);
    }
    else
    {
      if (v4l2_initialized == true)
      {
        pipuck_v4l2_deinit();
        v4l2_initialized = false;
      }
    }

    if (publisher_raw->get_subscription_count() > 0)
    {
      // TODO: Publish raw
    }

    if (publisher_compressed->get_subscription_count() > 0)
    {
      if (jpeg_initialized == false)
      {
        pipuck_jpeg_init(&captured_image, &compressed_image);
        jpeg_initialized = true;
      }
      pipuck_jpeg_encode(&captured_image, &compressed_image);

      auto message = sensor_msgs::msg::CompressedImage();
      message.format = "jpeg";
      message.header.stamp = now();
      message.header.frame_id = "pipuck_image";
      message.data.assign(compressed_image.data,
                          compressed_image.data + compressed_image.size);

      publisher_compressed->publish(message);
    }
    else
    {
      if (jpeg_initialized == true)
      {
        pipuck_jpeg_deinit();
        jpeg_initialized = false;
      }
    }
  }
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_compressed;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_raw;
  pipuck_image_t captured_image;
  pipuck_image_t compressed_image;
  OnSetParametersCallbackHandle::SharedPtr callback_handler;
  char compressed_image_buffer[500 * 1024];
  bool v4l2_initialized;
  bool jpeg_initialized;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraPublisher>());
  rclcpp::shutdown();
  return 0;
}