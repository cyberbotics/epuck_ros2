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

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

extern "C" {
#include "epuck_ros2_camera/pipuck_image.h"
#include "epuck_ros2_camera/pipuck_jpeg.h"
#include "epuck_ros2_camera/pipuck_ov7670.h"
#include "epuck_ros2_camera/pipuck_v4l2.h"
}

enum CameraMode { CAMERA_MODE_NONE = 0, CAMERA_MODE_RAW_RGB, CAMERA_MODE_COMPRESSED_JPEG };

class CameraPublisher : public rclcpp::Node {
public:
  CameraPublisher() : Node("camera_publisher"), mMode(CAMERA_MODE_NONE) {
    // Add parameters
    auto quality = declare_parameter<int>("quality", 8);
    auto interval = declare_parameter<int>("interval", 80);

    // Configure input/output formats
    pipuck_image_init(&mCapturedImage);
    pipuck_image_init(&mCompressedImage);
    mCompressedImage.quality = quality;
    mCompressedImage.data = mImageBuffer;
    mRgbEncodedImage.data = mRgbImageBuffer;
    mCompressedImage.encoding = PIPUCK_IMAGE_ENCODING_JPEG;
    mRgbEncodedImage.encoding = PIPUCK_IMAGE_ENCODING_RGB24;
    mCapturedImage.encoding = PIPUCK_IMAGE_ENCODING_YUYV;

    // Prepare ROS topics
    mCallbackHandler =
      this->add_on_set_parameters_callback(std::bind(&CameraPublisher::param_change_callback, this, std::placeholders::_1));
    mPublisherCompressed = this->create_publisher<sensor_msgs::msg::CompressedImage>("image_raw/compressed", 0);
    mPublisherRaw = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 0);
    mTimer = this->create_wall_timer(std::chrono::milliseconds(interval), std::bind(&CameraPublisher::timer_callback, this));
  }

  ~CameraPublisher() {
    if (mMode == CAMERA_MODE_RAW_RGB || mMode == CAMERA_MODE_COMPRESSED_JPEG) {
      pipuck_v4l2_deinit();
      pipuck_jpeg_deinit();
    }
  }

private:
  rcl_interfaces::msg::SetParametersResult param_change_callback(std::vector<rclcpp::Parameter> parameters) {
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    for (auto parameter : parameters) {
      if (parameter.get_name() == "quality") {
        setMode(CAMERA_MODE_NONE);
        mCompressedImage.quality = parameter.as_int();
        setMode(CAMERA_MODE_COMPRESSED_JPEG);

      } else if (parameter.get_name() == "interval") {
        mTimer->cancel();
        mTimer = this->create_wall_timer(std::chrono::milliseconds(parameter.as_int()),
                                         std::bind(&CameraPublisher::timer_callback, this));
      }

      RCLCPP_INFO(this->get_logger(), "Parameter '%s' has changed to %s", parameter.get_name().c_str(),
                  parameter.value_to_string().c_str());
    }

    return result;
  }

  void setMode(CameraMode mode) {
    // Check if it is different
    if (mode == mMode)
      return;

    // Deinit if previous mode required GPU
    pipuck_jpeg_deinit();
    pipuck_v4l2_deinit();

    // Init new configuration
    if (mMode == CAMERA_MODE_RAW_RGB) {
      pipuck_ov7670_init();
      pipuck_v4l2_init();
      pipuck_jpeg_init(&mCapturedImage, &mRgbEncodedImage);
    } else if (mMode == CAMERA_MODE_COMPRESSED_JPEG) {
      pipuck_ov7670_init();
      pipuck_v4l2_init();
      pipuck_jpeg_init(&mCapturedImage, &mCompressedImage);
    }

    mMode = mode;
  }

  void timer_callback() {
    // Set mode
    if (mPublisherCompressed->get_subscription_count() > 0)
      setMode(CAMERA_MODE_COMPRESSED_JPEG);
    else if (mPublisherRaw->get_subscription_count() > 0)
      setMode(CAMERA_MODE_RAW_RGB);
    else
      setMode(CAMERA_MODE_NONE);

    // Publish image
    if (mMode == CAMERA_MODE_RAW_RGB) {
      pipuck_v4l2_capture(&mCapturedImage);
      pipuck_jpeg_encode(&mCapturedImage, &mRgbEncodedImage);

      auto message = sensor_msgs::msg::Image();
      message.encoding = "rgb8";
      message.width = mCapturedImage.width;
      message.height = mCapturedImage.height;
      message.step = mCapturedImage.width * 3;
      message.is_bigendian = false;
      message.header.stamp = now();
      message.header.frame_id = "pipuck_image_raw";
      message.data.assign(mRgbEncodedImage.data, mRgbEncodedImage.data + mCapturedImage.height * mCapturedImage.width * 3);
    } else if (mMode == CAMERA_MODE_COMPRESSED_JPEG) {
      pipuck_v4l2_capture(&mCapturedImage);
      pipuck_jpeg_encode(&mCapturedImage, &mCompressedImage);

      auto message = sensor_msgs::msg::CompressedImage();
      message.format = "jpeg";
      message.header.stamp = now();
      message.header.frame_id = "pipuck_image_compressed";
      message.data.assign(mCompressedImage.data, mCompressedImage.data + mCompressedImage.size);

      mPublisherCompressed->publish(message);
    }
  }
  rclcpp::TimerBase::SharedPtr mTimer;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr mPublisherCompressed;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mPublisherRaw;
  pipuck_image_t mCapturedImage;
  pipuck_image_t mCompressedImage;
  pipuck_image_t mRgbEncodedImage;
  OnSetParametersCallbackHandle::SharedPtr mCallbackHandler;
  char mImageBuffer[900 * 1024];
  char mRgbImageBuffer[640 * 480 * 3 + 1000];
  bool mV4l2Initialized;
  bool mJpegInitialized;
  CameraMode mMode;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraPublisher>());
  rclcpp::shutdown();
  return 0;
}
