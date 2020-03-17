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
#include <opencv2/opencv.hpp>
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

using namespace std::chrono_literals;

class CameraPublisher : public rclcpp::Node {
public:
  CameraPublisher() : Node("camera_publisher"), mV4l2Initialized(false), mJpegInitialized(false) {
    auto quality = declare_parameter<int>("quality", 8);
    auto interval = declare_parameter<int>("interval", 80);

    pipuck_image_init(&mCapturedImage);
    pipuck_image_init(&compressedImage);

    compressedImage.quality = quality;
    compressedImage.data = imageBuffer;
    mCapturedImage.encoding = PIPUCK_IMAGE_ENCODING_YUYV;

    pipuck_ov7670_init();

    mCallbackHandler =
      this->add_on_set_parameters_callback(std::bind(&CameraPublisher::param_change_callback, this, std::placeholders::_1));

    mPublisherCompressed = this->create_publisher<sensor_msgs::msg::CompressedImage>("image_raw/compressed", 0);
    mPublisherRaw = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 0);
    mTimer = this->create_wall_timer(std::chrono::milliseconds(interval), std::bind(&CameraPublisher::timer_callback, this));
  }

  ~CameraPublisher() {
    if (mV4l2Initialized)
      pipuck_v4l2_deinit();

    if (mJpegInitialized)
      pipuck_jpeg_deinit();
  }

private:
  rcl_interfaces::msg::SetParametersResult param_change_callback(std::vector<rclcpp::Parameter> parameters) {
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    for (auto parameter : parameters) {
      if (parameter.get_name() == "quality") {
        if (mJpegInitialized)
          pipuck_jpeg_deinit();

        compressedImage.quality = parameter.as_int();
        if (mJpegInitialized)
          pipuck_jpeg_init(&mCapturedImage, &compressedImage);

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

  void timer_callback() {
    if (mPublisherCompressed->get_subscription_count() > 0 || mPublisherRaw->get_subscription_count()) {
      if (!mV4l2Initialized) {
        pipuck_v4l2_init();
        mV4l2Initialized = true;
      }
      pipuck_v4l2_capture(&mCapturedImage);
    } else if (mV4l2Initialized) {
      pipuck_v4l2_deinit();
      mV4l2Initialized = false;
    }

    if (mPublisherRaw->get_subscription_count() > 0) {
      cv::Mat inputMat(mCapturedImage.height, mCapturedImage.width, CV_8UC2, mCapturedImage.data);
      cv::Mat outputMat;
      // COLOR_YUV2BGR_Y422
      cv::cvtColor(inputMat, outputMat, cv::COLOR_YUV2BGR_YUY2);

      auto message = sensor_msgs::msg::Image();
      message.encoding = "rgb8";
      message.width = mCapturedImage.width;
      message.height = mCapturedImage.height;
      message.step = mCapturedImage.width * 3;
      message.is_bigendian = false;
      message.header.stamp = now();
      message.header.frame_id = "pipuck_image_raw";
      message.data.assign(outputMat.data, outputMat.data + mCapturedImage.height * mCapturedImage.width * 3);
    }

    if (mPublisherCompressed->get_subscription_count() > 0) {
      if (!mJpegInitialized) {
        pipuck_jpeg_init(&mCapturedImage, &compressedImage);
        mJpegInitialized = true;
      }
      pipuck_jpeg_encode(&mCapturedImage, &compressedImage);

      auto message = sensor_msgs::msg::CompressedImage();
      message.format = "jpeg";
      message.header.stamp = now();
      message.header.frame_id = "pipuck_image_compressed";
      message.data.assign(compressedImage.data, compressedImage.data + compressedImage.size);

      mPublisherCompressed->publish(message);
    } else if (mJpegInitialized) {
      pipuck_jpeg_deinit();
      mJpegInitialized = false;
    }
  }
  rclcpp::TimerBase::SharedPtr mTimer;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr mPublisherCompressed;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mPublisherRaw;
  pipuck_image_t mCapturedImage;
  pipuck_image_t compressedImage;
  OnSetParametersCallbackHandle::SharedPtr mCallbackHandler;
  char imageBuffer[900 * 1024];
  bool mV4l2Initialized;
  bool mJpegInitialized;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraPublisher>());
  rclcpp::shutdown();
  return 0;
}
