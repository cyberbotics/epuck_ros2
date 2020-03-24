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
#include "epuck_ros2_camera/pipuck_mmal.h"
#include "epuck_ros2_camera/pipuck_ov7670.h"
#include "epuck_ros2_camera/pipuck_v4l2.h"
}

class CameraPublisher : public rclcpp::Node {
public:
  CameraPublisher() : Node("epuck_ros2_camera"), mV4l2Initialized(false), mJpegInitialized(false), mRgbInitialized(false) {
    // Add parameters
    auto quality = declare_parameter<int>("quality", 8);
    auto interval = declare_parameter<int>("interval", 80);

    // MMAL JPEG
    pipuck_mmal_create(&mPipuckMmalJpeg);
    strcpy(mPipuckMmalJpeg.component, "vc.ril.image_encode");
    mPipuckMmalJpeg.output.data = mJpegImageBuffer;
    mPipuckMmalJpeg.output.encoding = MMAL_ENCODING_JPEG;

    // MMAL RGB
    pipuck_mmal_create(&mPipuckMmalRgb);
    strcpy(mPipuckMmalRgb.component, "vc.ril.isp");
    mPipuckMmalRgb.output.data = mRgbImageBuffer;
    mPipuckMmalRgb.output.encoding = MMAL_ENCODING_RGB24;

    // Prepare ROS topics
    mCallbackHandler =
      this->add_on_set_parameters_callback(std::bind(&CameraPublisher::param_change_callback, this, std::placeholders::_1));
    mPublisherCompressed = this->create_publisher<sensor_msgs::msg::CompressedImage>("image_raw/compressed", 0);
    mPublisherRaw = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 0);
    mTimer = this->create_wall_timer(std::chrono::milliseconds(interval), std::bind(&CameraPublisher::timerCallback, this));
    RCLCPP_INFO(this->get_logger(), "E-puck2 camera is ready");
  }

  ~CameraPublisher() {
    deinitV4l2();
    deinitRgb();
    deinitJpeg();
  }

private:
  rcl_interfaces::msg::SetParametersResult param_change_callback(std::vector<rclcpp::Parameter> parameters) {
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    for (auto parameter : parameters) {
      if (parameter.get_name() == "quality") {
        deinitJpeg();
        mPipuckMmalJpeg.output.quality = parameter.as_int();
        initJpeg();
      } else if (parameter.get_name() == "interval") {
        mTimer->cancel();
        mTimer = this->create_wall_timer(std::chrono::milliseconds(parameter.as_int()),
                                         std::bind(&CameraPublisher::timerCallback, this));
      }

      RCLCPP_INFO(this->get_logger(), "Parameter '%s' has changed to %s", parameter.get_name().c_str(),
                  parameter.value_to_string().c_str());
    }

    return result;
  }

  void timerCallback() {
    // Initialize V4L2 if needed
    if (mPublisherCompressed->get_subscription_count() > 0 || mPublisherRaw->get_subscription_count()) {
      initV4l2();
      pipuck_v4l2_capture(&(mPipuckMmalRgb.input));
    } else {
      deinitV4l2();
      return;
    }

        // Publish RAW RGB image if needed
    if (mPublisherRaw->get_subscription_count() > 0) {
      initRgb();

      pipuck_mmal_convert(&mPipuckMmalRgb);
      auto message = sensor_msgs::msg::Image();
      message.encoding = "rgb8";
      message.width = mPipuckMmalRgb.output.width;
      message.height = mPipuckMmalRgb.output.height;
      message.step = mPipuckMmalRgb.output.width * 3;
      message.is_bigendian = false;
      message.header.stamp = now();
      message.header.frame_id = "pipuck_image_raw";
      message.data.assign(mPipuckMmalRgb.output.data,
                          mPipuckMmalRgb.output.data + mPipuckMmalRgb.output.height * mPipuckMmalRgb.output.width * 3);
    } else
      deinitRgb();

    // Publish JPEG compressed image if needed
    if (mPublisherCompressed->get_subscription_count() > 0) {
      initJpeg();

      // It's the same data for both topics
      mPipuckMmalJpeg.input.data = mPipuckMmalRgb.input.data;
      pipuck_mmal_convert(&mPipuckMmalJpeg);
      auto message = sensor_msgs::msg::CompressedImage();
      message.format = "jpeg";
      message.header.stamp = now();
      message.header.frame_id = "pipuck_image_compressed";
      message.data.assign(mPipuckMmalJpeg.output.data, mPipuckMmalJpeg.output.data + mPipuckMmalJpeg.output.size);
      mPublisherCompressed->publish(message);
    } else
      deinitJpeg();
  }

  void initV4l2() {
    if (!mV4l2Initialized) {
      pipuck_v4l2_init();
      mV4l2Initialized = true;
      RCLCPP_INFO(this->get_logger(), "V4L2 component initialized");
    }
  }

  void deinitV4l2() {
    if (mV4l2Initialized) {
      pipuck_v4l2_deinit();
      mV4l2Initialized = false;
      RCLCPP_INFO(this->get_logger(), "V4L2 component deinitialized");
      deinitJpeg();
      deinitRgb();
    }
  }

  void deinitJpeg() {
    if (mJpegInitialized) {
      pipuck_mmal_deinit(&mPipuckMmalJpeg);
      mJpegInitialized = false;
      RCLCPP_INFO(this->get_logger(), "MMAL JPEG component deinitialized");
    }
  }

  void initJpeg() {
    if (!mJpegInitialized) {
      pipuck_mmal_init(&mPipuckMmalJpeg);
      mJpegInitialized = true;
      RCLCPP_INFO(this->get_logger(), "MMAL JPEG component initialized");
    }
  }

  void deinitRgb() {
    if (mRgbInitialized) {
      pipuck_mmal_deinit(&mPipuckMmalRgb);
      mRgbInitialized = false;
      RCLCPP_INFO(this->get_logger(), "MMAL RGB component deinitialized");
    }
  }

  void initRgb() {
    if (!mRgbInitialized) {
      pipuck_mmal_init(&mPipuckMmalRgb);
      mRgbInitialized = true;
      RCLCPP_INFO(this->get_logger(), "MMAL RGB component initialized");
    }
  }

  rclcpp::TimerBase::SharedPtr mTimer;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr mPublisherCompressed;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mPublisherRaw;
  OnSetParametersCallbackHandle::SharedPtr mCallbackHandler;
  char mJpegImageBuffer[900 * 1024];
  char mRgbImageBuffer[640 * 480 * 3 + 1000];
  bool mV4l2Initialized;
  bool mJpegInitialized;
  bool mRgbInitialized;
  pipuck_mmal_t mPipuckMmalJpeg;
  pipuck_mmal_t mPipuckMmalRgb;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraPublisher>());
  rclcpp::shutdown();
  return 0;
}
