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

extern "C" {
#include <assert.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
}

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

extern "C" {
#include "vl53l0x/tof.h"
}

#include "epuck_ros2_driver/i2c_wrapper.hpp"
#include "epuck_ros2_driver/mpu9250.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/illuminance.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))
#define CLIP(VAL, MIN_VAL, MAX_VAL) MAX(MIN((MAX_VAL), (VAL)), (MIN_VAL))
#define POW2(N) (1 << (N))

#define MSG_ACTUATORS_SIZE 20
#define MSG_SENSORS_SIZE 47
#define PERIOD_MS 64
#define PERIOD_S (PERIOD_MS / 1000.0f)
#define OUT_OF_RANGE 0.0f
#define ENCODER_RESOLUTION 1000.0f
#define ODOM_OVERFLOW_GRACE_TICKS 2000
#define GROUND_SENSOR_ADDRESS 0x60
#define INFRARED_MAX_RANGE 0.025f
#define INFRARED_MIN_RANGE 0.009f
#define TOF_MAX_RANGE 1.0f
#define TOF_MIN_RANGE 0.005f
#define GROUND_MIN_RANGE 0.0f
#define GROUND_MAX_RANGE 0.016f
#define DEFAULT_WHEEL_DISTANCE 0.05685f
#define DEFAULT_WHEEL_RADIUS 0.02f
#define SENSOR_DIST_FROM_CENTER 0.035f
#define NB_LIGHT_SENSORS 8
#define NB_INFRARED_SENSORS 8
#define NB_GROUND_SENSORS 3
#define NB_RGB_LEDS 4
#define NB_BINARY_LEDS 4
#define MSG_SENSORS_ODOMETRY_INDEX 41
#define MSG_ACTUATORS_BIN_LEDS_INDEX 5
#define MSG_ACTUATORS_RGB_LEDS_INDEX 6
#define MSG_ACTUATORS_MOTORS_INDEX 0
#define MSG_SENSORS_LIGHT_INDEX 16
#define MSG_SENSORS_DISTANCE_INDEX 0

const std::vector<std::vector<float>> INFRARED_TABLE = {{0, 4095},      {0.005, 2133.33}, {0.01, 1465.73}, {0.015, 601.46},
                                                        {0.02, 383.84}, {0.03, 234.93},   {0.04, 158.03},  {0.05, 120}};
const std::vector<std::vector<float>> GROUND_TABLE = {{0, 1000}, {0.016, 300}};
const std::vector<double> DISTANCE_SENSOR_ANGLE = {
  -15 * M_PI / 180,   // ps0
  -45 * M_PI / 180,   // ps1
  -90 * M_PI / 180,   // ps2
  -150 * M_PI / 180,  // ps3
  150 * M_PI / 180,   // ps4
  90 * M_PI / 180,    // ps5
  45 * M_PI / 180,    // ps6
  15 * M_PI / 180,    // ps7
  0 * M_PI / 180      // tof
};

class EPuckDriver : public rclcpp::Node {
public:
  EPuckDriver(int argc, char *argv[]) : Node("epuck_driver") {
    // Parse arguments
    std::string type = "hw";
    for (int i = 1; i < argc; i++) {
      if (strcmp(argv[i], "--type") == 0) {
        i++;
        type = argv[i];
      }
    }

    mWheelDistance = declare_parameter<float>("wheel_distance", DEFAULT_WHEEL_DISTANCE);
    mWheelRadius = declare_parameter<float>("wheel_radius", DEFAULT_WHEEL_RADIUS);
    mCallbackHandler =
      add_on_set_parameters_callback(std::bind(&EPuckDriver::paramChangeCallback, this, std::placeholders::_1));

    // Create I2C object
    if (type == "test")
      mI2cMain = std::make_shared<I2CWrapperTest>("/dev/i2c-12");
    else
      mI2cMain = std::make_shared<I2CWrapperHW>("/dev/i2c-12");
    mImu = std::make_shared<MPU9250>(mI2cMain);
    mImu->calibrate();
    mTofInitStatus = tofInit(12, 0x29, 1);
    if (!mTofInitStatus)
      RCLCPP_WARN(get_logger(), "ToF device is not accessible!");

    // Initialize the values
    std::fill(mMsgActuators, mMsgActuators + MSG_ACTUATORS_SIZE, 0);
    std::fill(mMsgSensors, mMsgSensors + MSG_SENSORS_SIZE, 0);
    resetOdometry();
    mI2cMainErrCnt = 0;

    // Create subscirbers and publishers
    mTwistSubscription = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 1, std::bind(&EPuckDriver::onCmdVelReceived, this, std::placeholders::_1));
    for (int i = 0; i < NB_RGB_LEDS; i++) {
      std::function<void(const std_msgs::msg::Int32::SharedPtr)> f =
        std::bind(&EPuckDriver::onRgbLedReceived, this, std::placeholders::_1, i);
      mRgbLedSubscription[i] = create_subscription<std_msgs::msg::Int32>("/led" + std::to_string(i * 2 + 1), 1, f);
    }
    for (int i = 0; i < NB_BINARY_LEDS; i++) {
      std::function<void(const std_msgs::msg::Bool::SharedPtr)> f =
        std::bind(&EPuckDriver::onLedReceived, this, std::placeholders::_1, i);
      mLedSubscription[i] = create_subscription<std_msgs::msg::Bool>("/led" + std::to_string(i * 2), 1, f);
    }
    mLaserPublisher = create_publisher<sensor_msgs::msg::LaserScan>("scan", 1);
    for (int i = 0; i < NB_GROUND_SENSORS; i++)
      mGroundRangePublisher[i] = create_publisher<sensor_msgs::msg::Range>("/gs" + std::to_string(i), 1);
    mOdometryPublisher = create_publisher<nav_msgs::msg::Odometry>("odom", 1);
    for (int i = 0; i < NB_INFRARED_SENSORS; i++) {
      mRangePublisher[i] = create_publisher<sensor_msgs::msg::Range>("ps" + std::to_string(i), 1);
      mIlluminancePublisher[i] = create_publisher<sensor_msgs::msg::Illuminance>("ls" + std::to_string(i), 1);
    }
    mRangeTofPublisher = create_publisher<sensor_msgs::msg::Range>("tof", 1);
    mImuPublisher = create_publisher<sensor_msgs::msg::Imu>("imu", 1);
    mTimer = create_wall_timer(std::chrono::milliseconds(PERIOD_MS), std::bind(&EPuckDriver::updateCallback, this));

    // Dynamic tf broadcaster: Odometry
    mDynamicBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // Static tf broadcaster: Laser
    mLaserBroadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    geometry_msgs::msg::TransformStamped laserTransform;
    laserTransform.header.stamp = now();
    laserTransform.header.frame_id = "base_link";
    laserTransform.child_frame_id = "laser_scanner";
    laserTransform.transform.rotation.x = 0;
    laserTransform.transform.rotation.y = 0;
    laserTransform.transform.rotation.z = 0;
    laserTransform.transform.rotation.w = 1;
    laserTransform.transform.translation.x = 0;
    laserTransform.transform.translation.y = 0;
    laserTransform.transform.translation.z = 0;
    mLaserBroadcaster->sendTransform(laserTransform);

    for (int i = 0; i < NB_INFRARED_SENSORS; i++) {
      // Static tf broadcaster: Range (infrared)
      mInfraredBroadcasters[i] = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
      geometry_msgs::msg::TransformStamped infraredTransform;
      infraredTransform.header.stamp = now();
      infraredTransform.header.frame_id = "base_link";
      infraredTransform.child_frame_id = "ps" + std::to_string(i);
      infraredTransform.transform.rotation = EPuckDriver::euler2quaternion(0, 0, DISTANCE_SENSOR_ANGLE[i]);
      infraredTransform.transform.translation.x = SENSOR_DIST_FROM_CENTER * cos(DISTANCE_SENSOR_ANGLE[i]);
      infraredTransform.transform.translation.y = SENSOR_DIST_FROM_CENTER * sin(DISTANCE_SENSOR_ANGLE[i]);
      infraredTransform.transform.translation.z = 0;
      mInfraredBroadcasters[i]->sendTransform(infraredTransform);

      // Static tf broadcaster: Light sensors
      mLightSensorBroadcasters[i] = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
      geometry_msgs::msg::TransformStamped lightTransform;
      lightTransform.header.stamp = now();
      lightTransform.header.frame_id = "base_link";
      lightTransform.child_frame_id = "ls" + std::to_string(i);
      lightTransform.transform.rotation = EPuckDriver::euler2quaternion(0, 0, DISTANCE_SENSOR_ANGLE[i]);
      lightTransform.transform.translation.x = SENSOR_DIST_FROM_CENTER * cos(DISTANCE_SENSOR_ANGLE[i]);
      lightTransform.transform.translation.y = SENSOR_DIST_FROM_CENTER * sin(DISTANCE_SENSOR_ANGLE[i]);
      lightTransform.transform.translation.z = 0;
      mLightSensorBroadcasters[i]->sendTransform(lightTransform);
    }

    // Static tf broadcaster: Range (ground sensors)
    for (int i = 0; i < NB_GROUND_SENSORS; i++) {
      mGroundBroadcasters[i] = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
      geometry_msgs::msg::TransformStamped groundTransform;
      groundTransform.header.stamp = now();
      groundTransform.header.frame_id = "base_link";
      groundTransform.child_frame_id = "gs" + std::to_string(i);
      groundTransform.transform.rotation = EPuckDriver::euler2quaternion(0, 0, DISTANCE_SENSOR_ANGLE[i]);
      groundTransform.transform.translation.x = SENSOR_DIST_FROM_CENTER - 0.005;
      groundTransform.transform.translation.y = 0.009 - i * 0.009;
      groundTransform.transform.translation.z = 0;
      mGroundBroadcasters[i]->sendTransform(groundTransform);
    }

    // Static tf broadcaster: Range (ToF)
    mTofBroadcasters = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    geometry_msgs::msg::TransformStamped infraredTransform;
    infraredTransform.header.stamp = now();
    infraredTransform.header.frame_id = "base_link";
    infraredTransform.child_frame_id = "tof";
    infraredTransform.transform.rotation = EPuckDriver::euler2quaternion(0, 0, 0);
    infraredTransform.transform.translation.x = SENSOR_DIST_FROM_CENTER;
    infraredTransform.transform.translation.y = 0;
    infraredTransform.transform.translation.z = 0;
    mTofBroadcasters->sendTransform(infraredTransform);

    RCLCPP_INFO(get_logger(), "EPuck Driver has been initialized");
    RCLCPP_INFO(get_logger(), "Driver mode: %s", type.c_str());
  }

  ~EPuckDriver() { close(mFile); }

private:
  void onLedReceived(const std_msgs::msg::Bool::SharedPtr msg, int index) {
    if (msg->data)
      mMsgActuators[MSG_ACTUATORS_BIN_LEDS_INDEX] |= (1 << index);
    else
      mMsgActuators[MSG_ACTUATORS_BIN_LEDS_INDEX] &= ~(1 << index);
  }
  void onRgbLedReceived(const std_msgs::msg::Int32::SharedPtr msg, int index) {
    mMsgActuators[MSG_ACTUATORS_RGB_LEDS_INDEX + index * 3] = ((msg->data >> 16) & 0xFF) / 2.55;
    mMsgActuators[MSG_ACTUATORS_RGB_LEDS_INDEX + index * 3 + 1] = ((msg->data >> 8) & 0xFF) / 2.55;
    mMsgActuators[MSG_ACTUATORS_RGB_LEDS_INDEX + index * 3 + 2] = (msg->data & 0xFF) / 2.55;
  }

  void resetOdometry() {
    mPrevLeftWheelRaw = 0;
    mPrevRightWheelRaw = 0;
    mOdomLeftOverflow = 0;
    mOdomRightOverflow = 0;
    mPrevPositionX = 0;
    mPrevPositionY = 0;
    mPrevAngle = 0;
  }

  rcl_interfaces::msg::SetParametersResult paramChangeCallback(std::vector<rclcpp::Parameter> parameters) {
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    for (auto parameter : parameters) {
      if (parameter.get_name() == "wheel_distance") {
        resetOdometry();
        mWheelDistance = static_cast<float>(parameter.as_double());
      } else if (parameter.get_name() == "wheel_radius") {
        resetOdometry();
        mWheelRadius = static_cast<float>(parameter.as_double());
      }

      RCLCPP_INFO(get_logger(), "Parameter '%s' has changed to %s", parameter.get_name().c_str(),
                  parameter.value_to_string().c_str());
    }

    return result;
  }

  static geometry_msgs::msg::Quaternion euler2quaternion(double roll, double pitch, double yaw) {
    geometry_msgs::msg::Quaternion quaternion;
    quaternion.x = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    quaternion.y = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
    quaternion.z = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
    quaternion.w = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    return quaternion;
  }

  void onCmdVelReceived(const geometry_msgs::msg::Twist::SharedPtr msg) {
    const double leftVelocity = (2.0 * msg->linear.x - msg->angular.z * mWheelDistance) / (2.0 * mWheelRadius);
    const double rightVelocity = (2.0 * msg->linear.x + msg->angular.z * mWheelDistance) / (2.0 * mWheelRadius);

    const int leftVelocityBig = CLIP(leftVelocity / 0.0068, -1108, 1108);
    const int rightVelocityBig = CLIP(rightVelocity / 0.0068, -1108, 1108);

    RCLCPP_INFO(get_logger(), "New velocity, left %d and right %d", leftVelocityBig, rightVelocityBig);

    mMsgActuators[MSG_ACTUATORS_MOTORS_INDEX + 0] = leftVelocityBig & 0xFF;
    mMsgActuators[MSG_ACTUATORS_MOTORS_INDEX + 1] = (leftVelocityBig >> 8) & 0xFF;
    mMsgActuators[MSG_ACTUATORS_MOTORS_INDEX + 2] = rightVelocityBig & 0xFF;
    mMsgActuators[MSG_ACTUATORS_MOTORS_INDEX + 3] = (rightVelocityBig >> 8) & 0xFF;
  }

  void publishImuData() {
    auto msg = sensor_msgs::msg::Imu();

    mImu->read();
    auto angularVelocity = mImu->getAngularVelocity();
    auto linearAcceleration = mImu->getLinearAcceleration();

    msg.header.stamp = now();
    msg.angular_velocity.x = angularVelocity[1];
    msg.angular_velocity.y = angularVelocity[0];
    msg.angular_velocity.z = angularVelocity[2];
    msg.linear_acceleration.x = linearAcceleration[1];
    msg.linear_acceleration.y = linearAcceleration[0];
    msg.linear_acceleration.z = linearAcceleration[2] + 9.81;

    mImuPublisher->publish(msg);
  }

  static inline float interpolateFunction(float value, float startX, float startY, float endX, float endY) {
    float slope = (endY - startY) / (endX - startX);
    return slope * (value - startX) + startY;
  }

  static float interpolateTable(float value, const std::vector<std::vector<float>> &table) {
    // Search inside two points
    for (unsigned int i = 0; i < table.size() - 1; i++) {
      if ((value < table[i][1] && value >= table[i + 1][1]) || (value > table[i][1] && value <= table[i + 1][1]))
        return EPuckDriver::interpolateFunction(value, table[i][1], table[i][0], table[i + 1][1], table[i + 1][0]);
    }

    // Edge case, search outside of two points.
    // This code assumes that the table is sorted in descending order
    if (value > table[0][1])
      // Interpolate as first
      return EPuckDriver::interpolateFunction(value, table[0][1], table[0][0], table[1][1], table[1][0]);
    else
      // Interpolate as last
      return EPuckDriver::interpolateFunction(value, table[table.size() - 2][1], table[table.size() - 2][0],
                                              table[table.size() - 1][1], table[table.size() - 1][0]);
  }

  void publishGroundSensorData() {
    mI2cMain->setAddress(GROUND_SENSOR_ADDRESS);

    for (int i = 0; i < NB_GROUND_SENSORS; i++) {
      const int16_t raw = (mI2cMain->readInt8Register(2 * i) << 8) | mI2cMain->readInt8Register(2 * i + 1);

      auto msg = sensor_msgs::msg::Range();
      msg.header.stamp = now();
      msg.header.frame_id = "gs" + std::to_string(i);
      msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
      msg.min_range = GROUND_MIN_RANGE;
      msg.max_range = GROUND_MAX_RANGE;
      msg.range = EPuckDriver::interpolateTable((float)raw, GROUND_TABLE);
      msg.field_of_view = 15 * M_PI / 180;
      mGroundRangePublisher[i]->publish(msg);
    }
  }

  void publishIlluminanceData(rclcpp::Time &stamp) {
    for (int i = 0; i < NB_LIGHT_SENSORS; i++) {
      const int16_t raw = (mMsgSensors[MSG_SENSORS_LIGHT_INDEX + i * 2] & 0x00FF) |
                          ((mMsgSensors[MSG_SENSORS_LIGHT_INDEX + 1 + i * 2] << 8) & 0xFF00);

      // Note that here we may need to calibrate the sensors since the expected
      // unit is lux
      auto msg = sensor_msgs::msg::Illuminance();
      msg.header.stamp = stamp;
      msg.illuminance = (double)raw;

      mIlluminancePublisher[i]->publish(msg);
    }
  }

  void publishDistanceData(rclcpp::Time &stamp) {
    // Decode measurements
    static float dist[NB_INFRARED_SENSORS];
    static float laser_dists[NB_INFRARED_SENSORS];
    float distTof = OUT_OF_RANGE;
    for (int i = 0; i < NB_INFRARED_SENSORS; i++) {
      const int distanceIntensity =
        mMsgSensors[MSG_SENSORS_DISTANCE_INDEX + i * 2] + (mMsgSensors[MSG_SENSORS_DISTANCE_INDEX + i * 2 + 1] << 8);
      float distance = EPuckDriver::interpolateTable((float)distanceIntensity, INFRARED_TABLE);
      dist[i] = distance;
    }
    if (mTofInitStatus)
      distTof = tofReadDistance() / 1000.0f;

    // Create LaserScan message
    for (int i = 0; i < NB_INFRARED_SENSORS; i++)
      laser_dists[i] = (dist[i] > INFRARED_MAX_RANGE) ? OUT_OF_RANGE : dist[i];

    auto msg = sensor_msgs::msg::LaserScan();
    msg.header.frame_id = "laser_scanner";
    msg.header.stamp = stamp;
    msg.angle_min = -150 * M_PI / 180;
    msg.angle_max = 150 * M_PI / 180;
    msg.angle_increment = 15 * M_PI / 180.0;
    msg.scan_time = PERIOD_S;
    msg.range_min = INFRARED_MIN_RANGE + SENSOR_DIST_FROM_CENTER;
    msg.range_max = TOF_MAX_RANGE + SENSOR_DIST_FROM_CENTER;
    msg.ranges = std::vector<float>{
      laser_dists[3] + SENSOR_DIST_FROM_CENTER,  // -150
      OUT_OF_RANGE,                              // -135
      OUT_OF_RANGE,                              // -120
      OUT_OF_RANGE,                              // -105
      laser_dists[2] + SENSOR_DIST_FROM_CENTER,  // -90
      OUT_OF_RANGE,                              // -75
      OUT_OF_RANGE,                              // -60
      laser_dists[1] + SENSOR_DIST_FROM_CENTER,  // -45
      OUT_OF_RANGE,                              // -30
      laser_dists[0] + SENSOR_DIST_FROM_CENTER,  // -15
      distTof + SENSOR_DIST_FROM_CENTER,         // 0
      laser_dists[7] + SENSOR_DIST_FROM_CENTER,  // 15
      OUT_OF_RANGE,                              // 30
      laser_dists[6] + SENSOR_DIST_FROM_CENTER,  // 45
      OUT_OF_RANGE,                              // 60
      OUT_OF_RANGE,                              // 75
      laser_dists[5] + SENSOR_DIST_FROM_CENTER,  // 90
      OUT_OF_RANGE,                              // 105
      OUT_OF_RANGE,                              // 120
      OUT_OF_RANGE,                              // 135
      laser_dists[4] + SENSOR_DIST_FROM_CENTER,  // 150
    };
    mLaserPublisher->publish(msg);

    // Create Range messages
    for (int i = 0; i < NB_INFRARED_SENSORS; i++) {
      auto msgRange = sensor_msgs::msg::Range();
      msgRange.header.stamp = stamp;
      msgRange.header.frame_id = "ps" + std::to_string(i);
      msgRange.radiation_type = sensor_msgs::msg::Range::INFRARED;
      msgRange.min_range = INFRARED_MIN_RANGE;
      msgRange.max_range = INFRARED_MAX_RANGE;
      msgRange.range = dist[i];
      msgRange.field_of_view = 15 * M_PI / 180;
      mRangePublisher[i]->publish(msgRange);
    }
    if (mTofInitStatus) {
      auto msgRange = sensor_msgs::msg::Range();
      msgRange.header.stamp = stamp;
      msgRange.header.frame_id = "tof";
      msgRange.radiation_type = sensor_msgs::msg::Range::INFRARED;
      msgRange.min_range = TOF_MIN_RANGE;
      msgRange.max_range = TOF_MAX_RANGE;
      msgRange.range = distTof;
      // Reference: https://forum.pololu.com/t/vl53l0x-beam-width-angle/11483/2
      msgRange.field_of_view = 25 * M_PI / 180;
      mRangeTofPublisher->publish(msgRange);
    }
  }

  void publishOdometryData(rclcpp::Time &stamp) {
    const int16_t leftWheelRaw =
      (mMsgSensors[MSG_SENSORS_ODOMETRY_INDEX] & 0x00FF) | ((mMsgSensors[MSG_SENSORS_ODOMETRY_INDEX + 1] << 8) & 0xFF00);
    const int16_t rightWheelRaw =
      (mMsgSensors[MSG_SENSORS_ODOMETRY_INDEX + 2] & 0x00FF) | ((mMsgSensors[MSG_SENSORS_ODOMETRY_INDEX + 3] << 8) & 0xFF00);
    const float samplePeriodS = (mI2cMainErrCnt + 1) * PERIOD_S;

    // Handle overflow
    // The MCU can handle only 2 bytes of ticks (about 4m), therefore this code allows us to
    // track even when the number of ticks has reached maximum value of 2^15. It is based on
    // detecting the overflow and updating counter `odom_left_overflow`/`odom_right_overflow`.
    const int64_t prevLeftWheelCorrected = mOdomLeftOverflow * POW2(16) + mPrevLeftWheelRaw;
    const int64_t prevRightWheelCorrected = mOdomRightOverflow * POW2(16) + mPrevRightWheelRaw;
    if (abs((int64_t)mPrevLeftWheelRaw - (int64_t)leftWheelRaw) > POW2(15) - ODOM_OVERFLOW_GRACE_TICKS)
      mOdomLeftOverflow = (mPrevLeftWheelRaw > 0 && leftWheelRaw < 0) ? mOdomLeftOverflow + 1 : mOdomLeftOverflow - 1;
    if (abs((int64_t)mPrevRightWheelRaw - (int64_t)rightWheelRaw) > POW2(15) - ODOM_OVERFLOW_GRACE_TICKS)
      mOdomRightOverflow = (mPrevRightWheelRaw > 0 && rightWheelRaw < 0) ? mOdomRightOverflow + 1 : mOdomRightOverflow - 1;
    const int64_t leftWheelCorrected = mOdomLeftOverflow * POW2(16) + leftWheelRaw;
    const int64_t rightWheelCorrected = mOdomRightOverflow * POW2(16) + rightWheelRaw;
    const float leftWheelRad = leftWheelCorrected / (ENCODER_RESOLUTION / (2 * M_PI));
    const float right_wheel_rad = rightWheelCorrected / (ENCODER_RESOLUTION / (2 * M_PI));
    const float prevLeftWheelRad = prevLeftWheelCorrected / (ENCODER_RESOLUTION / (2 * M_PI));
    const float prevRightWheelRad = prevRightWheelCorrected / (ENCODER_RESOLUTION / (2 * M_PI));

    // Calculate velocities
    const float vLeftRad = (leftWheelRad - prevLeftWheelRad) / samplePeriodS;
    const float vRightRad = (right_wheel_rad - prevRightWheelRad) / samplePeriodS;
    const float vLeft = vLeftRad * mWheelRadius;
    const float vRight = vRightRad * mWheelRadius;
    const float v = (vLeft + vRight) / 2;
    const float omega = (vRight - vLeft) / mWheelDistance;

    // Calculate position & angle
    // Fourth order Runge - Kutta
    // Reference: https://www.cs.cmu.edu/~16311/s07/labs/NXTLabs/Lab%203.html
    const float k00 = v * cos(mPrevAngle);
    const float k01 = v * sin(mPrevAngle);
    const float k02 = omega;
    const float k10 = v * cos(mPrevAngle + samplePeriodS * k02 / 2);
    const float k11 = v * sin(mPrevAngle + samplePeriodS * k02 / 2);
    const float k12 = omega;
    const float k20 = v * cos(mPrevAngle + samplePeriodS * k12 / 2);
    const float k21 = v * sin(mPrevAngle + samplePeriodS * k12 / 2);
    const float k22 = omega;
    const float k30 = v * cos(mPrevAngle + samplePeriodS * k22 / 2);
    const float k31 = v * sin(mPrevAngle + samplePeriodS * k22 / 2);
    const float k32 = omega;
    const float positionX = mPrevPositionX + (samplePeriodS / 6) * (k00 + 2 * (k10 + k20) + k30);
    const float positionY = mPrevPositionY + (samplePeriodS / 6) * (k01 + 2 * (k11 + k21) + k31);
    const float angle = mPrevAngle + (samplePeriodS / 6) * (k02 + 2 * (k12 + k22) + k32);

    // Update variables
    mPrevPositionX = positionX;
    mPrevPositionY = positionY;
    mPrevAngle = angle;
    mPrevLeftWheelRaw = leftWheelRaw;
    mPrevRightWheelRaw = rightWheelRaw;

    // Pack & publish odometry
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";
    msg.twist.twist.linear.x = v;
    msg.twist.twist.angular.z = omega;
    msg.pose.pose.position.x = positionX;
    msg.pose.pose.position.y = positionY;
    msg.pose.pose.orientation = euler2quaternion(0, 0, angle);
    mOdometryPublisher->publish(msg);

    // Pack & publish transforms
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = positionX;
    tf.transform.translation.y = positionY;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = euler2quaternion(0, 0, angle);
    mDynamicBroadcaster->sendTransform(tf);
  }

  void updateCallback() {
    int success;
    int retryCount;
    rclcpp::Time stamp;

    // I2C: Set address
    retryCount = 1;
    success = 0;
    while (!success && retryCount > 0) {
      success = mI2cMain->setAddress(0x1F);
      retryCount--;
    }

    // I2C: Write/Read
    retryCount = 3;
    success = 0;
    while (!success && retryCount > 0) {
      // Write
      mMsgActuators[MSG_ACTUATORS_SIZE - 1] = 0;
      for (int i = 0; i < MSG_ACTUATORS_SIZE - 1; i++)
        mMsgActuators[MSG_ACTUATORS_SIZE - 1] ^= mMsgActuators[i];
      success = (mI2cMain->writeData(mMsgActuators, MSG_ACTUATORS_SIZE) == MSG_ACTUATORS_SIZE);

      // Read
      success &= (mI2cMain->readData(mMsgSensors, MSG_SENSORS_SIZE) == MSG_SENSORS_SIZE);
      char checksum = 0;
      for (int i = 0; i < MSG_SENSORS_SIZE - 1; i++)
        checksum ^= mMsgSensors[i];
      success &= (checksum == mMsgSensors[MSG_SENSORS_SIZE - 1]);

      retryCount--;
    }

    stamp = now();

    if (success) {
      publishDistanceData(stamp);
      publishOdometryData(stamp);
      publishIlluminanceData(stamp);
      mI2cMainErrCnt = 0;
    } else
      mI2cMainErrCnt++;

    publishImuData();
    publishGroundSensorData();
  }

  OnSetParametersCallbackHandle::SharedPtr mCallbackHandler;

  rclcpp::TimerBase::SharedPtr mTimer;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr mImuPublisher;
  rclcpp::Publisher<sensor_msgs::msg::Illuminance>::SharedPtr mIlluminancePublisher[NB_LIGHT_SENSORS];
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr mLaserPublisher;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mOdometryPublisher;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr mRangePublisher[NB_INFRARED_SENSORS];
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr mGroundRangePublisher[NB_GROUND_SENSORS];
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr mRangeTofPublisher;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mTwistSubscription;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mRgbLedSubscription[NB_RGB_LEDS];
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mLedSubscription[NB_BINARY_LEDS];

  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> mLaserBroadcaster;
  std::unique_ptr<tf2_ros::TransformBroadcaster> mDynamicBroadcaster;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> mGroundBroadcasters[NB_GROUND_SENSORS];
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> mInfraredBroadcasters[NB_INFRARED_SENSORS];
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> mTofBroadcasters;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> mLightSensorBroadcasters[NB_INFRARED_SENSORS];

  std::shared_ptr<I2CWrapper> mI2cMain;
  std::shared_ptr<MPU9250> mImu;

  int mFile;
  char mMsgActuators[MSG_ACTUATORS_SIZE];
  char mMsgSensors[MSG_SENSORS_SIZE];

  float mPrevAngle;
  float mPrevPositionX;
  float mPrevPositionY;
  int16_t mPrevLeftWheelRaw;
  int16_t mPrevRightWheelRaw;
  int mOdomLeftOverflow;
  int mOdomRightOverflow;
  int mI2cMainErrCnt;

  float mWheelDistance;
  float mWheelRadius;

  int mTofInitStatus;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EPuckDriver>(argc, argv));
  rclcpp::shutdown();
  return 0;
}
