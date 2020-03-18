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

#include "epuck_ros2_cpp/i2c_wrapper.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

#define MSG_ACTUATORS_SIZE 20
#define MSG_SENSORS_SIZE 47
#define PERIOD_MS 64
#define PERIOD_S (PERIOD_MS / 1000.0)
#define OUT_OF_RANGE 0
#define ENCODER_RESOLUTION 1000.0
#define ODOM_OVERFLOW_GRACE_TICKS 2000

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))
#define CLIP(VAL, MIN_VAL, MAX_VAL) MAX(MIN((MAX_VAL), (VAL)), (MIN_VAL))
#define POW2(N) (1 << (N))

const float DEFAULT_WHEEL_DISTANCE = 0.05685;
const float DEFAULT_WHEEL_RADIUS = 0.02;
const float SENSOR_DIST_FROM_CENTER = 0.035;
const std::vector<std::vector<float>> INFRARED_TABLE = {{0, 4095},      {0.005, 2133.33}, {0.01, 1465.73}, {0.015, 601.46},
                                                        {0.02, 383.84}, {0.03, 234.93},   {0.04, 158.03}};
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

class EPuckPublisher : public rclcpp::Node {
public:
  EPuckPublisher(int argc, char *argv[]) : Node("pipuck_driver") {
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
      add_on_set_parameters_callback(std::bind(&EPuckPublisher::paramChangeCallback, this, std::placeholders::_1));

    // Create I2C object
    if (type == "test")
      mI2cMain = std::make_unique<I2CWrapperTest>("/dev/i2c-4");
    else
      mI2cMain = std::make_unique<I2CWrapperHW>("/dev/i2c-4");
    mTofInitialized = tofInit(4, 0x29, 1);
    if (!mTofInitialized) {
      RCLCPP_WARN(get_logger(), "ToF device is not accessible!");
    }

    // Initialize the values
    std::fill(mMsgActuators, mMsgActuators + MSG_ACTUATORS_SIZE, 0);
    std::fill(mMsgSensors, mMsgSensors + MSG_SENSORS_SIZE, 0);
    resetOdometry();
    mI2cMainErrCnt = 0;

    // Create subscirbers and publishers
    mSubscription = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 1, std::bind(&EPuckPublisher::onCmdVelReceived, this, std::placeholders::_1));
    mLaserPublisher = create_publisher<sensor_msgs::msg::LaserScan>("scan", 1);
    mOdometryPublisher = create_publisher<nav_msgs::msg::Odometry>("odom", 1);
    for (int i = 0; i < 8; i++)
      mRangePublisher[i] = create_publisher<sensor_msgs::msg::Range>("ps" + std::to_string(i), 1);
    mRangeTofPublisher = create_publisher<sensor_msgs::msg::Range>("tof", 1);
    mTimer = create_wall_timer(std::chrono::milliseconds(PERIOD_MS), std::bind(&EPuckPublisher::updateCallback, this));

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

    // Static tf broadcaster: Range (infrared)
    for (int i = 0; i < 8; i++) {
      mInfraredBroadcasters[i] = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
      geometry_msgs::msg::TransformStamped infraredTransform;
      infraredTransform.header.stamp = now();
      infraredTransform.header.frame_id = "base_link";
      infraredTransform.child_frame_id = "ps" + std::to_string(i);
      infraredTransform.transform.rotation = EPuckPublisher::euler2quaternion(0, 0, DISTANCE_SENSOR_ANGLE[i]);
      infraredTransform.transform.translation.x = SENSOR_DIST_FROM_CENTER;
      infraredTransform.transform.translation.y = 0;
      infraredTransform.transform.translation.z = 0;
      mInfraredBroadcasters[i]->sendTransform(infraredTransform);
    }

    // Static tf broadcaster: Range (ToF)
    mInfraredBroadcasters[8] = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    geometry_msgs::msg::TransformStamped infraredTransform;
    infraredTransform.header.stamp = now();
    infraredTransform.header.frame_id = "base_link";
    infraredTransform.child_frame_id = "tof";
    infraredTransform.transform.rotation = EPuckPublisher::euler2quaternion(0, 0, 0);
    infraredTransform.transform.translation.x = SENSOR_DIST_FROM_CENTER;
    infraredTransform.transform.translation.y = 0;
    infraredTransform.transform.translation.z = 0;
    mInfraredBroadcasters[8]->sendTransform(infraredTransform);

    RCLCPP_INFO(get_logger(), "EPuck Driver has been initialized");
    RCLCPP_INFO(get_logger(), "Driver mode: %s", type.c_str());
  }

  ~EPuckPublisher() { close(mFile); }

private:
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

  static float intensity2distance(int pX) {
    for (unsigned int i = 0; i < INFRARED_TABLE.size() - 1; i++) {
      if (INFRARED_TABLE[i][1] >= pX && INFRARED_TABLE[i + 1][1] < pX) {
        const float bX = INFRARED_TABLE[i][1];
        const float bY = INFRARED_TABLE[i][0];
        const float aX = INFRARED_TABLE[i + 1][1];
        const float aY = INFRARED_TABLE[i + 1][0];
        const float pY = ((bY - aY) / (bX - aX)) * (pX - aX) + aY;
        return pY;
      }
    }
    return OUT_OF_RANGE;
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

    mMsgActuators[0] = leftVelocityBig & 0xFF;
    mMsgActuators[1] = (leftVelocityBig >> 8) & 0xFF;
    mMsgActuators[2] = rightVelocityBig & 0xFF;
    mMsgActuators[3] = (rightVelocityBig >> 8) & 0xFF;
  }

  void publishDistanceData(rclcpp::Time &stamp) {
    // Decode measurements
    float dist[8];
    float distTof = OUT_OF_RANGE;
    for (int i = 0; i < 8; i++) {
      const int distanceIntensity = mMsgSensors[i * 2] + (mMsgSensors[i * 2 + 1] << 8);
      float distance = EPuckPublisher::intensity2distance(distanceIntensity);
      dist[i] = distance;
    }
    if (mTofInitialized) {
      distTof = tofReadDistance() / 1000.0;
    }

    // Create LaserScan message
    auto msg = sensor_msgs::msg::LaserScan();
    msg.header.frame_id = "laser_scanner";
    msg.header.stamp = stamp;
    msg.angle_min = -150 * M_PI / 180;
    msg.angle_max = 150 * M_PI / 180;
    msg.angle_increment = 15 * M_PI / 180.0;
    msg.scan_time = PERIOD_S;
    msg.range_min = 0.005 + SENSOR_DIST_FROM_CENTER;
    msg.range_max = 0.05 + SENSOR_DIST_FROM_CENTER;
    msg.ranges = std::vector<float>{
      dist[3] + SENSOR_DIST_FROM_CENTER,  // -150
      OUT_OF_RANGE,                       // -135
      OUT_OF_RANGE,                       // -120
      OUT_OF_RANGE,                       // -105
      dist[2] + SENSOR_DIST_FROM_CENTER,  // -90
      OUT_OF_RANGE,                       // -75
      OUT_OF_RANGE,                       // -60
      dist[1] + SENSOR_DIST_FROM_CENTER,  // -45
      OUT_OF_RANGE,                       // -30
      dist[0] + SENSOR_DIST_FROM_CENTER,  // -15
      distTof + SENSOR_DIST_FROM_CENTER,  // 0
      dist[7] + SENSOR_DIST_FROM_CENTER,  // 15
      OUT_OF_RANGE,                       // 30
      dist[6] + SENSOR_DIST_FROM_CENTER,  // 45
      OUT_OF_RANGE,                       // 60
      OUT_OF_RANGE,                       // 75
      dist[5] + SENSOR_DIST_FROM_CENTER,  // 90
      OUT_OF_RANGE,                       // 105
      OUT_OF_RANGE,                       // 120
      OUT_OF_RANGE,                       // 135
      dist[4] + SENSOR_DIST_FROM_CENTER,  // 150
    };
    mLaserPublisher->publish(msg);

    // Create Range messages
    for (int i = 0; i < 8; i++) {
      auto msgRange = sensor_msgs::msg::Range();
      msgRange.header.stamp = stamp;
      msgRange.header.frame_id = "ps" + std::to_string(i);
      msgRange.radiation_type = sensor_msgs::msg::Range::INFRARED;
      msgRange.min_range = 0.005;
      msgRange.max_range = 0.05;
      msgRange.range = dist[i];
      msgRange.field_of_view = 15 * M_PI / 180;
      mRangePublisher[i]->publish(msgRange);
    }
    if (mTofInitialized) {
      auto msgRange = sensor_msgs::msg::Range();
      msgRange.header.stamp = stamp;
      msgRange.header.frame_id = "tof";
      msgRange.radiation_type = sensor_msgs::msg::Range::INFRARED;
      msgRange.min_range = 0.005;
      msgRange.min_range = 2.0;
      msgRange.range = distTof;
      // Reference: https://forum.pololu.com/t/vl53l0x-beam-width-angle/11483/2
      msgRange.field_of_view = 25 * M_PI / 180;
      mRangeTofPublisher->publish(msgRange);
    }
  }

  void publishOdometryData(rclcpp::Time &stamp) {
    const int16_t leftWheelRaw = (mMsgSensors[41] & 0x00FF) | ((mMsgSensors[42] << 8) & 0xFF00);
    const int16_t rightWheelRaw = (mMsgSensors[43] & 0x00FF) | ((mMsgSensors[44] << 8) & 0xFF00);
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
    msg.twist.twist.linear.z = omega;
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
      mI2cMainErrCnt = 0;
    } else {
      mI2cMainErrCnt++;
    }
  }

  OnSetParametersCallbackHandle::SharedPtr mCallbackHandler;

  rclcpp::TimerBase::SharedPtr mTimer;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr mLaserPublisher;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mOdometryPublisher;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr mRangePublisher[8];
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr mRangeTofPublisher;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mSubscription;

  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> mLaserBroadcaster;
  std::unique_ptr<tf2_ros::TransformBroadcaster> mDynamicBroadcaster;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> mInfraredBroadcasters[9];

  std::unique_ptr<I2CWrapper> mI2cMain;

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

  int mTofInitialized;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EPuckPublisher>(argc, argv));
  rclcpp::shutdown();
  return 0;
}
