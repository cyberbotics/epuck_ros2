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

extern "C"
{
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
#include <memory>
#include <string>
#include <vector>
#include <map>

#include "epuck_ros2_cpp/i2c_wrapper.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"

#define MSG_ACTUATORS_SIZE 20
#define MSG_SENSORS_SIZE 47
#define PERIOD_MS 64
#define PERIOD_S (PERIOD_MS / 1000.0)
#define OUT_OF_RANGE 0
#define ENCODER_RESOLUTION 1000.0

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))
#define CLIP(VAL, MIN_VAL, MAX_VAL) MAX(MIN((MAX_VAL), (VAL)), (MIN_VAL))

const float DEFAULT_WHEEL_DISTANCE = 0.05685;
const float DEFAULT_WHEEL_RADIUS = 0.02;
const float SENSOR_DIST_FROM_CENTER = 0.035;
const std::vector<std::vector<float>> INFRARED_TABLE =
    {{0, 4095}, {0.005, 2133.33}, {0.01, 1465.73}, {0.015, 601.46}, {0.02, 383.84}, {0.03, 234.93}, {0.04, 158.03}, {0.05, 120}, {0.06, 104.09}, {0.07, 67.19}, {0.1, 0.0}};
const std::vector<double> DISTANCE_SENSOR_ANGLE = {
    -15 * M_PI / 180,  // ps0
    -45 * M_PI / 180,  // ps1
    -90 * M_PI / 180,  // ps2
    -150 * M_PI / 180, // ps3
    150 * M_PI / 180,  // ps4
    90 * M_PI / 180,   // ps5
    45 * M_PI / 180,   // ps6
    15 * M_PI / 180,   // ps7
    0 * M_PI / 180     // tof
};

class EPuckPublisher : public rclcpp::Node
{
public:
  EPuckPublisher(int argc, char *argv[])
      : Node("pipuck_driver")
  {
    // Parse arguments
    std::string type = "hw";
    for (int i = 1; i < argc; i++)
    {
      if (strcmp(argv[i], "--type") == 0)
      {
        i++;
        type = argv[i];
      }
    }

    wheel_distance = declare_parameter<float>("wheel_distance", DEFAULT_WHEEL_DISTANCE);
    wheel_radius = declare_parameter<float>("wheel_radius", DEFAULT_WHEEL_RADIUS);
    /*
    callback_handler =
      this->add_on_set_parameters_callback(std::bind(&EPuckPublisher::param_change_callback, this,
        std::placeholders::_1));
    */

    // Create I2C object
    if (type == "test")
    {
      i2c_main = std::make_unique<I2CWrapperTest>("/dev/i2c-4");
    }
    else
    {
      i2c_main = std::make_unique<I2CWrapperHW>("/dev/i2c-4");
    }

    // Initialize the values
    std::fill(msg_actuators, msg_actuators + MSG_ACTUATORS_SIZE, 0);
    std::fill(msg_sensors, msg_sensors + MSG_SENSORS_SIZE, 0);
    reset_odometry();

    // Create subscirbers and publishers
    subscription = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 1, std::bind(&EPuckPublisher::on_cmd_vel_received, this, std::placeholders::_1));
    laser_publisher = this->create_publisher<sensor_msgs::msg::LaserScan>("laser", 1);
    odometry_publisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
    for (int i = 0; i < 8; i++)
    {
      range_publisher[i] = this->create_publisher<sensor_msgs::msg::Range>("ps" + std::to_string(
                                                                                      i),
                                                                           1);
    }
    timer =
        this->create_wall_timer(std::chrono::milliseconds(PERIOD_MS),
                                std::bind(&EPuckPublisher::update_callback, this));

    // Dynamic tf broadcaster: Odometry
    dynamic_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // Static tf broadcaster: Laser
    laser_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    geometry_msgs::msg::TransformStamped laser_transform;
    laser_transform.header.stamp = this->now();
    laser_transform.header.frame_id = "base_link";
    laser_transform.child_frame_id = "laser_scanner";
    laser_transform.transform.rotation.x = 0;
    laser_transform.transform.rotation.y = 0;
    laser_transform.transform.rotation.z = 0;
    laser_transform.transform.rotation.w = 1;
    laser_transform.transform.translation.x = 0;
    laser_transform.transform.translation.y = 0;
    laser_transform.transform.translation.z = 0;
    laser_broadcaster->sendTransform(laser_transform);

    // Static tf broadcaster: Range (infrared)
    for (int i = 0; i < 8; i++)
    {
      infrared_broadcasters[i] = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
      geometry_msgs::msg::TransformStamped infrared_transform;
      infrared_transform.header.stamp = this->now();
      infrared_transform.header.frame_id = "base_link";
      infrared_transform.child_frame_id = "ps" + std::to_string(i);
      infrared_transform.transform.rotation = EPuckPublisher::euler_to_quaternion(0, 0, DISTANCE_SENSOR_ANGLE[i]);
      infrared_transform.transform.translation.x = 0;
      infrared_transform.transform.translation.y = 0;
      infrared_transform.transform.translation.z = 0;
      infrared_broadcasters[i]->sendTransform(infrared_transform);
    }

    // Static tf broadcaster: Range (ToF)
    infrared_broadcasters[8] = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    geometry_msgs::msg::TransformStamped infrared_transform;
    infrared_transform.header.stamp = this->now();
    infrared_transform.header.frame_id = "base_link";
    infrared_transform.child_frame_id = "tof";
    infrared_transform.transform.rotation = EPuckPublisher::euler_to_quaternion(0, 0, 0);
    infrared_transform.transform.translation.x = 0;
    infrared_transform.transform.translation.y = 0;
    infrared_transform.transform.translation.z = 0;
    infrared_broadcasters[8]->sendTransform(infrared_transform);

    RCLCPP_INFO(this->get_logger(), "EPuck Driver has been initialized");
    RCLCPP_INFO(this->get_logger(), "Driver mode: %s", type.c_str());
  }

  ~EPuckPublisher() { close(fh); }

private:
  void reset_odometry() {
    prev_left_wheel_ticks = 0;
    prev_right_wheel_ticks = 0;
    prev_position_x = 0;
    prev_position_y = 0;
    prev_angle = 0;
  }

  rcl_interfaces::msg::SetParametersResult param_change_callback(
    std::vector<rclcpp::Parameter> parameters)
  {
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    for (auto parameter : parameters) {
      if (parameter.get_name() == "wheel_distance") {
        reset_odometry();
        wheel_distance = (float)parameter.as_double();
      }
      else if (parameter.get_name() == "wheel_radius") {
        reset_odometry();
        wheel_radius = (float)parameter.as_double();
      }

      RCLCPP_INFO(this->get_logger(), "Parameter '%s' has changed to %s",
        parameter.get_name().c_str(),
        parameter.value_to_string().c_str());
    }

    return result;
  }

  static float intensity_to_distance(int p_x)
  {
    for (unsigned int i = 0; i < INFRARED_TABLE.size() - 1; i++)
    {
      if (INFRARED_TABLE[i][1] >= p_x && INFRARED_TABLE[i + 1][1] < p_x)
      {
        const float b_x = INFRARED_TABLE[i][1];
        const float b_y = INFRARED_TABLE[i][0];
        const float a_x = INFRARED_TABLE[i + 1][1];
        const float a_y = INFRARED_TABLE[i + 1][0];
        const float p_y = ((b_y - a_y) / (b_x - a_x)) * (p_x - a_x) + a_y;
        return p_y;
      }
    }
    return OUT_OF_RANGE;
  }

  static geometry_msgs::msg::Quaternion euler_to_quaternion(
      double roll, double pitch,
      double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    q.x = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    q.y = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
    q.z = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
    q.w = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    return q;
  }

  void on_cmd_vel_received(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const double left_velocity = (2.0 * msg->linear.x - msg->angular.z * wheel_distance) /
                                 (2.0 * wheel_radius);
    const double right_velocity = (2.0 * msg->linear.x + msg->angular.z * wheel_distance) /
                                  (2.0 * wheel_radius);

    const int left_velocity_big = CLIP(left_velocity / 0.0068, -1108, 1108);
    const int right_velocity_big = CLIP(right_velocity / 0.0068, -1108, 1108);

    RCLCPP_INFO(
        this->get_logger(), "New velocity, left %d and right %d", left_velocity_big,
        right_velocity_big);

    msg_actuators[0] = left_velocity_big & 0xFF;
    msg_actuators[1] = (left_velocity_big >> 8) & 0xFF;
    msg_actuators[2] = right_velocity_big & 0xFF;
    msg_actuators[3] = (right_velocity_big >> 8) & 0xFF;
  }

  void publish_distance_data(rclcpp::Time &stamp)
  {
    // Decode measurements
    float dist[8];
    for (int i = 0; i < 8; i++)
    {
      const int distance_intensity = msg_sensors[i * 2] + (msg_sensors[i * 2 + 1] << 8);
      float distance = EPuckPublisher::intensity_to_distance(distance_intensity) +
                       SENSOR_DIST_FROM_CENTER;
      dist[i] = distance;
    }

    // Create LaserScan message
    auto msg = sensor_msgs::msg::LaserScan();
    msg.header.frame_id = "laser_scanner";
    msg.header.stamp = stamp;
    msg.angle_min = -150 * M_PI / 180;
    msg.angle_max = 150 * M_PI / 180;
    msg.angle_increment = 15 * M_PI / 180.0;
    msg.scan_time = PERIOD_MS / 1000;
    msg.range_min = 0.005 + SENSOR_DIST_FROM_CENTER;
    msg.range_max = 0.05 + SENSOR_DIST_FROM_CENTER;
    msg.ranges = std::vector<float>{
        dist[3],      // -150
        OUT_OF_RANGE, // -135
        OUT_OF_RANGE, // -120
        OUT_OF_RANGE, // -105
        dist[2],      // -90
        OUT_OF_RANGE, // -75
        OUT_OF_RANGE, // -60
        dist[1],      // -45
        OUT_OF_RANGE, // -30
        dist[0],      // -15
        OUT_OF_RANGE, // dist['tof'], // 0
        dist[7],      // 15
        OUT_OF_RANGE, // 30
        dist[6],      // 45
        OUT_OF_RANGE, // 60
        OUT_OF_RANGE, // 75
        dist[5],      // 90
        OUT_OF_RANGE, // 105
        OUT_OF_RANGE, // 120
        OUT_OF_RANGE, // 135
        dist[4],      // 150
    };
    laser_publisher->publish(msg);

    // Create Range messages
    for (int i = 0; i < 8; i++)
    {
      auto msg_range = sensor_msgs::msg::Range();
      msg_range.header.stamp = stamp;
      msg_range.header.frame_id = "ps" + std::to_string(i);
      msg_range.radiation_type = sensor_msgs::msg::Range::INFRARED;
      msg_range.min_range = 0.05 + SENSOR_DIST_FROM_CENTER;
      msg_range.min_range = 0.005 + SENSOR_DIST_FROM_CENTER;
      msg_range.range = dist[i];
      range_publisher[i]->publish(msg_range);
    }
  }

  void publish_odometry_data(rclcpp::Time &stamp)
  {
    const int16_t left_wheel_raw = (msg_sensors[41] & 0x00FF) | ((msg_sensors[42] << 8) & 0xFF00);
    const int16_t right_wheel_raw = (msg_sensors[43] & 0x00FF) | ((msg_sensors[44] << 8) & 0xFF00);
    const float left_wheel_ticks = left_wheel_raw / (ENCODER_RESOLUTION / (2 * M_PI));
    const float right_wheel_ticks = right_wheel_raw / (ENCODER_RESOLUTION / (2 * M_PI));

    // Calculate velocities
    const float v_left_rad = (left_wheel_ticks - this->prev_left_wheel_ticks) / PERIOD_S;
    const float v_right_rad = (right_wheel_ticks - this->prev_right_wheel_ticks) / PERIOD_S;
    const float v_left = v_left_rad * this->wheel_radius;
    const float v_right = v_right_rad * this->wheel_radius;
    const float v = (v_left + v_right) / 2;
    const float omega = (v_right - v_left) / this->wheel_distance;

    // Calculate position & angle
    // Fourth order Runge - Kutta
    // Reference: https://www.cs.cmu.edu/~16311/s07/labs/NXTLabs/Lab%203.html
    const float k00 = v * cos(this->prev_angle);
    const float k01 = v * sin(this->prev_angle);
    const float k02 = omega;
    const float k10 = v * cos(this->prev_angle + PERIOD_S * k02 / 2);
    const float k11 = v * sin(this->prev_angle + PERIOD_S * k02 / 2);
    const float k12 = omega;
    const float k20 = v * cos(this->prev_angle + PERIOD_S * k12 / 2);
    const float k21 = v * sin(this->prev_angle + PERIOD_S * k12 / 2);
    const float k22 = omega;
    const float k30 = v * cos(this->prev_angle + PERIOD_S * k22 / 2);
    const float k31 = v * sin(this->prev_angle + PERIOD_S * k22 / 2);
    const float k32 = omega;
    const float position_x = this->prev_position_x + (PERIOD_S / 6) *
                                                         (k00 + 2 * (k10 + k20) + k30);
    const float position_y = this->prev_position_y + (PERIOD_S / 6) *
                                                         (k01 + 2 * (k11 + k21) + k31);
    const float angle = this->prev_angle + (PERIOD_S / 6) * (k02 + 2 * (k12 + k22) + k32);

    // Update variables
    this->prev_position_x = position_x;
    this->prev_position_y = position_y;
    this->prev_angle = angle;
    this->prev_left_wheel_ticks = left_wheel_ticks;
    this->prev_right_wheel_ticks = right_wheel_ticks;

    // Pack & publish odometry
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";
    msg.twist.twist.linear.x = v;
    msg.twist.twist.linear.z = omega;
    msg.pose.pose.position.x = position_x;
    msg.pose.pose.position.y = position_y;
    msg.pose.pose.orientation = euler_to_quaternion(0, 0, angle);
    odometry_publisher->publish(msg);

    // Pack & publish transforms
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = position_x;
    tf.transform.translation.y = position_y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = euler_to_quaternion(0, 0, angle);
    dynamic_broadcaster->sendTransform(tf);
  }

  void update_callback()
  {
    int status;
    rclcpp::Time stamp;

    // Main MCU
    status = i2c_main->set_address(0x1F);
    assert(status >= 0);

    // Main MCU: Write
    msg_actuators[MSG_ACTUATORS_SIZE - 1] = 0;
    for (int i = 0; i < MSG_ACTUATORS_SIZE - 1; i++)
    {
      msg_actuators[MSG_ACTUATORS_SIZE - 1] ^= msg_actuators[i];
    }

    i2c_main->write_data(msg_actuators, MSG_ACTUATORS_SIZE);
    i2c_main->read_data(msg_sensors, MSG_SENSORS_SIZE);

    stamp = now();
    publish_distance_data(stamp);
    publish_odometry_data(stamp);
  }

  OnSetParametersCallbackHandle::SharedPtr callback_handler;

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_publisher;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_publisher[9];
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription;

  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> laser_broadcaster;
  std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_broadcaster;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> infrared_broadcasters[9];

  std::unique_ptr<I2CWrapper> i2c_main;

  int fh;
  char msg_actuators[MSG_ACTUATORS_SIZE];
  char msg_sensors[MSG_SENSORS_SIZE];

  float prev_left_wheel_ticks;
  float prev_right_wheel_ticks;
  float prev_angle;
  float prev_position_x;
  float prev_position_y;

  float wheel_distance;
  float wheel_radius;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EPuckPublisher>(argc, argv));
  rclcpp::shutdown();
  return 0;
}
