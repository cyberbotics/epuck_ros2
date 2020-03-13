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

extern "C" {
#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <unistd.h>
}

#include <chrono>
#include <memory>
#include <algorithm>
#include <vector>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "epuck_ros2_cpp/i2c_wrapper.hpp"

#define MSG_ACTUATORS_SIZE 20
#define MSG_SENSORS_SIZE 47
#define PERIOD_MS 64

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))
#define CLIP(VAL, MIN_VAL, MAX_VAL) MAX(MIN((MAX_VAL), (VAL)), (MIN_VAL))

const double WHEEL_DISTANCE = 0.05685;
const double WHEEL_RADIUS = 0.02;
const float SENSOR_DIST_FROM_CENTER = 0.035;
const std::vector<std::vector<float>> INFRARED_TABLE = {
  {0, 4095}, {0.005, 2133.33}, {0.01, 1465.73}, {0.015, 601.46},
  {0.02, 383.84}, {0.03, 234.93}, {0.04, 158.03}, {0.05, 120},
  {0.06, 104.09}, {0.07, 67.19}, {0.1, 0.0}};

class EPuckPublisher : public rclcpp::Node
{
public:
  EPuckPublisher(int argc, char * argv[])
  : Node("pipuck_driver")
  {
    // Parse arguments
    std::string type = "hw";
    for (int i = 1; i < argc; i++) {
      if (strcmp(argv[i], "--type") == 0) {
        i++;
        type = argv[i];
      }
    }

    // Create I2C object
    if (type == "test") {
      i2c_main = std::make_unique<I2CWrapperTest>("/dev/i2c-4");
    } else {
      i2c_main = std::make_unique<I2CWrapperHW>("/dev/i2c-4");
    }

    // Initialize the buffers
    std::fill(msg_actuators, msg_actuators + MSG_ACTUATORS_SIZE, 0);
    std::fill(msg_sensors, msg_sensors + MSG_SENSORS_SIZE, 0);

    // Create subscirbers and publishers
    subscription = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 1,
      std::bind(&EPuckPublisher::on_cmd_vel_received, this,
      std::placeholders::_1));
    laser_publisher =
      this->create_publisher<sensor_msgs::msg::LaserScan>("laser", 1);
    for (int i = 0; i < 8; i++) {
      range_publisher[i] = this->create_publisher<sensor_msgs::msg::Range>(
        "ps" + std::to_string(i), 1);
    }
    timer = this->create_wall_timer(
      std::chrono::milliseconds(PERIOD_MS),
      std::bind(&EPuckPublisher::update_callback, this));

    RCLCPP_INFO(this->get_logger(), "EPuck Driver has been initialized");
    RCLCPP_INFO(this->get_logger(), "Driver mode: %s", type.c_str());
  }

  ~EPuckPublisher() {close(fh);}

private:
  static float intensity_to_distance(int p_x)
  {
    for (unsigned int i = 0; i < INFRARED_TABLE.size() - 1; i++) {
      if (INFRARED_TABLE[i][1] >= p_x && INFRARED_TABLE[i + 1][1] < p_x) {
        const float b_x = INFRARED_TABLE[i][1];
        const float b_y = INFRARED_TABLE[i][0];
        const float a_x = INFRARED_TABLE[i + 1][1];
        const float a_y = INFRARED_TABLE[i + 1][0];
        const float p_y = ((b_y - a_y) / (b_x - a_x)) * (p_x - a_x) + a_y;
        return p_y;
      }
    }
    return 100.0;
  }

  static geometry_msgs::msg::Quaternion::SharedPtr
  euler_to_quaternion(double roll, double pitch, double yaw)
  {
    geometry_msgs::msg::Quaternion::SharedPtr q;
    q->x = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) -
      cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    q->y = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) +
      sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
    q->z = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) -
      sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
    q->w = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) +
      sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    return q;
  }

  void on_cmd_vel_received(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const double left_velocity =
      (2.0 * msg->linear.x - msg->angular.z * WHEEL_DISTANCE) /
      (2.0 * WHEEL_RADIUS);
    const double right_velocity =
      (2.0 * msg->linear.x + msg->angular.z * WHEEL_DISTANCE) /
      (2.0 * WHEEL_RADIUS);

    const int left_velocity_big = CLIP(left_velocity / 0.0068, -1108, 1108);
    const int right_velocity_big = CLIP(right_velocity / 0.0068, -1108, 1108);

    RCLCPP_INFO(this->get_logger(), "New velocity, left %d and right %d",
      left_velocity_big, right_velocity_big);

    msg_actuators[0] = left_velocity_big & 0xFF;
    msg_actuators[1] = (left_velocity_big >> 8) & 0xFF;
    msg_actuators[2] = right_velocity_big & 0xFF;
    msg_actuators[3] = (right_velocity_big >> 8) & 0xFF;
  }

  void publish_distance_data(rclcpp::Time & stamp)
  {
    // Decode measurements
    float dist[8];
    for (int i = 0; i < 8; i++) {
      const int distance_intensity =
        msg_sensors[i * 2] + (msg_sensors[i * 2 + 1] << 8);
      float distance =
        EPuckPublisher::intensity_to_distance(distance_intensity) +
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
      dist[4],                                         // -150
      (3 / 4) * dist[4] + (1 / 4) * dist[5],           // -135
      (2 / 4) * dist[4] + (2 / 4) * dist[5],           // -120
      (1 / 4) * dist[4] + (3 / 4) * dist[5],           // -105
      dist[5],                                         // -90
      (2 / 3) * dist[5] + (1 / 3) * dist[6],           // -75
      (1 / 3) * dist[5] + (2 / 3) * dist[6],           // -60
      dist[6],                                         // -45
      (1 / 2) * dist[6] + (1 / 2) * dist[7],           // -30
      dist[7],                                         // -15
      (1 / 2) * dist[7] + (1 / 2) * dist[0],           // dist['tof'], // 0
      dist[0],                                         // 15
      (1 / 2) * dist[0] + (1 / 2) * dist[1],           // 30
      dist[1],                                         // 45
      (2 / 3) * dist[1] + (1 / 3) * dist[2],           // 60
      (1 / 3) * dist[1] + (2 / 3) * dist[2],           // 75
      dist[2],                                         // 90
      (3 / 4) * dist[2] + (1 / 4) * dist[3],           // 105
      (2 / 4) * dist[2] + (2 / 4) * dist[3],           // 120
      (1 / 4) * dist[2] + (3 / 4) * dist[3],           // 135
      dist[3],                                         // 150
    };
    laser_publisher->publish(msg);

    // Create Range messages
    for (int i = 0; i < 8; i++) {
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

  void update_callback()
  {
    int status;
    rclcpp::Time stamp;

    // Main MCU
    status = i2c_main->set_address(0x1F);
    assert(status >= 0);

    // Main MCU: Write
    msg_actuators[MSG_ACTUATORS_SIZE - 1] = 0;
    for (int i = 0; i < MSG_ACTUATORS_SIZE - 1; i++) {
      msg_actuators[MSG_ACTUATORS_SIZE - 1] ^= msg_actuators[i];
    }

    i2c_main->write_data(msg_actuators, MSG_ACTUATORS_SIZE);
    i2c_main->read_data(msg_sensors, MSG_SENSORS_SIZE);

    stamp = now();
    publish_distance_data(stamp);
  }

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_publisher[9];
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription;

  std::unique_ptr<I2CWrapper> i2c_main;

  int fh;
  char msg_actuators[MSG_ACTUATORS_SIZE];
  char msg_sensors[MSG_SENSORS_SIZE];
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EPuckPublisher>(argc, argv));
  rclcpp::shutdown();
  return 0;
}
