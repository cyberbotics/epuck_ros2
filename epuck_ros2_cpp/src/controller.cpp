#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

extern "C"
{
#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/i2c-dev.h> /* for I2C_SLAVE */
//#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <unistd.h>
}

#define MSG_ACTUATORS_SIZE 20
#define MSG_SENSORS_SIZE 47
#define PERIOD_MS 64

#define min(X, Y) (((X) < (Y)) ? (X) : (Y))
#define max(X, Y) (((X) > (Y)) ? (X) : (Y))
#define CLIP(VAL, MIN, MAX) max(min((MAX), (VAL)), (MIN))

using namespace std::chrono_literals;

const double WHEEL_DISTANCE = 0.05685;
const double WHEEL_RADIUS = 0.02;

class EPuckPublisher : public rclcpp::Node
{
public:
  EPuckPublisher()
      : Node("pipuck_driver")
  {
    fh = open("/dev/i2c-4", O_RDWR);
    assert(fh != 0);
    memset(msg_actuators, 0, MSG_ACTUATORS_SIZE);
    memset(msg_sensors, 0, MSG_SENSORS_SIZE);

    subscription = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&EPuckPublisher::on_cmd_vel_received, this, std::placeholders::_1));
    laser_publisher = this->create_publisher<geometry_msgs::msg::Twist>("laser", 1);
    
    timer = this->create_wall_timer(
        chrono::millisecods(PERIOD_MS), std::bind(&EPuckPublisher::update_callback, this));

    RCLCPP_INFO(this->get_logger(), "EPuck Driver has been initialized");
  }

  ~EPuckPublisher()
  {
    close(fh);
  }

private:
  static double intensity_to_distance(double p_x) {
    std::vector<std::vector<int>> vect = { 
      {0, 4095},
      {0.005, 2133.33},
      {0.01, 1465.73},
      {0.015, 601.46},
      {0.02, 383.84},
      {0.03, 234.93},
      {0.04, 158.03},
      {0.05, 120},
      {0.06, 104.09},
      {0.07, 67.19},
      {0.1, 0.0}
    };
    for (int i = 0; i < table.size() - 1; i++) {
        if (table[i][1] >= p_x and table[i+1][1] < p_x) {
            b_x = table[i][1];
            b_y = table[i][0];
            a_x = table[i+1][1];
            a_y = table[i+1][0];
            p_y = ((b_y - a_y) / (b_x - a_x)) * (p_x - a_x) + a_y;
            return p_y;
        }
    }
    return 0.1
    }

  static geometry_msgs::msg::Quaternion::SharedPtr euler_to_quaternion(double roll, double pitch, double yaw) {
    geometry_msgs::msg::Quaternion::SharedPtr q;
    q->x = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
    q->y = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
    q->z = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
    q->w = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);
    return q
  }

  void on_cmd_vel_received(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double left_velocity = (2.0 * msg->linear.x - msg->angular.z * WHEEL_DISTANCE) / (2.0 * WHEEL_RADIUS);
    double right_velocity = (2.0 * msg->linear.x + msg->angular.z * WHEEL_DISTANCE) / (2.0 * WHEEL_RADIUS);

    int left_velocity_big = CLIP(left_velocity / 0.0068, -1108, 1108);
    int right_velocity_big = CLIP(right_velocity / 0.0068, -1108, 1108);

    RCLCPP_INFO(this->get_logger(), "New velocity, left %d and right %d", left_velocity_big, left_velocity_big);

    msg_actuators[0] = left_velocity_big & 0xFF;
    msg_actuators[1] = (left_velocity_big >> 8) & 0xFF;
    msg_actuators[2] = right_velocity_big & 0xFF;
    msg_actuators[3] = (right_velocity_big >> 8) & 0xFF;
  }

  void publish_distance_data() {
    auto msg = sensor_msgs::msg::LaserScan();
    msg.header.frame_id = "laser_scanner";
    msg.angle_min = 0;
    msg.angle_max = 2 * pi;
    msg.angle_increment = 15 * pi / 180.0;
    msg.scan_time = PERIOD_MS / 1000;
    msg.range_min = 0.01;
    msg.range_max = 0.1;
    
    double encoded_distances[8];
    for (int i = 0; i < 8; i++) {
      int distance_intensity = msg_sensors[i * 2] + (msg_sensors[i * 2 + 1] << 8);
      double distance = EPuckPublisher::intensity_to_distance(distance_intensity);
      encoded_distances[i] = distance;
    }

    msg.ranges = std::vector<double>{
      0.0, // intensity_to_distance(self.sensors['tof'].getValue()),  # 0
      intensity_to_distance(encoded_distances[7]),  // 15
      0.0,                            // 30
      intensity_to_distance(encoded_distances[6]),  // 45
      0.0,                            // 60
      0.0,                            // 75
      intensity_to_distance(encoded_distances[5]),  // 90
      0.0,                            // 105
      0.0,                            // 120
      0.0,                            // 135
      intensity_to_distance(encoded_distances[4]),  // 150
      0.0,                            // 165
      0.0,                            // 180
      0.0,                            // 195
      intensity_to_distance(encoded_distances[3]), // 210
      0.0,                            // 225
      0.0,                            // 240
      0.0,                            // 255
      intensity_to_distance(encoded_distances[2]),  // 270
      0.0,                            // 285
      0.0,                            // 300
      intensity_to_distance(encoded_distances[1]),  // 315
      0.0,                            // 330
      intensity_to_distance(encoded_distances[0]),  // 345
    }
  }

  void update_callback()
  {
    int status;

    status = ioctl(fh, I2C_SLAVE, 0x1F);
    assert(status >= 0);

    for (int i = 0; i < MSG_ACTUATORS_SIZE - 1; i++)
    {
      msg_actuators[MSG_ACTUATORS_SIZE - 1] ^= msg_actuators[i];
    }

    status = write(fh, msg_actuators, MSG_ACTUATORS_SIZE);
    if (status != MSG_ACTUATORS_SIZE) {
      RCLCPP_INFO(this->get_logger(), "Error sending actuator commands");
      // close(fh);
      // fh = open("/dev/i2c-4", O_RDWR);
    }

    status = read(fh, msg_sensors, MSG_SENSORS_SIZE);
    if (status != MSG_SENSORS_SIZE) {
      RCLCPP_INFO(this->get_logger(), "Error receiving sensor readings");
      // close(fh);
      // fh = open("/dev/i2c-4", O_RDWR);
    }

    // int n = read(fh, &epuck_to_zero_buff[bytesRead], SENSORS_SIZE-bytesRead);
  }

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_publisher;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription;

  int fh;
  char msg_actuators[MSG_ACTUATORS_SIZE];
  char msg_sensors[MSG_SENSORS_SIZE];
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EPuckPublisher>());
  rclcpp::shutdown();
  return 0;
}