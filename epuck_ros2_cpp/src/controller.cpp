#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>

extern "C" {
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

#define MSG_ACTUATORS_SIZE 19

using namespace std::chrono_literals;

const int WHEEL_DISTANCE = 0.05685;
const int WHEEL_RADIUS = 0.02;

class EPuckPublisher : public rclcpp::Node
{
public:
  EPuckPublisher()
      : Node("pipuck_driver")
  {
    fh = open("/dev/i2c-4", O_RDWR);
    memset(msg_actuators + 4, 0, MSG_ACTUATORS_SIZE - 4);

    this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&EPuckPublisher::on_cmd_vel_received, this, std::placeholders::_1));

    publisher = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer = this->create_wall_timer(
        64ms, std::bind(&EPuckPublisher::update_callback, this));
  }

  ~EPuckPublisher() {
    close(fh);
  }

private:
  void on_cmd_vel_received(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    float left_velocity = (2.0 * msg->linear.x - msg->angular.z * WHEEL_DISTANCE) / (2.0 * WHEEL_RADIUS);
    float right_velocity = (2.0 * msg->linear.x + msg->angular.z * WHEEL_DISTANCE) / (2.0 * WHEEL_RADIUS);
    
    int left_velocity_big = left_velocity * 1000;
    int right_velocity_big = right_velocity * 1000;

    msg_actuators[0] = left_velocity_big & 0xFF;
		msg_actuators[1] = left_velocity_big >> 8;
		msg_actuators[2] = right_velocity_big & 0xFF;
		msg_actuators[3] = right_velocity_big >> 8;	
  }

  void update_callback()
  {
    ioctl(fh, I2C_SLAVE, 0x1F);
    write(fh, msg_actuators, MSG_ACTUATORS_SIZE);

    // int n = read(fh, &epuck_to_zero_buff[bytesRead], SENSORS_SIZE-bytesRead);
  }

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;

  int fh;
  char msg_actuators[MSG_ACTUATORS_SIZE]; 
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EPuckPublisher>());
  rclcpp::shutdown();
  return 0;
}