# Copyright 1996-2020 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit tests for pi-puck driver. It mocks I2C and the rest of ROS2 system."""

import time
import rclpy
from math import pi
import unittest
import launch
import launch_ros.actions
import launch_testing.actions
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range, LaserScan, Imu
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import ParameterType, ParameterValue
from rcl_interfaces.msg._parameter import Parameter
import os


SENSORS_SIZE = 47
ACTUATOR_SIZE = 20
DISTANCE_FROM_CENTER = 0.035
READ_WRITE_RETRY_COUNT = 10
READ_WRITE_RETRY_DELAY = 0.5
MESSAGE_SEND_RETRY_COUNT = 10
MESSAGE_SEND_DELAY = 0.5


def arr2int16(arr):
    """Little-endian formulation."""
    val = (arr[0] & 0x00FF) | ((arr[1] << 8) & 0xFF00)
    if val > 2**15:
        val -= 2**16
    return val


def int162arr(val):
    """Little-endian formulation."""
    val = int(val)
    if val < 0:
        val += 2**16
    arr = [
        val & 0xFF,
        (val >> 8) & 0xFF
    ]
    return arr


def read_params_from_i2c(idx=4, address=0x1F):
    params = {}
    for _ in range(READ_WRITE_RETRY_COUNT):
        with open(f'/tmp/dev/i2c-{idx}_write_' + str(address), 'r+b') as f:
            buffer = list(f.read())
            if len(buffer) > 0:
                params['left_speed'] = arr2int16(buffer[0:2])
                params['right_speed'] = arr2int16(buffer[2:4])
                params['led1'] = buffer[6:9]
                return params, buffer
        time.sleep(READ_WRITE_RETRY_DELAY)
    return params, []


def write_params_to_i2c(params, idx=4, address=0x1F):
    buffer = [0] * SENSORS_SIZE

    # Fill the buffer
    for key in params.keys():
        # Distance sensors
        if key[:2] == 'ps':
            sid = int(key[2])
            buffer[2 * sid:2 * sid + 2] = int162arr(params[key])
        elif key == 'left_position':
            buffer[41:43] = int162arr(params[key])
        elif key == 'right_position':
            buffer[43:45] = int162arr(params[key])

    buffer[SENSORS_SIZE - 1] = 0
    for i in range(SENSORS_SIZE - 1):
        buffer[SENSORS_SIZE - 1] ^= buffer[i]

    # Write the buffer
    for _ in range(READ_WRITE_RETRY_COUNT):
        with open(f'/tmp/dev/i2c-{idx}_read_' + str(address), 'w+b') as f:
            n_bytes = f.write(bytearray(buffer))
            if n_bytes == SENSORS_SIZE:
                return
        time.sleep(READ_WRITE_RETRY_DELAY)


def check_topic_condition(
        node, topic_class, topic_name, condition, timeout_sec=2):
    msgs_rx = []
    sub = node.create_subscription(
        topic_class,
        topic_name,
        lambda msg: msgs_rx.append(msg),
        1
    )
    end_time = time.time() + timeout_sec
    while time.time() < end_time:
        rclpy.spin_once(node, timeout_sec=0.1)
        for msg in msgs_rx:
            if condition(msg):
                node.destroy_subscription(sub)
                return True
    node.destroy_subscription(sub)
    return False


def publish_twist(node, linear_x=0.0, linear_y=0.0, angular_z=0.0):
    # Publish a message
    pub = node.create_publisher(
        Twist,
        'cmd_vel',
        1
    )
    for _ in range(MESSAGE_SEND_RETRY_COUNT):
        msg = Twist()
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = angular_z
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.linear.z = 0.0
        pub.publish(msg)
        time.sleep(MESSAGE_SEND_DELAY)

    # Wait a bit
    time.sleep(MESSAGE_SEND_DELAY)
    node.destroy_publisher(pub)


def set_param(cli, name, value):
    req = SetParameters.Request()
    param_value = ParameterValue(
        double_value=value, type=ParameterType.PARAMETER_DOUBLE)
    param = Parameter(name=name, value=param_value)
    req.parameters.append(param)
    cli.call_async(req)


def generate_test_description():
    """
    Launch decription configuration.

    To run the tests you can use either `launch_test` directly as:
    $ launch_test src/epuck_ros2/epuck_ros2_driver/test/test_driver.py
    or `colcon`:
    $ colcon test --packages-select epuck_ros2_driver

    The testing procedure is based on `launch_test`:
    https://github.com/ros2/launch/tree/master/launch_testing
    and the following example:
    https://github.com/ros2/launch_ros/blob/master/launch_testing_ros/test/examples/talker_listener_launch_test.py.
    """
    # Inital IMU data
    if not os.path.exists('/tmp/dev'):
        os.makedirs('/tmp/dev')
    with open(f'/tmp/dev/i2c-12_read_' + str(0x68), 'w+b') as f:
        f.write(bytearray([0] * 6))

    controller = launch_ros.actions.Node(
        package='epuck_ros2_driver',
        node_executable='driver',
        output='screen',
        arguments=['--type', 'test']
    )

    return launch.LaunchDescription([
        controller,
        launch_testing.actions.ReadyToTest(),
    ])


class TestController(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('driver_tester')

    def tearDown(self):
        self.node.destroy_node()

    def test_checksum(self, launch_service, proc_output):
        publish_twist(self.node, angular_z=1.0)

        _, buffer = read_params_from_i2c()
        checksum = 0
        for i in range(ACTUATOR_SIZE - 1):
            checksum ^= buffer[i]

        self.assertEqual(checksum, buffer[ACTUATOR_SIZE - 1])

    def test_forward_velocity(self, launch_service, proc_output):
        publish_twist(self.node, linear_x=0.02)

        # Check what has been written to I2C
        params, _ = read_params_from_i2c()
        self.assertEqual(
            params['left_speed'],
            147,
            'Left wheel speed of 0.02 should correspond to 147 over I2C'
        )
        self.assertEqual(
            params['right_speed'],
            147,
            'Right wheel speed of 0.02 should correspond to 147 over I2C'
        )

    def test_rotation(self, launch_service, proc_output):
        publish_twist(self.node, angular_z=-0.5)

        # Check what has been written to I2C
        params, _ = read_params_from_i2c()
        self.assertEqual(params['left_speed'], -1 * params['right_speed'],
                         'The wheel should rotate in opposite direction')

    def test_limits(self, launch_service, proc_output):
        publish_twist(self.node, linear_x=1.0)

        # Check what has been written to I2C
        params, _ = read_params_from_i2c()
        self.assertEqual(params['left_speed'], 1108,
                         'The speed should be in range of [-1108, 1108]')
        self.assertEqual(params['right_speed'], 1108,
                         'The speed should be in range of [-1108, 1108]')

    def test_distance_sensors(self, launch_service, proc_output):
        write_params_to_i2c({'ps0': 120})
        condition = check_topic_condition(
            self.node,
            Range,
            'ps0',
            lambda msg: abs(msg.range - 0.05) < 1E-3)
        self.assertTrue(
            condition, 'The node hasn\'t published any distance measurement')

        write_params_to_i2c({'ps1': 383})
        condition = check_topic_condition(
            self.node,
            Range,
            'ps1',
            lambda msg: abs(msg.range - 0.02) < 1E-3
        )
        self.assertTrue(
            condition, 'The node hasn\'t published any distance measurement')

    def test_laser_scan(self, launch_service, proc_output):
        write_params_to_i2c({'ps3': 1465.73})
        condition = check_topic_condition(
            self.node,
            LaserScan,
            'scan',
            lambda msg: abs(msg.ranges[0] - 0.01 -
                            DISTANCE_FROM_CENTER) < 1E-2)
        self.assertTrue(
            condition, 'Sensor ps3 at -150 doesn\'t give a good results')

        write_params_to_i2c({'ps2': 1465.73})
        condition = check_topic_condition(
            self.node,
            LaserScan,
            'scan',
            lambda msg: abs(msg.ranges[4] - 0.01 - DISTANCE_FROM_CENTER) < 1E-2
        )
        self.assertTrue(
            condition, 'Sensor ps2 at -90 doesn\'t give a good results')

    def test_odometry_forward(self, launch_service, proc_output):
        # This will restart odometry
        write_params_to_i2c({'left_position': 0})
        write_params_to_i2c({'right_position': 0})
        cli = self.node.create_client(
            SetParameters, 'pipuck_driver/set_parameters')
        cli.wait_for_service(timeout_sec=1.0)
        set_param(cli, 'wheel_distance', 0.05685)
        set_param(cli, 'wheel_radius', 0.02)
        time.sleep(MESSAGE_SEND_DELAY)

        # Set odometry to I2C and verify
        write_params_to_i2c({'left_position': 2000 / (2 * pi)})
        write_params_to_i2c({'right_position': 2000 / (2 * pi)})
        condition = check_topic_condition(
            self.node,
            Odometry,
            'odom',
            lambda msg: abs(msg.pose.pose.position.x > 0.01)
        )
        self.assertTrue(condition, 'Should move forward')

        # Set odometry to I2C and verify
        write_params_to_i2c({'left_position': 4000 / (2 * pi)})
        write_params_to_i2c({'right_position': 4000 / (2 * pi)})
        condition = check_topic_condition(
            self.node,
            Odometry,
            'odom',
            lambda msg: abs(msg.pose.pose.position.x > 0.02)
        )
        self.assertTrue(condition, 'Should move more forward')

    def test_odometry_backward(self, launch_service, proc_output):
        # This will restart odometry
        write_params_to_i2c({'left_position': 0})
        write_params_to_i2c({'right_position': 0})
        cli = self.node.create_client(
            SetParameters, 'pipuck_driver/set_parameters')
        cli.wait_for_service(timeout_sec=1.0)
        set_param(cli, 'wheel_distance', 0.05685)
        set_param(cli, 'wheel_radius', 0.02)
        time.sleep(MESSAGE_SEND_DELAY)

        # Set odometry to I2C and verify
        write_params_to_i2c({'left_position': -2000 / (2 * pi)})
        write_params_to_i2c({'right_position': -2000 / (2 * pi)})
        condition = check_topic_condition(
            self.node,
            Odometry,
            'odom',
            lambda msg: abs(msg.pose.pose.position.x < -0.01)
        )
        self.assertTrue(condition, 'Should move backward')

    def test_imu(self, launch_service, proc_output):
        with open(f'/tmp/dev/i2c-12_read_' + str(0x68), 'w+b') as f:
            f.write(bytearray([0]*6))
        time.sleep(MESSAGE_SEND_DELAY)
        condition = check_topic_condition(
            self.node,
            Imu,
            'imu',
            lambda msg: abs(msg.linear_acceleration.x < 0.01)
        )
        self.assertTrue(condition, 'IMU should publish zeros')

        with open(f'/tmp/dev/i2c-12_read_' + str(0x68), 'w+b') as f:
            f.write(bytearray([127, 0] * 3))
        time.sleep(MESSAGE_SEND_DELAY)
        condition = check_topic_condition(
            self.node,
            Imu,
            'imu',
            lambda msg: msg.linear_acceleration.x > 1
        )
        self.assertTrue(
            condition, 'IMU should publish value greater than 1m/s^2')

    def test_rgb(self, launch_service, proc_output):
        # Publish a message
        pub = self.node.create_publisher(
            Int32,
            'led1',
            1
        )
        for _ in range(3):
            msg = Int32()
            msg.data = 0xFFFFFF
            pub.publish(msg)
            time.sleep(MESSAGE_SEND_DELAY)

        self.node.destroy_publisher(pub)
        # Check what has been written to I2C
        params, _ = read_params_from_i2c()
        self.assertTrue(all([i == 100 for i in params['led1']]))
