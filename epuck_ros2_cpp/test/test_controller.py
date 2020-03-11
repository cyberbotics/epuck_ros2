
import time
import rclpy
import launch
import launch_ros.actions
import unittest
import launch_testing.actions
from geometry_msgs.msg import Twist


prev_data = []


def get_int16(bytearr):
    val = (bytearr[0] & 0x00FF) | ((bytearr[1] << 8) & 0xFF00)
    if val > 2**15:
        val -= 2**16
    return val


def get_params(buffer):
    params = {}
    params['left_speed'] = get_int16(buffer[0:2])
    params['right_speed'] = get_int16(buffer[2:4])
    return params


def read_i2c_data(idx=4):
    for _ in range(3):
        with open(f'/tmp/dev/i2c-{idx}_write', 'rb') as f:
            data = list(f.read())
            if len(data) > 0:
                return get_params(data)
        time.sleep(0.01)
    return {}


def generate_test_description():
    # launch_test src/epuck_ros2/epuck_ros2_cpp/test/test_controller.py
    # Docs: https://github.com/ros2/launch/tree/master/launch_testing
    # Example: https://github.com/ros2/launch_ros/blob/master/launch_testing_ros/test/examples/talker_listener_launch_test.py

    controller = launch_ros.actions.Node(
        package='epuck_ros2_cpp',
        node_executable='controller'
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
        self.node = rclpy.create_node('test_controller')

    def tearDown(self):
        self.node.destroy_node()

    def test_forward_velocity(self, launch_service, proc_output):
        # Publish a message
        pub = self.node.create_publisher(
            Twist,
            'cmd_vel',
            1
        )
        msg = Twist()
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        msg.linear.x = 0.02
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        pub.publish(msg)

        # Wait a bit
        time.sleep(0.1)
        self.node.destroy_publisher(pub)

        # Check what has been written to I2C
        params = read_i2c_data()
        self.assertEqual(params['left_speed'], 147)
        self.assertEqual(params['right_speed'], 147)
