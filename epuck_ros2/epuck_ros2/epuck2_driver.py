import rclpy
from math import pi, cos, sin
from rclpy.node import Node
from .controller import Robot
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from geometry_msgs.msg import Twist


WHEEL_DISTANCE = 0.05685
WHEEL_RADIUS = 0.02
ENCODER_PERIOD_MS = 100

ENCODER_PERIOD_S = ENCODER_PERIOD_MS / 1000


def euler_to_quaternion(roll, pitch, yaw):
    """
    Source: https://computergraphics.stackexchange.com/a/8229
    """
    q = Quaternion()
    q.x = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - \
        cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    q.y = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + \
        sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    q.z = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - \
        sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    q.w = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + \
        sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return q


class EPuck2Controller(Node):
    def __init__(self, name, args=None):
        super().__init__(name)
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # Init motors
        self.left_motor = self.robot.getMotor('left wheel motor')
        self.right_motor = self.robot.getMotor('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 1)
        self.get_logger().info('EPuck Initialized')

        # Initialize odometry
        self.prev_left_wheel_ticks = 0
        self.prev_right_wheel_ticks = 0
        self.prev_position = (0, 0)
        self.prev_angle = 0
        self.left_wheel_sensor = self.robot.getPositionSensor(
            'left wheel sensor')
        self.right_wheel_sensor = self.robot.getPositionSensor(
            'right wheel sensor')
        self.left_wheel_sensor.enable(self.timestep)
        self.right_wheel_sensor.enable(self.timestep)
        self.odometry_publisher = self.create_publisher(Odometry, '/odom', 1)
        self.create_timer(ENCODER_PERIOD_MS / 1000, self.odometry_callback)

        # Steps...
        self.create_timer(self.timestep / 1000, self.step_callback)

    def step_callback(self):
        self.robot.step(self.timestep)

    def cmd_vel_callback(self, twist):
        self.get_logger().info('Message received')
        left_velocity = (2.0 * twist.linear.x - twist.angular.z *
                         WHEEL_DISTANCE) / (2.0 * WHEEL_RADIUS)
        right_velocity = (2.0 * twist.linear.x + twist.angular.z *
                          WHEEL_DISTANCE) / (2.0 * WHEEL_RADIUS)
        self.left_motor.setVelocity(left_velocity)
        self.right_motor.setVelocity(right_velocity)

    def odometry_callback(self):
        # Calculate velocities
        left_wheel_ticks = self.left_wheel_sensor.getValue()
        right_wheel_ticks = self.right_wheel_sensor.getValue()
        v_left_rad = (left_wheel_ticks -
                      self.prev_left_wheel_ticks) / ENCODER_PERIOD_S
        v_right_rad = (right_wheel_ticks -
                       self.prev_right_wheel_ticks) / ENCODER_PERIOD_S
        v_left = v_left_rad * WHEEL_RADIUS
        v_right = v_right_rad * WHEEL_RADIUS
        v = (v_left + v_right) / 2
        omega = (v_right - v_left) / WHEEL_DISTANCE

        # Calculate position & angle
        # Fourth order Runge - Kutta
        # Reference: https://www.cs.cmu.edu/~16311/s07/labs/NXTLabs/Lab%203.html
        k00 = v * cos(self.prev_angle)
        k01 = v * sin(self.prev_angle)
        k02 = omega
        k10 = v * cos(self.prev_angle + ENCODER_PERIOD_S * k02 / 2)
        k11 = v * sin(self.prev_angle + ENCODER_PERIOD_S * k02 / 2)
        k12 = omega
        k20 = v * cos(self.prev_angle + ENCODER_PERIOD_S * k12 / 2)
        k21 = v * sin(self.prev_angle + ENCODER_PERIOD_S * k12 / 2)
        k22 = omega
        k30 = v * cos(self.prev_angle + ENCODER_PERIOD_S * k22 / 2)
        k31 = v * sin(self.prev_angle + ENCODER_PERIOD_S * k22 / 2)
        k32 = omega
        position = [
            self.prev_position[0] + (ENCODER_PERIOD_S / 6) *
            (k00 + 2 * (k10 + k20) + k30),
            self.prev_position[1] + (ENCODER_PERIOD_S / 6) *
            (k01 + 2 * (k11 + k21) + k31)
        ]
        angle = self.prev_angle + \
            (ENCODER_PERIOD_S / 6) * (k02 + 2 * (k12 + k22) + k32)

        # Update variables
        self.prev_position = position.copy()
        self.prev_angle = angle
        self.prev_left_wheel_ticks = left_wheel_ticks
        self.prev_right_wheel_ticks = right_wheel_ticks

        # Pack & publish odometry
        msg = Odometry()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.twist.twist.linear.x = v
        msg.twist.twist.linear.z = omega
        msg.pose.pose.position.x = position[0]
        msg.pose.pose.position.y = position[1]
        msg.pose.pose.orientation = euler_to_quaternion(0, 0, angle)
        self.odometry_publisher.publish(msg)

        # Pack & publish transforms
        tf_broadcaster = TransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = position[0]
        tf.transform.translation.y = position[1]
        tf.transform.translation.z = 0.0
        tf.transform.rotation = euler_to_quaternion(0, 0, angle)
        tf_broadcaster.sendTransform(tf)

def main(args=None):
    rclpy.init(args=args)

    epuck2_controller = EPuck2Controller('epuck2', args=args)

    rclpy.spin(epuck2_controller)

    epuck2_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
