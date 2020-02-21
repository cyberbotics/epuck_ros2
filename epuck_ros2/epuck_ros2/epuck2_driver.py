import rclpy
from rclpy.node import Node
from .controller import Robot
from geometry_msgs.msg import Twist


WHEEL_DISTANCE = 0.05685
WHEEL_RADIUS = 0.02


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
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.get_logger().info('EPuck Initialized')

        # Steps...
        # self.create_timer(self.timestep * 2, self.step_callback)

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
        self.robot.step(self.timestep)


def main(args=None):
    rclpy.init(args=args)

    epuck2_controller = EPuck2Controller('epuck2', args=args)

    rclpy.spin(epuck2_controller)

    epuck2_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()