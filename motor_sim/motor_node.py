import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.get_logger().info('Motor node started, waiting for /cmd_vel')

    def cmd_vel_callback(self, msg):
        self.get_logger().info(
            f'Received: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
