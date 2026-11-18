import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')

        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_separation', 0.4)
        self.declare_parameter('max_rpm', 3000.0)
        self.declare_parameter('time_constant', 0.5)
        self.declare_parameter('publish_rate', 50.0)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.max_rpm = self.get_parameter('max_rpm').value
        self.time_constant = self.get_parameter('time_constant').value
        self.dt = 1.0 / self.get_parameter('publish_rate').value

        self.target_rpm = [0.0, 0.0]
        self.current_rpm = [0.0, 0.0]
        self.last_cmd_time = self.get_clock().now()

        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.rpm_pub = self.create_publisher(Float32MultiArray, 'motor/rpm', 10)
        self.create_timer(self.dt, self.update)
        self.get_logger().info('Motor node started, waiting for /cmd_vel')

    def clamp(self, rpm):
        return max(-self.max_rpm, min(self.max_rpm, rpm))

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()
        half = msg.angular.z * self.wheel_separation / 2.0
        v_left = msg.linear.x - half
        v_right = msg.linear.x + half
        k = 60.0 / (2.0 * math.pi * self.wheel_radius)
        self.target_rpm = [self.clamp(v_left * k), self.clamp(v_right * k)]
        self.get_logger().info(
            f'cmd: lin={msg.linear.x:.2f} ang={msg.angular.z:.2f} -> '
            f'target L={self.target_rpm[0]:.1f} R={self.target_rpm[1]:.1f} rpm')

    def update(self):
        idle = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if idle > 1.0:
            self.target_rpm = [0.0, 0.0]
        alpha = 1.0 - math.exp(-self.dt / self.time_constant)
        for i in range(2):
            self.current_rpm[i] += (self.target_rpm[i] - self.current_rpm[i]) * alpha
        out = Float32MultiArray()
        out.data = [float(self.current_rpm[0]), float(self.current_rpm[1])]
        self.rpm_pub.publish(out)


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
