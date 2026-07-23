#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interfaces_pkg.msg import Control
from geometry_msgs.msg import Twist

MAX_LINEAR_SPEED = 0.3
MAX_STEERING_DEG = 45.0
MAX_ANGULAR_SPEED = 1.5
STEERING_SIGN = 1.0


class SimMotorBridge(Node):

    def __init__(self):
        super().__init__('sim_motor_bridge_node')
        self.sub = self.create_subscription(
            Control, '/control/auto', self.callback, 10
        )
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('SimMotorBridge READY: /control/auto -> /cmd_vel')

    def callback(self, msg: Control):
        twist = Twist()
        speed = max(0.0, min(1.0, float(msg.speed)))
        twist.linear.x = speed * MAX_LINEAR_SPEED
        steering_ratio = max(-1.0, min(1.0, float(msg.steering) / MAX_STEERING_DEG))
        twist.angular.z = STEERING_SIGN * steering_ratio * MAX_ANGULAR_SPEED
        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = SimMotorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
