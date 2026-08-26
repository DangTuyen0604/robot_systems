#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interfaces_pkg.msg import Control
from geometry_msgs.msg import Twist
from std_msgs.msg import String

MAX_LINEAR_SPEED = 0.3
MAX_STEERING_DEG = 45.0
MAX_ANGULAR_SPEED = 1.5
# Control.steering is positive to the right, whereas ROS angular.z is
# positive counter-clockwise (to the left).
STEERING_SIGN = -1.0


def control_to_velocity(speed, steering, max_linear, max_angular):
    """Convert bounded normalized control to Gazebo linear/angular velocity."""
    bounded_speed = max(0.0, min(1.0, float(speed)))
    bounded_steering = max(
        -1.0, min(1.0, float(steering) / MAX_STEERING_DEG))
    return (
        bounded_speed * max_linear,
        STEERING_SIGN * bounded_steering * max_angular,
    )


class SimMotorBridge(Node):

    def __init__(self):
        super().__init__('sim_motor_bridge_node')
        self.declare_parameter('command_timeout', 0.3)
        self.declare_parameter('max_linear_speed', MAX_LINEAR_SPEED)
        self.declare_parameter('max_angular_speed', MAX_ANGULAR_SPEED)
        self.command_timeout = float(
            self.get_parameter('command_timeout').value)
        self.max_linear_speed = float(
            self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(
            self.get_parameter('max_angular_speed').value)
        self.last_command_time = None
        self.timed_out = True
        self.sub = self.create_subscription(
            Control, '/control/cmd', self.callback, 10
        )
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(
            String, '/diagnostics/sim_motor', 10)
        self.create_timer(0.05, self.watchdog_callback)
        self.get_logger().info('SimMotorBridge READY: /control/cmd -> /cmd_vel')

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def publish_stop(self, reason):
        self.pub.publish(Twist())
        status = String()
        status.data = reason
        self.status_pub.publish(status)

    def watchdog_callback(self):
        if (self.last_command_time is None or
                self.now_sec() - self.last_command_time > self.command_timeout):
            self.publish_stop('ERROR:COMMAND_TIMEOUT')
            if not self.timed_out:
                self.get_logger().error('COMMAND_TIMEOUT: simulated robot stopped')
            self.timed_out = True

    def callback(self, msg: Control):
        self.last_command_time = self.now_sec()
        self.timed_out = False
        twist = Twist()
        twist.linear.x, twist.angular.z = control_to_velocity(
            msg.speed, msg.steering,
            self.max_linear_speed, self.max_angular_speed)
        self.pub.publish(twist)
        status = String()
        status.data = f'OK:{msg.message}'
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = SimMotorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop('STOP:BRIDGE_SHUTDOWN')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
