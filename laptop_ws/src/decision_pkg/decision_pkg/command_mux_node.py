#!/usr/bin/env python3
"""Fail-safe command multiplexer for autonomous and manual control."""

import rclpy
from interfaces_pkg.msg import Control
from rclpy.node import Node
from std_msgs.msg import String


def select_command(mode, commands, received_at, now, timeout):
    """Return ``(command, status)`` for a mux state without ROS side effects."""
    if mode == 'ESTOP':
        return None, 'ESTOP'
    received = received_at.get(mode)
    if received is None or now - received > timeout:
        return None, f'{mode}_TIMEOUT'
    return commands.get(mode), mode


class CommandMuxNode(Node):
    """Publish exactly one selected, fresh command on ``/control/cmd``."""

    VALID_MODES = {'AUTO', 'MANUAL', 'ESTOP'}

    def __init__(self):
        super().__init__('command_mux_node')
        self.declare_parameter('initial_mode', 'ESTOP')
        self.declare_parameter('command_timeout', 0.3)
        self.declare_parameter('publish_period', 0.05)

        requested_mode = str(self.get_parameter('initial_mode').value).upper()
        self.mode = requested_mode if requested_mode in self.VALID_MODES else 'ESTOP'
        self.timeout = float(self.get_parameter('command_timeout').value)
        period = float(self.get_parameter('publish_period').value)

        self.commands = {'AUTO': None, 'MANUAL': None}
        self.received_at = {'AUTO': None, 'MANUAL': None}
        self.cmd_pub = self.create_publisher(Control, '/control/cmd', 10)
        self.status_pub = self.create_publisher(String, '/control/mux_status', 10)
        self.create_subscription(Control, '/control/auto', self._auto_cb, 10)
        self.create_subscription(Control, '/control/manual', self._manual_cb, 10)
        self.create_subscription(String, '/control/mode', self._mode_cb, 10)
        self.create_timer(period, self._tick)
        self.get_logger().warn(f'Command mux started in {self.mode}')

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _auto_cb(self, msg):
        self.commands['AUTO'] = msg
        self.received_at['AUTO'] = self._now()

    def _manual_cb(self, msg):
        self.commands['MANUAL'] = msg
        self.received_at['MANUAL'] = self._now()

    def _mode_cb(self, msg):
        requested = msg.data.strip().upper()
        if requested not in self.VALID_MODES:
            self.get_logger().warn(f'Ignoring invalid control mode: {requested}')
            return
        if requested != self.mode:
            self.mode = requested
            self.get_logger().warn(f'Control mode changed to {self.mode}')

    @staticmethod
    def _stop(reason):
        cmd = Control()
        cmd.speed = 0.0
        cmd.steering = 0.0
        cmd.message = reason
        return cmd

    def _tick(self):
        cmd, status = select_command(
            self.mode, self.commands, self.received_at,
            self._now(), self.timeout)
        if cmd is None:
            cmd = self._stop(status)
        self.cmd_pub.publish(cmd)
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CommandMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(node._stop('MUX_SHUTDOWN'))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
