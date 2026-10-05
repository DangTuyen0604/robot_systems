#!/usr/bin/env python3
"""Serial motor bridge with command watchdog and reconnect handling."""

import signal

import rclpy
import serial
from interfaces_pkg.msg import Control
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from control_pkg.serial_protocol import (
    STOP_PACKET, motion_packet, pid_rejection_reason)


class MotorBridgeNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.declare_parameter('serial_device', '/dev/robot_arduino')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('command_timeout', 0.3)
        self.declare_parameter('reconnect_interval', 2.0)
        self.device = str(self.get_parameter('serial_device').value)
        self.baud = int(self.get_parameter('baud_rate').value)
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        self.reconnect_interval = float(self.get_parameter('reconnect_interval').value)
        self.ser = None
        self.last_command_time = None
        self.commanded_speed = 0.0
        self.last_connect_attempt = 0.0
        self.watchdog_active = True
        # Every line the Arduino prints, e.g. the 10 Hz ERR/PWM/encoder debug.
        self.telemetry_pub = self.create_publisher(String, '/arduino/telemetry', 10)
        self.create_subscription(String, '/arduino/pid', self.pid_callback, 10)
        self.create_subscription(Control, '/control/cmd', self.cmd_callback, 10)
        self.create_timer(0.05, self.timer_callback)
        self._connect()

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _connect(self):
        self.last_connect_attempt = self._now()
        try:
            self.ser = serial.Serial(self.device, self.baud, timeout=0)
            self.get_logger().info(f'Connected to Arduino at {self.device}')
            self._send_stop()
        except (serial.SerialException, OSError) as exc:
            self.ser = None
            self.get_logger().error(f'Serial unavailable ({self.device}): {exc}')

    def _disconnect(self, exc):
        self.get_logger().error(f'Serial connection lost: {exc}')
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def _write(self, packet):
        if self.ser is None:
            return False
        try:
            self.ser.write(packet.encode('utf-8'))
            return True
        except (serial.SerialException, OSError) as exc:
            self._disconnect(exc)
            return False

    def _send_stop(self):
        self.commanded_speed = 0.0
        self._write(STOP_PACKET)

    def timer_callback(self):
        now = self._now()
        if self.ser is None:
            if now - self.last_connect_attempt >= self.reconnect_interval:
                self._connect()
            return
        if self.last_command_time is None or now - self.last_command_time > self.command_timeout:
            self._send_stop()
            if not self.watchdog_active:
                self.get_logger().error('COMMAND_TIMEOUT: motor stopped')
            self.watchdog_active = True
        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='replace').strip()
                if line:
                    self._handle_arduino_line(line)
        except (serial.SerialException, OSError) as exc:
            self._disconnect(exc)

    def _handle_arduino_line(self, line):
        msg = String()
        msg.data = line
        self.telemetry_pub.publish(msg)
        if line.startswith('ERR:'):
            # Periodic debug line: full stream is on /arduino/telemetry.
            self.get_logger().info(f'Arduino: {line}', throttle_duration_sec=2.0)
        else:
            self.get_logger().info(f'Arduino: {line}')

    def pid_callback(self, msg):
        packet = msg.data.strip()
        reason = pid_rejection_reason(packet, self.commanded_speed)
        if reason is not None:
            self.get_logger().warn(f'Rejected /arduino/pid "{packet[:32]}": {reason}')
            return
        self._write(packet + '\n')

    def cmd_callback(self, msg):
        self.last_command_time = self._now()
        self.watchdog_active = False
        self.commanded_speed = max(0.0, min(1.0, float(msg.speed)))
        self._write(motion_packet(msg.speed, msg.steering))

    def close(self):
        self._send_stop()
        if self.ser is not None:
            try:
                self.ser.flush()
                self.ser.close()
            except (serial.SerialException, OSError):
                pass
            self.ser = None


def main(args=None):
    # Ctrl+C reaches the node twice (terminal and ros2 launch). Handle the
    # first signal here and ignore the rest, so nothing interrupts close(),
    # which sends the final stop packet to the Arduino.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = []

    def on_signal(signum, frame):
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, signal.SIG_IGN)
        stop_requested.append(signum)

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, on_signal)
    node = MotorBridgeNode()
    try:
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
