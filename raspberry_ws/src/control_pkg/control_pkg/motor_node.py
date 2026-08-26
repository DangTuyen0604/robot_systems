#!/usr/bin/env python3
"""Serial motor bridge with command watchdog and reconnect handling."""

import rclpy
import serial
from interfaces_pkg.msg import Control
from rclpy.node import Node
from std_msgs.msg import String


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
        self.last_connect_attempt = 0.0
        self.watchdog_active = True
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
        self._write('V:0;E:0\n')

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
                    self.get_logger().info(f'Arduino: {line}')
        except (serial.SerialException, OSError) as exc:
            self._disconnect(exc)

    def pid_callback(self, msg):
        self._write(msg.data.strip() + '\n')

    def cmd_callback(self, msg):
        self.last_command_time = self._now()
        self.watchdog_active = False
        speed = max(0.0, min(1.0, float(msg.speed)))
        steering = max(-45.0, min(45.0, float(msg.steering)))
        base_v = int(speed * 255)
        error_px = int(steering * (320.0 / 45.0))
        self._write(f'V:{base_v};E:{error_px}\n')

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
    rclpy.init(args=args)
    node = MotorBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
