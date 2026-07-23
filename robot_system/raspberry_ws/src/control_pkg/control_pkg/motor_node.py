#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interfaces_pkg.msg import Control
from std_msgs.msg import String
import serial
import time


class MotorBridgeNode(Node):
    def __init__(self):
        super().__init__('motor_node')

        # ✅ Gán None trước để tránh crash nếu Serial thất bại
        self.ser = None

        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            time.sleep(2)
            self.get_logger().info('Đã kết nối Arduino qua Serial!')
        except Exception as e:
            self.get_logger().error(f'Không thể kết nối Serial: {e}')
            self.get_logger().warn('Node vẫn chạy nhưng không gửi được lệnh!')

        self.create_subscription(String,  '/arduino/pid',  self.pid_callback, 10)
        self.create_subscription(Control, '/control/cmd',  self.cmd_callback, 10)
        self.create_timer(0.05, self.read_serial_callback)

    # ------------------------------------------------------------------
    def read_serial_callback(self):
        if self.ser is None:
            return
        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    self.get_logger().info(f'Arduino: {line}')
        except Exception as e:
            self.get_logger().error(f'Lỗi đọc Serial: {e}')

    # ------------------------------------------------------------------
    def pid_callback(self, msg: String):
        if self.ser is None:
            return
        packet = msg.data.strip() + '\n'
        try:
            self.ser.write(packet.encode('utf-8'))
            self.get_logger().info(f'Sent PID: {packet.strip()}')
        except Exception as e:
            self.get_logger().error(f'Lỗi gửi PID: {e}')

    # ------------------------------------------------------------------
    def cmd_callback(self, msg: Control):
        if self.ser is None:
            return

        # speed: 0.0..1.0 → 0..255
        base_v = int(msg.speed * 255)
        base_v = max(0, min(255, base_v))

        # steering: độ (-45..45) → pixel error (-320..320)
        error_px = int(msg.steering * (320.0 / 45.0))
        error_px = max(-400, min(400, error_px))

        packet = f'V:{base_v};E:{error_px}\n'

        self.get_logger().info(
            f'→ Arduino: speed={msg.speed:.2f}({base_v}) '
            f'steering={msg.steering:.1f}° error_px={error_px} '
            f'msg={msg.message}'
        )

        try:
            self.ser.write(packet.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f'Lỗi gửi Serial: {e}')

    # ------------------------------------------------------------------
    def __del__(self):
        if self.ser is not None:
            self.ser.close()


def main(args=None):
    rclpy.init(args=args)
    node = MotorBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
