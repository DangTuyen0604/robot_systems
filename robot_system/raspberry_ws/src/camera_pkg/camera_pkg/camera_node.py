import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

import cv2
import time

from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy


class CameraPublisher(Node):

    def __init__(self):

        super().__init__('camera_publisher_node')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.publisher_ = self.create_publisher(
            CompressedImage,
            '/raw_image/compressed',
            qos
        )

        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

        self.cap.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*'MJPG')
        )

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        self.get_logger().info('Camera started')

        self.prev = time.time()

        self.timer = self.create_timer(
            1.0 / 20.0,
            self.timer_callback
        )

    def timer_callback(self):

        ret, frame = self.cap.read()

        if not ret:
            return

        msg = CompressedImage()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.format = "jpeg"

        _, buffer = cv2.imencode(
            '.jpg',
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), 50]
        )

        msg.data = buffer.tobytes()

        self.publisher_.publish(msg)

        now = time.time()

        fps = 1.0 / (now - self.prev)

        self.prev = now

        self.get_logger().info(
            f'Publish FPS: {fps:.1f}',
            throttle_duration_sec=2.0
        )


def main(args=None):

    rclpy.init(args=args)

    node = CameraPublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.cap.release()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
