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

        self.declare_parameter('device', 0)
        self.declare_parameter('width', 320)
        self.declare_parameter('height', 240)
        self.declare_parameter('fps', 30)
        self.declare_parameter('publish_fps', 20.0)
        self.declare_parameter('jpeg_quality', 50)
        device = self.get_parameter('device').value
        width = int(self.get_parameter('width').value)
        height = int(self.get_parameter('height').value)
        fps = int(self.get_parameter('fps').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)

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

        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)

        self.cap.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*'MJPG')
        )

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open camera device {device}')

        self.get_logger().info('Camera started')

        self.prev = time.time()

        self.timer = self.create_timer(
            1.0 / float(self.get_parameter('publish_fps').value),
            self.timer_callback
        )

    def timer_callback(self):

        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().error(
                'Camera frame read failed', throttle_duration_sec=2.0)
            return

        msg = CompressedImage()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.format = "jpeg"

        _, buffer = cv2.imencode(
            '.jpg',
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
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
