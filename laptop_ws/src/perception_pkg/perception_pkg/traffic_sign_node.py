#!/usr/bin/env python3
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Float32

import cv2
import os
import numpy as np
import math
import torch

from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from perception_pkg.node_runner import image_age, run_node
from perception_pkg.sign_labels import (
    ACTIVE_SIGN_LABELS, build_class_map, pick_detection)

# Fallback only: used when the loaded model carries no usable class names.
SIGN_CLASSES = {
    0: 'no_right_turn',
    1: 'slow_down',
    2: 'stop',
    3: 'turn_left',
    4: 'speed_limit_20',
}

CLASS_DISPLAY_MAP = {
    'no_right_turn':   'NO RIGHT TURN',
    'slow_down':        'SLOW DOWN',
    'stop':              'STOP',
    'turn_left':         'TURN LEFT',
    'speed_limit_20':    'SPEED LIMIT 20',
    'none':              'NO SIGN DETECTED'
}

DEFAULT_SIGN_CONF = 0.5

# ✅ All these labels execute IMMEDIATELY upon detection.
# Distance is not used for them since the small test-track camera
# gives inaccurate distance estimates.
IMMEDIATE_LABELS = {'stop', 'speed_limit_20', 'turn_left'}

CAMERA_PARAMS = {
    'f':     530.0,
    'dy':    1.0,
    'h':     0.26,
    'alpha': 35.0,
    'v0':    240.0,
}

DISTANCE_EXECUTE_THRESHOLD = 0.35


class TrafficSignNode(Node):

    def __init__(self):
        super().__init__('sign_node_instance')

        # The custom viet.pt detector was trained at 640.  Running it at 320
        # caused small signs to disappear before the detector feature maps.
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('process_every_n_frames', 2)
        self.declare_parameter('sign_conf', DEFAULT_SIGN_CONF)
        self.declare_parameter('publish_image', False)
        self.declare_parameter('confirm_frames', 3)
        # Drop images older than this (s, by capture stamp); 0 disables.
        self.declare_parameter('max_image_age', 0.5)
        self.declare_parameter('sign_model_file', 'viet.pt')
        self.declare_parameter('camera_f',     CAMERA_PARAMS['f'])
        self.declare_parameter('camera_dy',    CAMERA_PARAMS['dy'])
        self.declare_parameter('camera_h',     CAMERA_PARAMS['h'])
        self.declare_parameter('camera_alpha', CAMERA_PARAMS['alpha'])
        self.declare_parameter('camera_v0',    CAMERA_PARAMS['v0'])

        self.imgsz = self.get_parameter('imgsz').value
        self.process_every = self.get_parameter('process_every_n_frames').value
        self.sign_conf = float(self.get_parameter('sign_conf').value)
        self.confirm_frames = max(1, int(self.get_parameter('confirm_frames').value))
        self.max_image_age = float(self.get_parameter('max_image_age').value)
        self.sign_model_file = str(
            self.get_parameter('sign_model_file').value)
        self.candidate_label = 'none'
        self.candidate_count = 0

        self.cam_f = float(self.get_parameter('camera_f').value)
        self.cam_dy = float(self.get_parameter('camera_dy').value)
        self.cam_h = float(self.get_parameter('camera_h').value)
        self.cam_alpha = float(self.get_parameter('camera_alpha').value)
        self.cam_v0 = float(self.get_parameter('camera_v0').value)
        self.a_y = self.cam_f / self.cam_dy

        self.frame_count = 0

        package_dir = get_package_share_directory('perception_pkg')
        sign_path = os.path.join(package_dir, self.sign_model_file)

        self.sign_uses_onnx = sign_path.lower().endswith('.onnx')
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'Device: {self.device}')

        try:
            from ultralytics import YOLO
            self.sign_model = YOLO(sign_path, task='detect')
            self.get_logger().info('Sign model loaded successfully!')
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            raise

        try:
            sign_model_names = self.sign_model.names
        except Exception as e:
            self.get_logger().warn(f'Cannot read sign model class names: {e}')
            sign_model_names = None
        self.sign_class_map, source = build_class_map(
            sign_model_names, SIGN_CLASSES, ACTIVE_SIGN_LABELS)
        self.get_logger().info(
            f'Sign classes from {source}: {self.sign_class_map} '
            f'(model names: {sign_model_names})')
        if source == 'fallback':
            self.get_logger().warn(
                'Sign model names do not match any active label; using the '
                'hard-coded SIGN_CLASSES order. Verify it against the model.')
        missing = ACTIVE_SIGN_LABELS - set(self.sign_class_map.values())
        if missing:
            self.get_logger().warn(f'Sign model cannot detect: {sorted(missing)}')

        self.label_pub = self.create_publisher(String,  '/perception/detected_label', 10)
        self.dist_pub = self.create_publisher(Float32, '/perception/sign_distance',  10)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ✅ Subscribe to the raw image
        self.subscription = self.create_subscription(
            CompressedImage,
            '/raw_image/compressed',
            self.listener_callback,
            qos
        )

        self.get_logger().info('TrafficSignNode READY')

    def decode_yolo_ultralytics(self, results, class_map, conf_threshold):
        if results is None or len(results) == 0:
            return 'none', 0.0, None

        result = results[0]
        boxes = result.boxes

        if boxes is None or len(boxes) == 0:
            return 'none', 0.0, None

        return pick_detection(
            boxes.cls.cpu().numpy(),
            boxes.conf.cpu().numpy(),
            boxes.xyxy.cpu().numpy()[:, 3],
            class_map,
            conf_threshold,
        )

    def compute_distance(self, v_pixel: float) -> float:
        alpha_rad = math.radians(self.cam_alpha)
        angle = alpha_rad + math.atan2(v_pixel - self.cam_v0, self.a_y)
        tan_val = math.tan(angle)
        if abs(tan_val) < 1e-6:
            return float('inf')
        d = self.cam_h / tan_val
        if d < 0:
            return float('inf')
        return d

    def listener_callback(self, msg):
        self.frame_count += 1
        if self.frame_count % self.process_every != 0:
            return
        if self.max_image_age > 0.0:
            age = image_age(self.get_clock().now(), Time.from_msg(msg.header.stamp))
            if age is not None and not -self.max_image_age <= age <= self.max_image_age:
                self.get_logger().error(
                    f'Dropping camera image {age:+.2f} s old (limit {self.max_image_age} s)',
                    throttle_duration_sec=2.0)
                return

        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        sign_predict_args = {
            'imgsz': self.imgsz,
            'verbose': False,
        }
        # ONNX Runtime selects its own provider. Passing a torch device here
        # can make Ultralytics initialize an unnecessary PyTorch inference path.
        if not self.sign_uses_onnx:
            sign_predict_args['device'] = self.device
        sign_results = self.sign_model.predict(frame, **sign_predict_args)

        label, conf, v_pixel = self.decode_yolo_ultralytics(
            sign_results, self.sign_class_map, self.sign_conf
        )

        # Temporal confirmation prevents a single bad inference from changing
        # vehicle behaviour. A disappearance is published immediately so a
        # consumed STOP can be re-armed only after it really leaves the scene.
        if label == 'none':
            self.candidate_label = 'none'
            self.candidate_count = 0
        elif label == self.candidate_label:
            self.candidate_count += 1
        else:
            self.candidate_label = label
            self.candidate_count = 1
        if label != 'none' and self.candidate_count < self.confirm_frames:
            label = 'none'
            conf = 0.0
            v_pixel = None

        # Compute actual distance for logging/debugging
        if v_pixel is not None:
            distance = self.compute_distance(v_pixel)
        else:
            distance = float('inf')

        # ✅ IMMEDIATE_LABELS → execute right away, distance=0.0
        # Other labels → only once estimated within the execute threshold
        if label in IMMEDIATE_LABELS:
            effective_distance = 0.0
        elif distance <= DISTANCE_EXECUTE_THRESHOLD:
            effective_distance = 0.0
        else:
            effective_distance = distance

        msg_out = String()
        msg_out.data = f'{label}:{effective_distance:.3f}'
        self.label_pub.publish(msg_out)

        dist_msg = Float32()
        dist_msg.data = distance if distance != float('inf') else -1.0
        self.dist_pub.publish(dist_msg)

        if label != 'none':
            dist_str = f'{distance:.2f}m' if distance != float('inf') else '∞'
            exec_str = 'EXECUTE NOW' if effective_distance == 0.0 else f'WAITING ({dist_str})'
            self.get_logger().info(
                f"[DETECTED]: {CLASS_DISPLAY_MAP.get(label, label)} | "
                f"Confidence: {conf*100:.1f}% | "
                f"Distance: {dist_str} | "
                f"Status: {exec_str}",
                throttle_duration_sec=0.5
            )


def main(args=None):
    run_node(TrafficSignNode, args=args)


if __name__ == '__main__':
    main()
