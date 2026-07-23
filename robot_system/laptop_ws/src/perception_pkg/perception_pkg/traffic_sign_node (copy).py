#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge

import cv2
import os
import numpy as np
import math
import torch

from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

SIGN_CLASSES = {
    0: 'cam_re_phai',
    1: 'di_cham',
    2: 'dung_lai',
    3: 're_trai',
    4: 'toc_do_toi_da_20',
}

LIGHT_CLASSES = {
    0: 'den_do',
    1: 'den_vang',
    2: 'den_xanh',
}

CLASS_VI_MAP = {
    'cam_re_phai':      'CẤM RẼ PHẢI',
    'di_cham':          'ĐI CHẬM',
    'dung_lai':         'STOP - DỪNG LẠI',
    're_trai':          'RẼ TRÁI',
    'toc_do_toi_da_20': 'TỐC ĐỘ TỐI ĐA 20',
    'den_do':           'ĐÈN ĐỎ',
    'den_vang':         'ĐÈN VÀNG',
    'den_xanh':         'ĐÈN XANH',
    'none':             'Không có biển báo'
}

CONF_THRESHOLD = 0.5

# ✅ Tất cả biển đều thực thi NGAY khi nhìn thấy
# Không dùng khoảng cách nữa vì camera sa bàn nhỏ tính không chính xác
IMMEDIATE_LABELS = {
    'dung_lai', 'den_do', 'den_vang', 'toc_do_toi_da_20', 're_trai',
}

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

        self.bridge = CvBridge()

        self.declare_parameter('imgsz', 320)
        self.declare_parameter('process_every_n_frames', 2)
        self.declare_parameter('camera_f',     CAMERA_PARAMS['f'])
        self.declare_parameter('camera_dy',    CAMERA_PARAMS['dy'])
        self.declare_parameter('camera_h',     CAMERA_PARAMS['h'])
        self.declare_parameter('camera_alpha', CAMERA_PARAMS['alpha'])
        self.declare_parameter('camera_v0',    CAMERA_PARAMS['v0'])

        self.imgsz         = self.get_parameter('imgsz').value
        self.process_every = self.get_parameter('process_every_n_frames').value

        self.cam_f     = float(self.get_parameter('camera_f').value)
        self.cam_dy    = float(self.get_parameter('camera_dy').value)
        self.cam_h     = float(self.get_parameter('camera_h').value)
        self.cam_alpha = float(self.get_parameter('camera_alpha').value)
        self.cam_v0    = float(self.get_parameter('camera_v0').value)
        self.a_y       = self.cam_f / self.cam_dy

        self.frame_count = 0

        package_dir = get_package_share_directory('perception_pkg')
        sign_path   = os.path.join(package_dir, 'viet.pt')
        light_path  = os.path.join(package_dir, 'traffic_light.pt')

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'Device: {self.device}')

        try:
            from ultralytics import YOLO
            self.sign_model  = YOLO(sign_path)
            self.light_model = YOLO(light_path)
            self.get_logger().info('Load model .pt thành công!')
        except Exception as e:
            self.get_logger().error(f'Lỗi load model: {e}')
            raise

        self.label_pub = self.create_publisher(String,  '/perception/detected_label', 10)
        self.dist_pub  = self.create_publisher(Float32, '/perception/sign_distance',  10)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ✅ Subscribe ảnh gốc
        self.subscription = self.create_subscription(
            CompressedImage,
            '/raw_image/compressed',
            self.listener_callback,
            qos
        )

        self.get_logger().info('TrafficSignNode READY')

    def decode_yolo_ultralytics(self, results, class_map):
        if results is None or len(results) == 0:
            return 'none', 0.0, None

        result = results[0]
        boxes  = result.boxes

        if boxes is None or len(boxes) == 0:
            return 'none', 0.0, None

        confs     = boxes.conf.cpu().numpy()
        best_idx  = int(np.argmax(confs))
        best_conf = float(confs[best_idx])

        if best_conf < CONF_THRESHOLD:
            return 'none', 0.0, None

        best_cls = int(boxes.cls.cpu().numpy()[best_idx])
        label    = class_map.get(best_cls, 'unknown')

        xyxy     = boxes.xyxy.cpu().numpy()[best_idx]
        y_bottom = float(xyxy[3])

        return label, best_conf, y_bottom

    def compute_distance(self, v_pixel: float) -> float:
        alpha_rad = math.radians(self.cam_alpha)
        angle     = alpha_rad + math.atan2(v_pixel - self.cam_v0, self.a_y)
        tan_val   = math.tan(angle)
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

        np_arr = np.frombuffer(msg.data, np.uint8)
        frame  = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        sign_results  = self.sign_model.predict(
            frame, imgsz=self.imgsz, verbose=False, device=self.device)
        light_results = self.light_model.predict(
            frame, imgsz=self.imgsz, verbose=False, device=self.device)

        sign_label,  sign_conf,  sign_v  = self.decode_yolo_ultralytics(sign_results,  SIGN_CLASSES)
        light_label, light_conf, light_v = self.decode_yolo_ultralytics(light_results, LIGHT_CLASSES)

        # Ưu tiên đèn giao thông hơn biển báo
        if light_label != 'none':
            label   = light_label
            conf    = light_conf
            v_pixel = light_v
        elif sign_label != 'none':
            label   = sign_label
            conf    = sign_conf
            v_pixel = sign_v
        else:
            label   = 'none'
            conf    = 0.0
            v_pixel = None

        # Tính khoảng cách thực để log/debug
        if v_pixel is not None:
            distance = self.compute_distance(v_pixel)
        else:
            distance = float('inf')

        # ✅ IMMEDIATE_LABELS → thực thi ngay, distance=0.0
        # Các nhãn khác (den_xanh) → chỉ log, không thực thi
        if label in IMMEDIATE_LABELS:
            effective_distance = 0.0
        elif distance <= DISTANCE_EXECUTE_THRESHOLD:
            effective_distance = 0.0
        else:
            effective_distance = distance

        msg_out      = String()
        msg_out.data = f'{label}:{effective_distance:.3f}'
        self.label_pub.publish(msg_out)

        dist_msg      = Float32()
        dist_msg.data = distance if distance != float('inf') else -1.0
        self.dist_pub.publish(dist_msg)

        if label != 'none':
            dist_str = f'{distance:.2f}m' if distance != float('inf') else '∞'
            exec_str = 'THỰC THI NGAY' if effective_distance == 0.0 else f'chờ ({dist_str})'
            self.get_logger().info(
                f"[PHÁT HIỆN]: {CLASS_VI_MAP.get(label, label)} | "
                f"Tin cậy: {conf*100:.1f}% | "
                f"Khoảng cách: {dist_str} | "
                f"Trạng thái: {exec_str}",
                throttle_duration_sec=0.5
            )


def main(args=None):
    rclpy.init(args=args)
    node = TrafficSignNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
