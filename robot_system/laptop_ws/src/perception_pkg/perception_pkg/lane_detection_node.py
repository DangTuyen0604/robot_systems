#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge

from interfaces_pkg.msg import Control

import cv2
import numpy as np
import time

from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy


class LaneFollowerNode(Node):

    def __init__(self):
        super().__init__('lane_node_instance')

        self.bridge = CvBridge()
        self.frame_count = 0

        self.width = 640
        self.height = 480

        self.roi_y = int(self.height * 0.55)
        self.lookahead_y = int(self.height * 0.70)
        self.prev_lane_center = self.width // 2

        self.lane_left_x = None
        self.lane_right_x = None
        self.tracked_left_x = int(self.width * 0.22)
        self.tracked_right_x = int(self.width * 0.64)
        self.tracked_lane_width = 566   # gia tri khoi tao, se tu dong hieu chinh khi thay ca 2 vach

        self.prev_fps_time = time.time()
        self.fps = 0.0
        self.no_lane_count = 0

        self.current_sign = 'none'
        self.current_light = 'none'

        self.L_pixels = 350
        self.ld_default = 100

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub = self.create_subscription(
            CompressedImage,
            '/raw_image/compressed',
            self.image_callback,
            qos
        )

        self.create_subscription(
            String,
            '/perception/detected_label',
            self.sign_callback,
            10
        )

        self.control_pub = self.create_publisher(
            Control,
            '/control/lane_suggest',
            10
        )

        # Đổi từ Image sang CompressedImage để giảm tải băng thông truyền nhận
        self.image_pub = self.create_publisher(
            CompressedImage,
            '/perception/lane_processed/compressed',
            qos
        )

        self.get_logger().info('PURE PURSUIT LANE FOLLOWER READY')

    def sign_callback(self, msg: String):
        raw = msg.data.strip()
        label = raw.split(':')[0].strip().lower() if raw else 'none'
        if label in ('den_do', 'den_vang', 'den_xanh'):
            self.current_light = label
            self.current_sign = 'none'
        elif label and label != 'none':
            self.current_sign = label
            self.current_light = 'none'
        else:
            self.current_sign = 'none'
            self.current_light = 'none'

    def image_callback(self, msg):
        self.frame_count += 1
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        frame = cv2.resize(frame, (self.width, self.height))
        result, error, mode, lane_center = self.detect_lane(frame)
        steering, speed = self.compute_control(lane_center, mode)

        cmd = Control()
        cmd.steering = float(steering)
        cmd.speed = float(speed)
        self.control_pub.publish(cmd)

        # Nén ảnh kết quả thành JPEG trước khi publish lên ROS 2
        if self.frame_count % 3 == 0:
            msg_compressed = CompressedImage()
            msg_compressed.header.stamp = self.get_clock().now().to_msg()
            msg_compressed.format = "jpeg"
            
            success, encoded_image = cv2.imencode('.jpg', result)
            if success:
                msg_compressed.data = encoded_image.tobytes()
                self.image_pub.publish(msg_compressed)

    def _build_binary_lane(self, frame):
        # 1. Chuyển sang ảnh xám (Grayscale)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 2. Tạo mặt nạ vùng quan tâm (ROI)
        mask_roi = np.zeros_like(gray)
        mask_roi[self.roi_y:, :] = 255
        
        # Chỉ lấy vùng ảnh xám nằm trong ROI
        gray_roi = cv2.bitwise_and(gray, mask_roi)
        
        # 3. Làm mờ để giảm nhiễu
        blur = cv2.medianBlur(gray_roi, 5)
        
        # 4. Phân ngưỡng nhị phân (vạch đen trên nền sáng)
        # Vì vạch màu đen (giá trị pixel thấp), ta sẽ lấy các pixel có độ sáng nhỏ hơn 80 (hoặc 100)
        # Bạn có thể điều chỉnh số 100 này (tăng lên nếu vạch chưa ăn, giảm xuống nếu bị lem nền)
        _, binary = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY_INV)
        
        # Chỉ giữ lại phần nhị phân bên trong ROI
        binary = cv2.bitwise_and(binary, mask_roi)

        # 5. Xử lý hình thái học để làm mịn vạch
        kernel = np.ones((5, 5), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

        # 6. Lọc contours theo diện tích để loại bỏ nhiễu nhỏ
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        filtered = np.zeros_like(binary)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if 200 < area < 12000: # Nới rộng diện tích tối đa lên một chút
                cv2.drawContours(filtered, [cnt], -1, 255, -1)
        return filtered

    def _find_all_blobs(self, scan_line, margin=30):
        region = scan_line[margin:self.width - margin]
        padded = np.concatenate([[0], (region > 0).astype(np.int32), [0]])
        diff = np.diff(padded)
        starts = np.where(diff > 0)[0] + margin
        ends = np.where(diff < 0)[0] + margin
        blobs = []
        for s, e in zip(starts, ends):
            if e - s >= 5:
                blobs.append(int((s + e) // 2))
        return blobs

    def detect_lane(self, frame):
        output = frame.copy()
        mid = self.width // 2

        roi_overlay = output.copy()
        cv2.rectangle(roi_overlay, (0, self.roi_y), (self.width, self.height), (0, 255, 0), -1)
        output = cv2.addWeighted(roi_overlay, 0.12, output, 0.88, 0)
        cv2.line(output, (0, self.roi_y), (self.width, self.roi_y), (0, 255, 0), 2)

        binary_lane = self._build_binary_lane(frame)
        lane_vis = np.zeros_like(output)
        lane_vis[binary_lane > 0] = [0, 0, 255]
        output = cv2.addWeighted(output, 1.0, lane_vis, 0.45, 0)

        scan_y = max(self.lookahead_y, self.roi_y + 20)
        scan_line = binary_lane[scan_y, :]
        blobs = self._find_all_blobs(scan_line, 30)

        left_found = False
        left_x = None
        right_found = False
        right_x = None

        if len(blobs) == 1:
            blob = blobs[0]
            dist_l = abs(blob - self.tracked_left_x)
            dist_r = abs(blob - self.tracked_right_x)
            if dist_l <= dist_r:
                left_found = True
                left_x = blob
                self.tracked_left_x = int(0.7 * self.tracked_left_x + 0.3 * blob)
            else:
                right_found = True
                right_x = blob
                self.tracked_right_x = int(0.7 * self.tracked_right_x + 0.3 * blob)
        elif len(blobs) >= 2:
            blobs.sort()
            left_x = blobs[0]
            right_x = blobs[-1]
            left_found = True
            right_found = True
            self.tracked_left_x = int(0.7 * self.tracked_left_x + 0.3 * left_x)
            self.tracked_right_x = int(0.7 * self.tracked_right_x + 0.3 * right_x)

        self.lane_left_x = left_x
        self.lane_right_x = right_x

        if left_found and right_found:
            lane_center = (left_x + right_x) // 2
            mode = 'BOTH LANES'
            # Cap nhat dong lane_width tu du lieu thuc te (perspective thay doi theo khoang cach)
            measured_width = right_x - left_x
            if 50 < measured_width < 620:
                self.tracked_lane_width = int(0.7 * self.tracked_lane_width + 0.3 * measured_width)
        elif left_found and not right_found:
            lane_center = left_x + self.tracked_lane_width // 2
            mode = 'LEFT ONLY -> STEER RIGHT'
        elif right_found and not left_found:
            lane_center = right_x - self.tracked_lane_width // 2
            mode = 'RIGHT ONLY -> STEER LEFT'
        else:
            lane_center = self.prev_lane_center
            mode = 'NO LANE'

        lane_center = int(np.clip(lane_center, 0, self.width - 1))

        # Gioi han buoc nhay toi da moi frame (chong dao dong/giat cuc do uoc luong nhieu)
        MAX_JUMP = 60
        delta = lane_center - self.prev_lane_center
        if delta > MAX_JUMP:
            lane_center = self.prev_lane_center + MAX_JUMP
        elif delta < -MAX_JUMP:
            lane_center = self.prev_lane_center - MAX_JUMP

        # Khi chi thay 1 ben (uoc luong kem tin cay hon), tin vao gia tri cu nhieu hon
        if mode in ('BOTH LANES',):
            blend = 0.6
        else:
            blend = 0.25   # single-side hoac NO LANE -> it tin tuong hon vao gia tri moi

        lane_center = int((1 - blend) * self.prev_lane_center + blend * lane_center)
        self.prev_lane_center = lane_center

        image_center = self.width // 2
        error = image_center - lane_center

        cv2.line(output, (image_center, 0), (image_center, self.height), (0, 255, 255), 2)
        cv2.line(output, (mid, self.roi_y), (mid, self.height), (255, 255, 0), 1)
        cv2.line(output, (0, scan_y), (self.width, scan_y), (255, 255, 0), 2)

        if left_found:
            cv2.circle(output, (left_x, scan_y), 12, (255, 0, 0), -1)
            cv2.putText(output, 'L', (left_x - 6, scan_y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        if right_found:
            cv2.circle(output, (right_x, scan_y), 12, (0, 0, 255), -1)
            cv2.putText(output, 'R', (right_x - 6, scan_y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.circle(output, (lane_center, scan_y), 16, (0, 255, 0), -1)
        cv2.circle(output, (lane_center, scan_y), 20, (255, 255, 255), 2)
        cv2.line(output, (image_center, scan_y), (lane_center, scan_y), (0, 255, 255), 3)

        cv2.rectangle(output, (0, 0), (380, 195), (0, 0, 0), -1)
        sign_color = (0, 255, 255) if self.current_sign != 'none' else (255, 255, 255)
        light_color = (0, 255, 0) if self.current_light != 'none' else (255, 255, 255)

        hud_lines = [
            ((f'Mode: {mode}', (10, 30)), (255, 255, 255)),
            ((f'Error: {error:.1f} px', (10, 60)), (255, 255, 255)),
            ((f'Left X: {left_x if left_found else "-"}', (10, 90)), (255, 255, 255)),
            ((f'Right X: {right_x if right_found else "-"}', (10, 120)), (255, 255, 255)),
            ((f'Sign: {self.current_sign}', (10, 150)), sign_color),
            ((f'Light: {self.current_light}', (10, 180)), light_color),
        ]
        for (text, pos), color in hud_lines:
            cv2.putText(output, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        self.get_logger().info(
            f"left={'Y' if left_found else 'N'} right={'Y' if right_found else 'N'} "
            f"left_x={left_x} right_x={right_x} lane_center={lane_center} error={error:.1f}",
            throttle_duration_sec=1.0
        )

        return output, error, mode, lane_center

    KP_STEERING = 0.05

    def compute_control(self, lane_center, mode):
        if mode == 'NO LANE':
            self.no_lane_count += 1
        else:
            self.no_lane_count = 0

        if self.no_lane_count >= 10:
            self.get_logger().warn(
                f'KHONG PHAT HIEN LANE ({self.no_lane_count} frame) -> DUNG LAI',
                throttle_duration_sec=1.0
            )
            return 0.0, 0.0

        # LƯU Ý Ở ĐÂY:
        # Nếu error > 0 (vạch lệch bên phải xe), xe cần bẻ sang phải.
        # Nếu xe bạn quy ước bẻ phải là góc LÁI ÂM (-) thì công thức gốc phải thêm dấu TRỪ (-).
        error = lane_center - (self.width // 2)
        
        # SỬA DÒNG NÀY: Thêm dấu trừ (-) vào trước phép tính steering để đảo hướng lái
        steering = float(np.clip(error * self.KP_STEERING, -45.0, 45.0))

        abs_steer = abs(steering)
        if abs_steer > 25:
            speed = 0.18
        elif abs_steer > 15:
            speed = 0.22
        elif abs_steer > 8:
            speed = 0.28
        else:
            speed = 0.32

        self.get_logger().info(
            f'P-ctrl error={error:.1f}px steering={steering:.1f}° speed={speed:.2f}',
            throttle_duration_sec=1.0
        )
        return steering, speed


def main():
    rclpy.init()
    node = LaneFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('He thong dang dung boi nguoi dung (Ctrl+C)...')
    finally:
        # Ngăn chặn lỗi RCLError do shutdown trùng lặp
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()