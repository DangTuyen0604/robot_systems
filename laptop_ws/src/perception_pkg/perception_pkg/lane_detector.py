"""
Black-tape lane detection and steering, independent of ROS.

The image is resized to ``width x height``. Pixels darker than ``threshold``
inside the lower ROI become lane candidates; blobs on one scan line give the
left/right tape positions, which are tracked over frames. A proportional
controller turns the lane-centre offset into a steering angle (degrees,
positive = right) and a speed that drops in sharp turns.
"""

from dataclasses import dataclass, field, fields

import cv2
import numpy as np

MODE_BOTH = 'BOTH LANES'
MODE_LEFT = 'LEFT ONLY -> STEER RIGHT'
MODE_RIGHT = 'RIGHT ONLY -> STEER LEFT'
MODE_NONE = 'NO LANE'


@dataclass
class LaneParams:
    width: int = 640
    height: int = 480
    roi_ratio: float = 0.55          # ROI starts at this fraction of the height
    lookahead_ratio: float = 0.70    # scan line, fraction of the height
    threshold: int = 100             # grey level below which a pixel is tape
    blur_kernel: int = 5
    morph_kernel: int = 5
    min_contour_area: float = 200.0
    max_contour_area: float = 12000.0
    scan_margin: int = 30            # ignore this many pixels at each image edge
    min_blob_width: int = 5
    initial_lane_width: int = 566    # px at the scan line, refined when both tapes show
    min_lane_width: int = 50
    max_lane_width: int = 620
    tracking_gain: float = 0.3       # weight of a new tape position
    max_center_jump: int = 60        # px per frame
    blend_both: float = 0.6          # weight of the new centre when both tapes show
    blend_single: float = 0.25       # ... when one or no tape shows
    kp_steering: float = 0.05        # degrees per pixel of centre offset
    max_steering: float = 45.0
    # Speed falls as |steering| exceeds each threshold (degrees).
    speed_straight: float = 0.32
    speed_turn_thresholds: list = field(default_factory=lambda: [8.0, 15.0, 25.0])
    speed_turn_values: list = field(default_factory=lambda: [0.28, 0.22, 0.18])

    @classmethod
    def names(cls):
        return [f.name for f in fields(cls)]


@dataclass
class LaneResult:
    mode: str
    lane_center: int
    left_x: object = None
    right_x: object = None
    scan_y: int = 0
    binary: object = None


class LaneDetector:

    def __init__(self, params=None):
        self.p = params or LaneParams()
        p = self.p
        self.roi_y = int(p.height * p.roi_ratio)
        self.scan_y = max(int(p.height * p.lookahead_ratio), self.roi_y + 20)
        self.prev_lane_center = p.width // 2
        self.tracked_left_x = int(p.width * 0.22)
        self.tracked_right_x = int(p.width * 0.64)
        self.tracked_lane_width = p.initial_lane_width
        self.kernel = np.ones((p.morph_kernel, p.morph_kernel), np.uint8)
        self.roi_mask = np.zeros((p.height, p.width), np.uint8)
        self.roi_mask[self.roi_y:, :] = 255

    def binary_lane(self, frame):
        """Return the mask of tape-coloured blobs inside the ROI."""
        p = self.p
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_roi = cv2.bitwise_and(gray, self.roi_mask)
        blur = cv2.medianBlur(gray_roi, p.blur_kernel)
        # Dark tape on a light floor.
        _, binary = cv2.threshold(blur, p.threshold, 255, cv2.THRESH_BINARY_INV)
        binary = cv2.bitwise_and(binary, self.roi_mask)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, self.kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, self.kernel)

        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        filtered = np.zeros_like(binary)
        for cnt in contours:
            if p.min_contour_area < cv2.contourArea(cnt) < p.max_contour_area:
                cv2.drawContours(filtered, [cnt], -1, 255, -1)
        return filtered

    def blobs(self, scan_line):
        """Return the centres of tape blobs crossing one image row."""
        p = self.p
        margin = p.scan_margin
        region = scan_line[margin:p.width - margin]
        padded = np.concatenate([[0], (region > 0).astype(np.int32), [0]])
        diff = np.diff(padded)
        starts = np.where(diff > 0)[0] + margin
        ends = np.where(diff < 0)[0] + margin
        return [int((s + e) // 2) for s, e in zip(starts, ends) if e - s >= p.min_blob_width]

    def detect(self, frame):
        """Locate the lane in a BGR frame of size ``width x height``."""
        p = self.p
        gain = p.tracking_gain
        binary = self.binary_lane(frame)
        found = self.blobs(binary[self.scan_y, :])

        left_x = right_x = None
        if len(found) == 1:
            blob = found[0]
            if abs(blob - self.tracked_left_x) <= abs(blob - self.tracked_right_x):
                left_x = blob
                self.tracked_left_x = int((1 - gain) * self.tracked_left_x + gain * blob)
            else:
                right_x = blob
                self.tracked_right_x = int((1 - gain) * self.tracked_right_x + gain * blob)
        elif len(found) >= 2:
            found.sort()
            left_x, right_x = found[0], found[-1]
            self.tracked_left_x = int((1 - gain) * self.tracked_left_x + gain * left_x)
            self.tracked_right_x = int((1 - gain) * self.tracked_right_x + gain * right_x)

        if left_x is not None and right_x is not None:
            mode = MODE_BOTH
            lane_center = (left_x + right_x) // 2
            # Perspective changes the apparent width; follow the measurement.
            measured_width = right_x - left_x
            if p.min_lane_width < measured_width < p.max_lane_width:
                self.tracked_lane_width = int(
                    (1 - gain) * self.tracked_lane_width + gain * measured_width)
        elif left_x is not None:
            mode = MODE_LEFT
            lane_center = left_x + self.tracked_lane_width // 2
        elif right_x is not None:
            mode = MODE_RIGHT
            lane_center = right_x - self.tracked_lane_width // 2
        else:
            mode = MODE_NONE
            lane_center = self.prev_lane_center

        lane_center = int(np.clip(lane_center, 0, p.width - 1))
        # Limit the per-frame jump, then low-pass; trust single-tape estimates less.
        delta = lane_center - self.prev_lane_center
        delta = max(-p.max_center_jump, min(p.max_center_jump, delta))
        lane_center = self.prev_lane_center + delta
        blend = p.blend_both if mode == MODE_BOTH else p.blend_single
        lane_center = int((1 - blend) * self.prev_lane_center + blend * lane_center)
        self.prev_lane_center = lane_center

        return LaneResult(mode, lane_center, left_x, right_x, self.scan_y, binary)

    def control(self, lane_center):
        """Return ``(steering_deg, speed)`` for a lane centre in pixels."""
        p = self.p
        error = lane_center - p.width // 2
        steering = float(np.clip(error * p.kp_steering, -p.max_steering, p.max_steering))
        speed = p.speed_straight
        for limit, value in zip(p.speed_turn_thresholds, p.speed_turn_values):
            if abs(steering) > limit:
                speed = value
        return steering, speed


def draw_overlay(frame, result, roi_y, sign_label):
    """Return a debug image showing the ROI, tape mask, tapes and lane centre."""
    height, width = frame.shape[:2]
    output = frame.copy()
    roi_overlay = output.copy()
    cv2.rectangle(roi_overlay, (0, roi_y), (width, height), (0, 255, 0), -1)
    output = cv2.addWeighted(roi_overlay, 0.12, output, 0.88, 0)
    cv2.line(output, (0, roi_y), (width, roi_y), (0, 255, 0), 2)

    lane_vis = np.zeros_like(output)
    lane_vis[result.binary > 0] = [0, 0, 255]
    output = cv2.addWeighted(output, 1.0, lane_vis, 0.45, 0)

    mid = width // 2
    scan_y = result.scan_y
    cv2.line(output, (mid, 0), (mid, height), (0, 255, 255), 2)
    cv2.line(output, (0, scan_y), (width, scan_y), (255, 255, 0), 2)
    font = cv2.FONT_HERSHEY_SIMPLEX
    for x, text, color in ((result.left_x, 'L', (255, 0, 0)),
                           (result.right_x, 'R', (0, 0, 255))):
        if x is not None:
            cv2.circle(output, (x, scan_y), 12, color, -1)
            cv2.putText(output, text, (x - 6, scan_y + 5), font, 0.5, (255, 255, 255), 1)
    cv2.circle(output, (result.lane_center, scan_y), 16, (0, 255, 0), -1)
    cv2.circle(output, (result.lane_center, scan_y), 20, (255, 255, 255), 2)
    cv2.line(output, (mid, scan_y), (result.lane_center, scan_y), (0, 255, 255), 3)

    cv2.rectangle(output, (0, 0), (380, 165), (0, 0, 0), -1)
    white = (255, 255, 255)
    hud = [
        (f'Mode: {result.mode}', white),
        (f'Error: {mid - result.lane_center:.1f} px', white),
        (f'Left X: {"-" if result.left_x is None else result.left_x}', white),
        (f'Right X: {"-" if result.right_x is None else result.right_x}', white),
        (f'Sign: {sign_label}', (0, 255, 255) if sign_label != 'none' else white),
    ]
    for row, (text, color) in enumerate(hud):
        cv2.putText(output, text, (10, 30 + 30 * row), font, 0.6, color, 2)
    return output
