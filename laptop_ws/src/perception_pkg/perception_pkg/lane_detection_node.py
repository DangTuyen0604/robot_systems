#!/usr/bin/env python3
"""ROS wrapper of LaneDetector: camera image in, lane-following suggestion out."""

import cv2
from interfaces_pkg.msg import Control
import numpy as np
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from perception_pkg.lane_detector import (
    draw_overlay, LaneDetector, LaneParams, MODE_NONE)
from perception_pkg.node_runner import image_age, run_node


class LaneFollowerNode(Node):

    def __init__(self):
        super().__init__('lane_node_instance')

        defaults = LaneParams()
        values = {}
        for name in LaneParams.names():
            self.declare_parameter(name, getattr(defaults, name))
            values[name] = self.get_parameter(name).value
        values['speed_turn_thresholds'] = [float(v) for v in values['speed_turn_thresholds']]
        values['speed_turn_values'] = [float(v) for v in values['speed_turn_values']]
        self.detector = LaneDetector(LaneParams(**values))

        # Safe by default: a real robot stops when the lane is lost. The
        # simulation may enable a fixed search turn at its T-junction, where
        # the straight lane deliberately ends before the next branch appears.
        self.declare_parameter('lost_lane_recovery_enabled', False)
        self.declare_parameter('lost_lane_recovery_frames', 10)
        self.declare_parameter('lost_lane_recovery_max_frames', 30)
        self.declare_parameter('lost_lane_recovery_steering', -25.0)
        self.declare_parameter('lost_lane_recovery_speed', 0.22)
        # Images older than this (s, by their capture stamp) are dropped, so a
        # Wi-Fi stall cannot drive the robot on stale frames; the decision
        # node's lane watchdog then stops it. 0 disables the check. Needs the
        # camera and laptop clocks synchronised (see ENVIRONMENT.md).
        self.declare_parameter('max_image_age', 0.5)

        def param(name):
            return self.get_parameter(name).value

        self.recovery_enabled = bool(param('lost_lane_recovery_enabled'))
        self.recovery_frames = int(param('lost_lane_recovery_frames'))
        self.recovery_max_frames = int(param('lost_lane_recovery_max_frames'))
        self.recovery_steering = float(param('lost_lane_recovery_steering'))
        self.recovery_speed = float(param('lost_lane_recovery_speed'))
        self.max_image_age = float(param('max_image_age'))

        self.no_lane_count = 0
        self.current_sign = 'none'

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(
            CompressedImage, '/raw_image/compressed', self.image_callback, qos)
        self.create_subscription(
            String, '/perception/detected_label', self.sign_callback, 10)
        self.control_pub = self.create_publisher(Control, '/control/lane_suggest', 10)
        self.image_pub = self.create_publisher(
            CompressedImage, '/perception/lane_processed/compressed', qos)

        self.get_logger().info('LANE FOLLOWER (P-controller) READY')

    def sign_callback(self, msg: String):
        raw = msg.data.strip()
        label = raw.split(':')[0].strip().lower() if raw else 'none'
        self.current_sign = label or 'none'

    def image_callback(self, msg):
        if self.max_image_age > 0.0:
            age = image_age(self.get_clock().now(), Time.from_msg(msg.header.stamp))
            if age is not None and not -self.max_image_age <= age <= self.max_image_age:
                self.get_logger().error(
                    f'Dropping camera image {age:+.2f} s old (limit {self.max_image_age} s). '
                    'Negative age means the camera clock is ahead: synchronise clocks.',
                    throttle_duration_sec=2.0)
                return

        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        p = self.detector.p
        frame = cv2.resize(frame, (p.width, p.height))
        result = self.detector.detect(frame)
        steering, speed = self.compute_control(result)

        cmd = Control()
        cmd.steering = float(steering)
        cmd.speed = float(speed)
        # The mode lets the decision node tell lane following from the end of
        # the lane at a junction (NO LANE).
        cmd.message = result.mode
        self.control_pub.publish(cmd)

        # Drawing and JPEG encoding cost CPU; only do it when someone watches.
        if self.image_pub.get_subscription_count() > 0:
            debug = draw_overlay(frame, result, self.detector.roi_y, self.current_sign)
            ok, encoded = cv2.imencode('.jpg', debug)
            if ok:
                out = CompressedImage()
                out.header.stamp = msg.header.stamp
                out.format = 'jpeg'
                out.data = encoded.tobytes()
                self.image_pub.publish(out)

    def compute_control(self, result):
        if result.mode == MODE_NONE:
            self.no_lane_count += 1
        else:
            self.no_lane_count = 0

        if self.no_lane_count >= self.recovery_frames:
            if self.recovery_enabled and self.no_lane_count <= self.recovery_max_frames:
                self.get_logger().warn(
                    f'Lane lost ({self.no_lane_count} frames): search turn '
                    f'{self.recovery_steering:+.0f} deg',
                    throttle_duration_sec=1.0)
                return self.recovery_steering, self.recovery_speed
            self.get_logger().warn(
                f'Lane lost ({self.no_lane_count} frames): stop',
                throttle_duration_sec=1.0)
            return 0.0, 0.0

        steering, speed = self.detector.control(result.lane_center)
        self.get_logger().info(
            f'{result.mode}: left={result.left_x} right={result.right_x} '
            f'center={result.lane_center} steering={steering:.1f} deg speed={speed:.2f}',
            throttle_duration_sec=1.0)
        return steering, speed


def main(args=None):
    run_node(LaneFollowerNode, args=args)


if __name__ == '__main__':
    main()
