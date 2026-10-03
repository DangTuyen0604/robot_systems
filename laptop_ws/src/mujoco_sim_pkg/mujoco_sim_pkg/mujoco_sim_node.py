#!/usr/bin/env python3
"""
MuJoCo replacement for the Gazebo world, robot plugins and camera.

Interface matches the Gazebo setup used by sim_full_launch.py:
  /cmd_vel (geometry_msgs/Twist) -> differential-drive wheel servos
  /raw_image/compressed (sensor_msgs/CompressedImage) <- front camera, JPEG
  /odom (nav_msgs/Odometry) <- ground-truth pose of base_link
"""

import os
import signal
import time

# Offscreen rendering must not need a window; set before importing mujoco.
os.environ.setdefault('MUJOCO_GL', 'egl')

import cv2  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from rclpy.signals import SignalHandlerOptions  # noqa: E402
from sensor_msgs.msg import CompressedImage  # noqa: E402

from mujoco_sim_pkg.diff_drive import slew, wheel_speeds  # noqa: E402
from mujoco_sim_pkg.scene import load_model  # noqa: E402

CAMERA_NAME = 'front_camera'
# Never simulate more than this per tick, so a stalled process slows the
# simulation down instead of jumping the robot forward.
MAX_CATCH_UP = 0.1


class MujocoSimNode(Node):

    def __init__(self):
        super().__init__('mujoco_sim_node')
        share = get_package_share_directory('mujoco_sim_pkg')
        self.declare_parameter(
            'model_file', os.path.join(share, 'models', 'datn_track.xml'))
        self.declare_parameter('gui', False)
        self.declare_parameter('real_time_factor', 1.0)
        self.declare_parameter('tick_period', 0.01)
        self.declare_parameter('camera_rate', 20.0)
        self.declare_parameter('camera_width', 320)
        self.declare_parameter('camera_height', 240)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('image_topic', '/raw_image/compressed')
        self.declare_parameter('odom_rate', 50.0)
        # Same values as the Gazebo diff-drive plugin (wheel radius from the
        # STL mesh, separation between the URDF wheel joints).
        self.declare_parameter('wheel_radius', 0.034)
        self.declare_parameter('wheel_separation', 0.2133)
        self.declare_parameter('max_wheel_acceleration', 1.0)  # m/s^2
        self.declare_parameter('command_timeout', 0.5)

        def param(name):
            return self.get_parameter(name).value

        self.rtf = float(param('real_time_factor'))
        self.camera_period = 1.0 / float(param('camera_rate'))
        self.odom_period = 1.0 / float(param('odom_rate'))
        self.jpeg_quality = int(param('jpeg_quality'))
        self.wheel_radius = float(param('wheel_radius'))
        self.wheel_separation = float(param('wheel_separation'))
        self.max_wheel_accel = float(param('max_wheel_acceleration'))
        self.command_timeout = float(param('command_timeout'))

        self.model = load_model(
            str(param('model_file')), get_package_share_directory('DATN'))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.left_act = self.model.actuator('left_motor').id
        self.right_act = self.model.actuator('right_motor').id
        self.renderer = mujoco.Renderer(
            self.model, int(param('camera_height')), int(param('camera_width')))

        self.viewer = None
        if bool(param('gui')):
            from mujoco import viewer
            self.viewer = viewer.launch_passive(self.model, self.data)

        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.last_cmd_wall = None
        self.wheel_lin = [0.0, 0.0]  # wheel rim speeds (m/s) after slew limit
        self.next_camera = 0.0
        self.next_odom = 0.0
        self.start_wall = time.monotonic()

        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.image_pub = self.create_publisher(
            CompressedImage, str(param('image_topic')), image_qos)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.create_timer(float(param('tick_period')), self._tick)
        self.get_logger().info(
            f'MuJoCo {mujoco.__version__} READY: /cmd_vel -> wheels, '
            f'{CAMERA_NAME} -> {param("image_topic")}')

    def _cmd_cb(self, msg):
        self.cmd_linear = float(msg.linear.x)
        self.cmd_angular = float(msg.angular.z)
        self.last_cmd_wall = time.monotonic()

    def _target_wheel_lin(self):
        if (self.last_cmd_wall is None or
                time.monotonic() - self.last_cmd_wall > self.command_timeout):
            return 0.0, 0.0
        # Radius 1.0 gives rim speeds in m/s, the unit of the acceleration limit.
        return wheel_speeds(
            self.cmd_linear, self.cmd_angular, self.wheel_separation, 1.0)

    def _tick(self):
        if self.viewer is not None and not self.viewer.is_running():
            self.get_logger().info('MuJoCo viewer closed; simulation continues headless')
            self.viewer = None

        target_time = (time.monotonic() - self.start_wall) * self.rtf
        target_time = min(target_time, self.data.time + MAX_CATCH_UP)
        target = self._target_wheel_lin()
        dt = self.model.opt.timestep
        max_delta = self.max_wheel_accel * dt

        while self.data.time < target_time:
            for i in (0, 1):
                self.wheel_lin[i] = slew(self.wheel_lin[i], target[i], max_delta)
            self.data.ctrl[self.left_act] = self.wheel_lin[0] / self.wheel_radius
            self.data.ctrl[self.right_act] = self.wheel_lin[1] / self.wheel_radius
            mujoco.mj_step(self.model, self.data)

            if self.data.time >= self.next_camera:
                self.next_camera += self.camera_period
                self._publish_camera()
            if self.data.time >= self.next_odom:
                self.next_odom += self.odom_period
                self._publish_odom()

        if self.viewer is not None:
            self.viewer.sync()

    def _publish_camera(self):
        self.renderer.update_scene(self.data, CAMERA_NAME)
        rgb = self.renderer.render()
        ok, jpeg = cv2.imencode(
            '.jpg', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = CAMERA_NAME
        msg.format = 'jpeg'
        msg.data = jpeg.tobytes()
        self.image_pub.publish(msg)

    def _publish_odom(self):
        qpos = self.data.joint('root').qpos
        qvel = self.data.joint('root').qvel
        rotation = np.zeros(9)
        mujoco.mju_quat2Mat(rotation, qpos[3:7])
        # Free-joint linear velocity is in the world frame; Odometry twist is
        # expressed in the child frame (base_link).
        linear_body = rotation.reshape(3, 3).T @ qvel[0:3]

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = float(qpos[0])
        msg.pose.pose.position.y = float(qpos[1])
        msg.pose.pose.position.z = float(qpos[2])
        msg.pose.pose.orientation.w = float(qpos[3])
        msg.pose.pose.orientation.x = float(qpos[4])
        msg.pose.pose.orientation.y = float(qpos[5])
        msg.pose.pose.orientation.z = float(qpos[6])
        msg.twist.twist.linear.x = float(linear_body[0])
        msg.twist.twist.linear.y = float(linear_body[1])
        msg.twist.twist.linear.z = float(linear_body[2])
        msg.twist.twist.angular.x = float(qvel[3])
        msg.twist.twist.angular.y = float(qvel[4])
        msg.twist.twist.angular.z = float(qvel[5])
        self.odom_pub.publish(msg)

    def close(self):
        # The viewer must close first: closing the EGL renderer while the
        # viewer is open crashes the process.
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None
        self.renderer.close()


def main(args=None):
    # Ctrl+C reaches this process twice: from the terminal and again from
    # ros2 launch. Handle the first one and ignore the rest, so the GL
    # cleanup in close() is never interrupted halfway.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = []

    def on_sigint(signum, frame):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        stop_requested.append(signum)

    signal.signal(signal.SIGINT, on_sigint)
    node = MujocoSimNode()
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
