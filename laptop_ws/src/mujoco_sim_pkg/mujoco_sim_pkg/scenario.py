"""Run the full MuJoCo stack headless and record the robot trajectory."""

import math
import os
import random
import signal
import subprocess
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor

from mujoco_sim_pkg.evaluation import end_reached, Sample


def free_domain_id():
    """Return a ROS domain id unlikely to collide with a running system."""
    return random.randint(100, 200)


def run_scenario(max_duration, signs=True, domain_id=None, log_path=None):
    """
    Launch mujoco_full_launch.py and return the /odom samples.

    The run ends when the robot has stopped at the end of the route, or after
    ``max_duration`` seconds, so a slow machine (software rendering in CI)
    only makes it take longer. The stack runs in its own process group and
    ROS domain, and is stopped with SIGINT like Ctrl+C in a terminal.
    """
    domain_id = free_domain_id() if domain_id is None else domain_id
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain_id))
    env.setdefault('MUJOCO_GL', 'egl')
    cmd = ['ros2', 'launch', 'mujoco_sim_pkg', 'mujoco_full_launch.py',
           'gui:=false', f'signs:={"true" if signs else "false"}']
    log = open(log_path, 'w') if log_path else subprocess.DEVNULL
    proc = subprocess.Popen(
        cmd, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    samples = []
    context = Context()
    rclpy.init(context=context, domain_id=domain_id)
    node = rclpy.create_node('scenario_recorder', context=context)
    t0 = time.monotonic()

    def on_odom(msg):
        pose = msg.pose.pose
        yaw = 2.0 * math.atan2(pose.orientation.z, pose.orientation.w)
        samples.append(Sample(
            t=time.monotonic() - t0, x=pose.position.x, y=pose.position.y,
            yaw=yaw, speed=msg.twist.twist.linear.x))

    node.create_subscription(Odometry, '/odom', on_odom, 50)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    try:
        while time.monotonic() - t0 < max_duration and proc.poll() is None:
            executor.spin_once(timeout_sec=0.05)
            if samples and end_reached(samples):
                break
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
        if log_path:
            log.close()
    return samples
