"""Run a node so that its final stop command is really published."""

import signal
import time

import rclpy
from rclpy.signals import SignalHandlerOptions


def spin_then_stop(node_factory, publish_stop, args=None, repeats=3, interval=0.02):
    """
    Spin ``node_factory()`` until SIGINT/SIGTERM, then call ``publish_stop``.

    rclpy's default handler shuts the ROS context down as soon as Ctrl+C
    arrives, so a stop command published afterwards fails with "publisher's
    context is invalid" and never leaves the process. Ctrl+C also reaches every
    node twice (terminal and ros2 launch). Here the first signal only ends the
    spin loop, later ones are ignored, and the stop is published while the
    context is still valid. It is repeated a few times because subscribers
    shutting down at the same moment may miss a single message.
    """
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = []

    def on_signal(signum, frame):
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, signal.SIG_IGN)
        stop_requested.append(signum)

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, on_signal)
    node = node_factory()
    try:
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        try:
            for _ in range(repeats):
                publish_stop(node)
                time.sleep(interval)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
