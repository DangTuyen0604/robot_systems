"""Entry-point helpers shared by the perception nodes."""

import signal

import rclpy
from rclpy.signals import SignalHandlerOptions


def image_age(now, stamp):
    """Return how old ``stamp`` is at ``now`` in seconds, or None if unset."""
    if stamp.nanoseconds == 0:
        return None
    return (now - stamp).nanoseconds / 1e9


def run_node(node_factory, args=None):
    """
    Spin ``node_factory()`` until SIGINT/SIGTERM, then shut down cleanly.

    Ctrl+C reaches every node twice (terminal and ros2 launch). rclpy's own
    handler would shut the context down under the running callbacks and print
    tracebacks; here the first signal just ends the loop and later ones are
    ignored.
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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
