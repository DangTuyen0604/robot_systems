#!/usr/bin/env python3
"""
Measure wheel speed (encoder pulses/s) against PWM from /arduino/telemetry.

Passive: it only listens. Drive the robot (teleop, or the mux in MANUAL) at a
few steady speeds, ideally with the wheels lifted, then stop with Ctrl+C to
print pulses per second for each PWM value seen:

    ros2 run control_pkg encoder_rate
"""

from collections import defaultdict
import re
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TELEMETRY = re.compile(
    r'ERR:(?P<err>-?\d+),D:(?P<d>-?[\d.]+),PL:(?P<pl>\d+),PR:(?P<pr>\d+),'
    r'EL:(?P<el>-?\d+),ER:(?P<er>-?\d+)')


def parse_telemetry(line):
    """Return ``(pwm_left, pwm_right, count_left, count_right)`` or None."""
    match = TELEMETRY.fullmatch(line.strip())
    if not match:
        return None
    return (int(match['pl']), int(match['pr']), int(match['el']), int(match['er']))


class RateTable:
    """Accumulate pulses and time per PWM value for each wheel."""

    def __init__(self):
        self.pulses = {'left': defaultdict(int), 'right': defaultdict(int)}
        self.seconds = {'left': defaultdict(float), 'right': defaultdict(float)}
        self.prev = None

    def add(self, t, sample):
        pwm_l, pwm_r, count_l, count_r = sample
        if self.prev is not None:
            prev_t, prev_l, prev_r = self.prev
            dt = t - prev_t
            # Skip gaps (robot stopped, telemetry paused) and counter resets.
            if 0.0 < dt < 0.5 and count_l >= prev_l and count_r >= prev_r:
                self.pulses['left'][pwm_l] += count_l - prev_l
                self.seconds['left'][pwm_l] += dt
                self.pulses['right'][pwm_r] += count_r - prev_r
                self.seconds['right'][pwm_r] += dt
        self.prev = (t, count_l, count_r)

    def rows(self, min_seconds=1.0):
        out = []
        for wheel in ('left', 'right'):
            for pwm in sorted(self.seconds[wheel]):
                seconds = self.seconds[wheel][pwm]
                if seconds >= min_seconds:
                    out.append((wheel, pwm, self.pulses[wheel][pwm] / seconds, seconds))
        return out


class EncoderRateNode(Node):

    def __init__(self):
        super().__init__('encoder_rate')
        self.table = RateTable()
        self.create_subscription(String, '/arduino/telemetry', self.on_line, 50)
        self.get_logger().info('Listening on /arduino/telemetry; Ctrl+C prints the table')

    def on_line(self, msg):
        sample = parse_telemetry(msg.data)
        if sample:
            self.table.add(time.monotonic(), sample)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderRateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    print('\n| Wheel | PWM | Pulses/s | Measured for (s) |\n|---|---|---|---|')
    for wheel, pwm, rate, seconds in node.table.rows():
        print(f'| {wheel} | {pwm} | {rate:.1f} | {seconds:.1f} |')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
