"""Serial packets exchanged with the Arduino motor controller."""

import re

# Protocol of firmware/code_xe/code_xe.ino (one packet per line):
#   V:<pwm 0..255>;E:<pixel error -400..400>   motion, from /control/cmd only
#   KS:<kp>;KSI:<ki>;KSD:<kd>                  steering PID gains
#   P:<kp>;I:<ki>;D:<kd>                       wheel-speed PID gains (USE_SPEED_PID)
_NUMBER = r'[-+]?(?:\d+\.?\d*|\.\d+)'
PID_PACKETS = (
    re.compile(rf'KS:{_NUMBER};KSI:{_NUMBER};KSD:{_NUMBER}'),
    re.compile(rf'P:{_NUMBER};I:{_NUMBER};D:{_NUMBER}'),
)
# The sketch drops any line longer than its 32-character receive buffer.
MAX_PID_PACKET_LENGTH = 32


def motion_packet(speed, steering):
    """Return the motion packet for a bounded ``Control`` command."""
    speed = max(0.0, min(1.0, float(speed)))
    steering = max(-45.0, min(45.0, float(steering)))
    base_v = int(speed * 255)
    error_px = int(steering * (320.0 / 45.0))
    return f'V:{base_v};E:{error_px}\n'


STOP_PACKET = motion_packet(0.0, 0.0)


def pid_rejection_reason(packet, commanded_speed):
    """
    Return why a PID tuning packet must not be sent, or ``None``.

    /arduino/pid writes straight to the serial port, so it must not become a
    second path to the motors around the command mux. Only a single PID-gain
    packet is forwarded, and only while the robot is commanded to stand
    still.
    """
    if len(packet) > MAX_PID_PACKET_LENGTH:
        return f'longer than the {MAX_PID_PACKET_LENGTH}-character firmware buffer'
    if not any(pattern.fullmatch(packet) for pattern in PID_PACKETS):
        return 'not a PID packet KS:<p>;KSI:<i>;KSD:<d> or P:<p>;I:<i>;D:<d>'
    if commanded_speed != 0.0:
        return 'robot is not commanded to stand still'
    return None
