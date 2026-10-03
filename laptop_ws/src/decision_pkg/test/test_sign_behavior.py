"""Behaviour tests for the three signs on the track."""

import pytest
import rclpy
from std_msgs.msg import String

from decision_pkg.decision_node import TurnBehaviorNode


@pytest.fixture
def node():
    rclpy.init()
    behavior = TurnBehaviorNode()
    yield behavior
    behavior.destroy_node()
    rclpy.shutdown()


def send_label(node, label):
    msg = String()
    msg.data = label
    node.label_callback(msg)


def test_speed_limit_slows_to_sign_speed(node):
    node.lane_speed = 0.32
    send_label(node, 'speed_limit_20:0.000')
    cmd = node.normal_lane_cmd()
    assert cmd.message == 'SLOW'
    assert cmd.speed == pytest.approx(node.sign_slow_speed)
    assert cmd.speed > 0.1


def test_speed_limit_never_overrides_lost_lane_stop(node):
    node.lane_speed = 0.0
    send_label(node, 'speed_limit_20:0.000')
    cmd = node.normal_lane_cmd()
    assert cmd.message == 'SLOW'
    assert cmd.speed == 0.0


def test_stop_sign_stops(node):
    node.lane_speed = 0.32
    send_label(node, 'stop:0.000')
    cmd = node.normal_lane_cmd()
    assert (cmd.message, cmd.speed) == ('STOP', 0.0)


def test_turn_left_arms_left_turn_after_confirmation(node):
    for _ in range(node.sign_confirm_needed):
        send_label(node, 'turn_left:0.000')
    assert node.pending_turn == 'left'


def test_no_right_turn_never_arms_a_turn(node):
    for _ in range(node.sign_confirm_needed * 2):
        send_label(node, 'no_right_turn:0.000')
    assert node.pending_turn is None
