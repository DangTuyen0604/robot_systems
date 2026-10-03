"""Unit tests for differential-drive helpers."""

import pytest

from mujoco_sim_pkg.diff_drive import slew, wheel_speeds


def test_straight_and_spin_in_place():
    assert wheel_speeds(0.3, 0.0, 0.2, 0.05) == pytest.approx((6.0, 6.0))
    left, right = wheel_speeds(0.0, 1.0, 0.2, 0.05)
    assert left == pytest.approx(-2.0)
    assert right == pytest.approx(2.0)


def test_positive_angular_turns_left():
    left, right = wheel_speeds(0.2, 0.5, 0.2133, 0.034)
    assert right > left


def test_slew_limits_each_step():
    assert slew(0.0, 1.0, 0.1) == pytest.approx(0.1)
    assert slew(0.0, -1.0, 0.1) == pytest.approx(-0.1)
    assert slew(0.95, 1.0, 0.1) == pytest.approx(1.0)
