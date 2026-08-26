"""Unit tests for simulation control safety logic."""

import pytest

from decision_pkg.command_mux_node import select_command
from decision_pkg.sim_motor_bridge_node import control_to_velocity


def test_estop_never_forwards_a_command():
    command, status = select_command(
        'ESTOP', {'AUTO': object()}, {'AUTO': 10.0}, 10.0, 0.3)
    assert command is None
    assert status == 'ESTOP'


def test_mux_forwards_only_fresh_selected_command():
    auto = object()
    manual = object()
    command, status = select_command(
        'AUTO', {'AUTO': auto, 'MANUAL': manual},
        {'AUTO': 9.8, 'MANUAL': 10.0}, 10.0, 0.3)
    assert command is auto
    assert status == 'AUTO'


def test_mux_stops_on_stale_or_missing_command():
    command, status = select_command(
        'AUTO', {'AUTO': object()}, {'AUTO': 9.0}, 10.0, 0.3)
    assert command is None
    assert status == 'AUTO_TIMEOUT'

    command, status = select_command(
        'MANUAL', {'MANUAL': None}, {'MANUAL': None}, 10.0, 0.3)
    assert command is None
    assert status == 'MANUAL_TIMEOUT'


@pytest.mark.parametrize(
    'speed,steering,expected',
    [
        (0.5, 0.0, (0.15, 0.0)),
        (2.0, 90.0, (0.3, -1.5)),
        (-1.0, -90.0, (0.0, 1.5)),
    ],
)
def test_sim_velocity_is_bounded_and_has_correct_sign(
        speed, steering, expected):
    actual = control_to_velocity(speed, steering, 0.3, 1.5)
    assert actual == pytest.approx(expected)
