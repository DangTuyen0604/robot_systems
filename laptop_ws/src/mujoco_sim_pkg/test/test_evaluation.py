"""Unit tests for the scenario metrics, on synthetic trajectories."""

import pytest

from mujoco_sim_pkg.evaluation import end_reached, evaluate, lane_error, Sample


def path(points, dt=0.1):
    """Return samples moving through ``points`` (x, y, speed) at a fixed rate."""
    return [Sample(t=i * dt, x=x, y=y, yaw=0.0, speed=v) for i, (x, y, v) in enumerate(points)]


def ideal_run(stem_offset=0.0, branch_offset=0.0, stop_s=5.0, slow_s=5.0):
    pts = [(x / 100, stem_offset, 0.095) for x in range(0, 101)]
    pts += [(1.5 + branch_offset, 0.7 + i * 0.01, 0.054) for i in range(int(slow_s * 10))]
    pts += [(1.5 + branch_offset, 1.2 + i * 0.01, 0.095) for i in range(140)]
    pts += [(1.5 + branch_offset, 2.6, 0.0)] * int(stop_s * 10)
    pts += [(1.5 + branch_offset, 2.6 + i * 0.01, 0.095) for i in range(45)]
    return path(pts)


def test_lane_error_segments():
    assert lane_error(0.5, 0.02) == ('stem', pytest.approx(0.02))
    assert lane_error(1.47, 2.0) == ('branch', pytest.approx(-0.03))
    assert lane_error(1.3, 0.3) == (None, None)  # junction


def test_ideal_run_passes_everything():
    m = evaluate(ideal_run(stem_offset=0.004, branch_offset=-0.003))
    assert m.stem_mean_error_mm == pytest.approx(4.0)
    assert m.branch_max_error_mm == pytest.approx(3.0)
    assert m.kept_lane and m.took_left_branch and m.completed
    assert m.stop_hold_s >= 4.0 and m.slow_hold_s >= 4.0


def test_leaving_the_lane_is_detected():
    m = evaluate(ideal_run(branch_offset=0.14))
    assert not m.kept_lane
    assert not m.took_left_branch  # centre outside the branch tapes
    assert not m.completed


def test_end_of_route_needs_a_lasting_stop_past_the_end():
    moving = ideal_run()
    assert not end_reached(moving)
    t = moving[-1].t
    stopped = moving + [Sample(t + 0.1 * i, 1.5, 3.05, 0.0, 0.0) for i in range(1, 41)]
    assert end_reached(stopped)
    # The 5 s halt at the STOP sign (y 2.6) is not the end of the route.
    assert not end_reached(ideal_run()[:-45])


def test_finish_time():
    m = evaluate(ideal_run())
    assert 0.0 < m.finish_time_s < m.duration


def test_missing_stop_is_detected():
    m = evaluate(ideal_run(stop_s=0.0))
    assert m.stop_hold_s < 1.0
