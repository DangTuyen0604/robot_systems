"""
End-to-end scenario tests: the full stack drives the MuJoCo track.

Each test launches mujoco_full_launch.py headless and runs until the robot
stops at the end of the route (about 1 min, longer with software rendering),
so they only run when MUJOCO_SCENARIO=1 (set in CI):

    MUJOCO_SCENARIO=1 python -m pytest src/mujoco_sim_pkg/test/test_scenario.py
"""

import os

import pytest

from mujoco_sim_pkg.evaluation import evaluate, MIN_STOP_TIME

pytestmark = pytest.mark.skipif(
    os.environ.get('MUJOCO_SCENARIO') != '1',
    reason='scenario tests run only with MUJOCO_SCENARIO=1')

MAX_MEAN_ERROR_MM = 10.0
MAX_DURATION = 240.0


def sign_model_installed():
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory('perception_pkg')
    return os.path.exists(os.path.join(share, 'viet_320.onnx'))


def run(signs, tmp_path):
    from mujoco_sim_pkg.scenario import run_scenario
    log = tmp_path / 'launch.log'
    samples = run_scenario(MAX_DURATION, signs=signs, log_path=str(log))
    assert samples, f'no /odom received; see {log}'
    return evaluate(samples)


def test_lane_following_takes_the_left_branch(tmp_path):
    # Without signs the lane node's search turn handles the T-junction.
    m = run(signs=False, tmp_path=tmp_path)
    assert m.kept_lane, m
    assert m.took_left_branch and m.completed, m
    assert m.stem_mean_error_mm < MAX_MEAN_ERROR_MM, m
    assert m.branch_mean_error_mm < MAX_MEAN_ERROR_MM, m


@pytest.mark.skipif(
    not os.environ.get('AMENT_PREFIX_PATH') or not sign_model_installed(),
    reason='viet_320.onnx not installed (run scripts/fetch_models.py)')
def test_signs_turn_slow_and_stop(tmp_path):
    m = run(signs=True, tmp_path=tmp_path)
    assert m.kept_lane, m
    assert m.took_left_branch and m.completed, m
    assert m.slow_hold_s >= 3.0, m
    assert m.stop_hold_s >= MIN_STOP_TIME, m
