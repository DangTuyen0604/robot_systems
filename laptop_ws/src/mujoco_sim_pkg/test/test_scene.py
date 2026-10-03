"""Tests that the MuJoCo model compiles and behaves like the Gazebo robot."""

import math
from pathlib import Path

import numpy as np
import pytest

from mujoco_sim_pkg.diff_drive import wheel_speeds
from mujoco_sim_pkg.scene import composite_on_plate

mujoco = pytest.importorskip('mujoco')

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DATN_ROOT = PACKAGE_ROOT.parent / 'DATN'
MODEL = PACKAGE_ROOT / 'models' / 'datn_track.xml'
RADIUS = 0.034
SEPARATION = 0.2133


@pytest.fixture(scope='module')
def model():
    from mujoco_sim_pkg.scene import load_model
    return load_model(str(MODEL), str(DATN_ROOT))


def drive(model, linear, angular, seconds):
    data = mujoco.MjData(model)
    left, right = wheel_speeds(linear, angular, SEPARATION, RADIUS)
    data.ctrl[model.actuator('left_motor').id] = left
    data.ctrl[model.actuator('right_motor').id] = right
    for _ in range(int(seconds / model.opt.timestep)):
        mujoco.mj_step(model, data)
    qpos = data.joint('root').qpos
    w, z = qpos[3], qpos[6]
    return qpos[0], qpos[1], qpos[2], 2.0 * math.atan2(z, w)


def test_transparent_pixels_become_plate_not_black():
    rgba = np.zeros((2, 2, 4), dtype=np.uint8)
    rgba[0, 0] = (0, 0, 255, 255)
    bgr = composite_on_plate(rgba, (200, 200, 200))
    assert tuple(bgr[0, 0]) == (0, 0, 255)
    assert tuple(bgr[1, 1]) == (200, 200, 200)


def test_camera_matches_gazebo_intrinsics(model):
    camera = model.camera('front_camera')
    fy = 120.0 / math.tan(math.radians(camera.fovy[0]) / 2.0)
    assert fy == pytest.approx(265.0, abs=0.5)


def test_robot_rests_level_on_wheels_and_caster(model):
    x, y, z, yaw = drive(model, 0.0, 0.0, 1.0)
    assert z == pytest.approx(0.035, abs=0.002)
    assert abs(x) < 0.005 and abs(y) < 0.005 and abs(yaw) < 0.01


def test_forward_command_drives_straight(model):
    x, y, _, yaw = drive(model, 0.2, 0.0, 2.0)
    assert x == pytest.approx(0.4, rel=0.15)
    assert abs(y) < 0.02
    assert abs(yaw) < math.radians(3)


def test_positive_angular_turns_left(model):
    _, _, _, yaw = drive(model, 0.0, 1.0, 1.0)
    assert yaw == pytest.approx(1.0, rel=0.2)
