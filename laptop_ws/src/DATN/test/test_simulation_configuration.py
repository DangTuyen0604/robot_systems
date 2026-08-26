"""Static regression tests for the Gazebo control pipeline."""

from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_simulation_config_has_safety_timeouts():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'simulation.yaml').read_text())
    assert config['command_mux']['ros__parameters']['command_timeout'] <= 0.5
    assert (
        config['sim_motor_bridge_node']['ros__parameters']['command_timeout']
        <= 0.5)
    lane = config['lane_node_instance']['ros__parameters']
    assert lane['lost_lane_recovery_max_frames'] > lane['lost_lane_recovery_frames']


def test_sim_bridge_uses_mux_output():
    bridge = (PACKAGE_ROOT.parent / 'decision_pkg' / 'decision_pkg' /
              'sim_motor_bridge_node.py').read_text()
    assert "Control, '/control/cmd'" in bridge
    assert "Control, '/control/auto'" not in bridge
