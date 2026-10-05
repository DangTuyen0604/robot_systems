"""Cross-workspace consistency checks (run with pytest from the repo root)."""

import filecmp
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'raspberry_ws' / 'src' / 'control_pkg'))

from control_pkg.serial_protocol import (  # noqa: E402
    MAX_PID_PACKET_LENGTH, motion_packet, STOP_PACKET)

FIRMWARE = (ROOT / 'firmware' / 'code_xe' / 'code_xe.ino').read_text()


def test_interfaces_pkg_copies_are_identical():
    # Each workspace builds its own copy; a drift breaks laptop <-> Pi DDS.
    laptop = ROOT / 'laptop_ws' / 'src' / 'interfaces_pkg'
    pi = ROOT / 'raspberry_ws' / 'src' / 'interfaces_pkg'
    files = ['CMakeLists.txt', 'package.xml', 'msg/Control.msg']
    match, mismatch, errors = filecmp.cmpfiles(laptop, pi, files, shallow=False)
    assert mismatch == [] and errors == [], f'differs: {mismatch + errors}'


def test_pid_packet_limit_matches_firmware_buffer():
    buffer = int(re.search(r'serial_buffer\.length\(\) < (\d+)', FIRMWARE).group(1))
    assert MAX_PID_PACKET_LENGTH == buffer


def test_motion_packets_fit_firmware_parser():
    v_max = int(re.search(r'if \(v < 0 \|\| v > (\d+)\) return;', FIRMWARE).group(1))
    e_max = int(re.search(r'if \(e < -(\d+) \|\| e > \d+\) return;', FIRMWARE).group(1))
    for speed, steering in ((1.0, 45.0), (1.0, -45.0), (0.0, 0.0)):
        packet = motion_packet(speed, steering).strip()
        v, e = (int(x) for x in re.fullmatch(r'V:(-?\d+);E:(-?\d+)', packet).groups())
        assert 0 <= v <= v_max and -e_max <= e <= e_max
    assert STOP_PACKET == 'V:0;E:0\n'


def test_firmware_watchdog_matches_readme():
    timeout = int(re.search(r'#define CMD_TIMEOUT_MS\s+(\d+)', FIRMWARE).group(1))
    assert f'{timeout} ms' in (ROOT / 'README.md').read_text()
