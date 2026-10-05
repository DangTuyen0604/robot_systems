"""Tests for the Arduino serial packets and the /arduino/pid gate."""

from control_pkg.serial_protocol import (
    STOP_PACKET, motion_packet, pid_rejection_reason)


def test_motion_packet_is_bounded():
    assert motion_packet(0.5, 0.0) == 'V:127;E:0\n'
    assert motion_packet(2.0, 90.0) == 'V:255;E:320\n'
    assert motion_packet(-1.0, -90.0) == 'V:0;E:-320\n'
    assert STOP_PACKET == 'V:0;E:0\n'


def test_pid_packets_accepted_while_stopped():
    assert pid_rejection_reason('KS:0.03;KSI:0.001;KSD:0.05', 0.0) is None
    assert pid_rejection_reason('P:2.0;I:0.05;D:0.02', 0.0) is None
    assert pid_rejection_reason('KS:-1;KSI:.5;KSD:2.', 0.0) is None


def test_pid_rejected_while_moving():
    assert pid_rejection_reason('KS:0.03;KSI:0.001;KSD:0.05', 0.3) is not None


def test_only_pid_packets_pass():
    for packet in (
            'V:255;E:0',                  # motion
            'KS:1;KSI:0;KSD:0;V:255',     # motion appended
            'KS:1;KSI:0',                 # incomplete
            'KS:abc;KSI:0;KSD:0',         # not a number
            ''):
        assert pid_rejection_reason(packet, 0.0) is not None


def test_pid_cannot_inject_a_second_line():
    assert pid_rejection_reason('KS:1;KSI:0;KSD:0\nV:255;E:0', 0.0) is not None
    assert pid_rejection_reason('KS:1;KSI:0;KSD:0\r', 0.0) is not None


def test_pid_rejects_packets_longer_than_firmware_buffer():
    assert pid_rejection_reason('KS:0.0300;KSI:0.00100;KSD:0.05000', 0.0) is not None
