"""Tests for the encoder telemetry parser and the pulses-per-second table."""

import pytest

from control_pkg.encoder_rate import parse_telemetry, RateTable


def test_parse_firmware_debug_line():
    line = 'ERR:-12,D:-0.4,PL:81,PR:65,EL:1200,ER:1150\r\n'
    assert parse_telemetry(line) == (81, 65, 1200, 1150)
    assert parse_telemetry('Arduino ready!') is None


def test_rate_per_pwm():
    table = RateTable()
    for i in range(21):  # 2 s at 10 Hz: left 30 pulses/s, right 25 pulses/s
        table.add(i * 0.1, (81, 70, 3 * i, int(2.5 * i)))
    rows = {(w, pwm): rate for w, pwm, rate, _ in table.rows()}
    assert rows[('left', 81)] == pytest.approx(30.0)
    assert rows[('right', 70)] == pytest.approx(25.0, abs=0.6)


def test_gaps_are_not_counted():
    table = RateTable()
    table.add(0.0, (81, 81, 0, 0))
    table.add(5.0, (81, 81, 500, 500))  # telemetry paused while stopped
    assert table.rows(min_seconds=0.0) == []
