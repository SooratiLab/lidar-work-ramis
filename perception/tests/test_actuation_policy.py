import pytest

from actuation_policy import StopCommandGate


def test_sends_on_rising_edge_and_rate_limits_repeats():
    gate = StopCommandGate(repeat_interval=1.0)
    assert gate.update(True, 0.0).should_send_stop
    assert not gate.update(True, 0.5).should_send_stop
    repeated = gate.update(True, 1.0)
    assert repeated.should_send_stop
    assert repeated.reason == "repeat_stop_request"


def test_clear_rearms_next_stop_request():
    gate = StopCommandGate()
    gate.update(True, 0.0)
    assert not gate.update(False, 0.1).should_send_stop
    assert gate.update(True, 0.2).should_send_stop


def test_clock_regression_sends_again():
    gate = StopCommandGate()
    gate.update(True, 10.0)
    assert gate.update(True, 1.0).should_send_stop


def test_rejects_invalid_interval():
    with pytest.raises(ValueError):
        StopCommandGate(repeat_interval=0.0)
