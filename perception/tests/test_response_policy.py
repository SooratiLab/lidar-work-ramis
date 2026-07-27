import numpy as np
import pytest

from response_policy import NearClusterStopPolicy


def test_requests_stop_only_after_consecutive_near_frames():
    policy = NearClusterStopPolicy(trigger_duration=1.0)
    first = policy.update([[1.0, 0.0, 0.0]], np.zeros(3), 10.0)
    second = policy.update([[1.0, 0.0, 0.0]], np.zeros(3), 11.0)
    assert not first.stop_requested
    assert first.state == "pending_stop"
    assert second.stop_requested
    assert second.changed


def test_hysteresis_prevents_chatter_and_clears_on_empty_frames():
    policy = NearClusterStopPolicy(
        stop_distance=2.0,
        clear_distance=2.5,
        trigger_duration=0.0,
        clear_duration=1.0,
    )
    assert policy.update([[1.5, 0, 0]], [0, 0, 0], 0.0).stop_requested
    assert policy.update([[2.2, 0, 0]], [0, 0, 0], 0.5).stop_requested
    assert policy.update([], [0, 0, 0], 1.0).stop_requested
    cleared = policy.update([], [0, 0, 0], 2.0)
    assert not cleared.stop_requested
    assert cleared.changed
    assert cleared.state == "clear"


def test_uses_sensor_position_not_map_origin():
    policy = NearClusterStopPolicy(trigger_duration=0.0)
    decision = policy.update([[11.0, 0, 0]], [10.0, 0, 0], 0.0)
    assert decision.stop_requested
    assert decision.nearest_distance == pytest.approx(1.0)


def test_uses_horizontal_distance_by_default():
    policy = NearClusterStopPolicy(trigger_duration=0.0)
    decision = policy.update([[1.0, 0.0, 10.0]], [0.0, 0.0, 0.0], 0.0)
    assert decision.stop_requested
    assert decision.nearest_distance == pytest.approx(1.0)


def test_requires_continuous_near_duration():
    policy = NearClusterStopPolicy(trigger_duration=1.0)
    assert not policy.update([[1, 0, 0]], [0, 0, 0], 0.0).stop_requested
    assert not policy.update([], [0, 0, 0], 0.9).stop_requested
    assert not policy.update([[1, 0, 0]], [0, 0, 0], 1.0).stop_requested
    assert policy.update([[1, 0, 0]], [0, 0, 0], 2.0).stop_requested


def test_clock_regression_starts_a_fresh_dwell_period():
    policy = NearClusterStopPolicy(trigger_duration=1.0)
    policy.update([[1, 0, 0]], [0, 0, 0], 10.0)
    decision = policy.update([[1, 0, 0]], [0, 0, 0], 1.0)
    assert not decision.stop_requested
    assert decision.state == "pending_stop"


def test_rejects_invalid_thresholds():
    with pytest.raises(ValueError):
        NearClusterStopPolicy(stop_distance=2.0, clear_distance=1.0)
    with pytest.raises(ValueError):
        NearClusterStopPolicy(trigger_duration=-1.0)
