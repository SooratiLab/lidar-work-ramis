import numpy as np
import pytest

from temporal_consensus import changed_in_history_mask


def test_rejects_point_missing_from_only_one_history_frame():
    current = np.array([[1.0, 0.0, 0.0]])
    history = [
        np.array([[3.0, 0.0, 0.0]]),
        np.array([[1.01, 0.0, 0.0]]),
        np.array([[0.99, 0.0, 0.0]]),
    ]

    result = changed_in_history_mask(
        current, history, distance_threshold=0.15, min_changed_ratio=0.5)

    assert result.histories_used == 3
    assert result.required_changed_histories == 2
    assert not result.changed_mask[0]


def test_keeps_point_changed_from_a_majority_of_history():
    current = np.array([[1.0, 0.0, 0.0]])
    history = [
        np.array([[3.0, 0.0, 0.0]]),
        np.array([[4.0, 0.0, 0.0]]),
        np.array([[1.01, 0.0, 0.0]]),
    ]

    result = changed_in_history_mask(
        current, history, distance_threshold=0.15, min_changed_ratio=0.5)

    assert result.changed_mask[0]


def test_ratio_controls_required_votes():
    current = np.array([[1.0, 0.0, 0.0]])
    history = [
        np.array([[3.0, 0.0, 0.0]]),
        np.array([[4.0, 0.0, 0.0]]),
        np.array([[1.0, 0.0, 0.0]]),
    ]

    result = changed_in_history_mask(
        current, history, distance_threshold=0.15, min_changed_ratio=1.0)

    assert result.required_changed_histories == 3
    assert not result.changed_mask[0]


def test_ignores_empty_history_frames():
    result = changed_in_history_mask(
        np.array([[1.0, 0.0, 0.0]]),
        [np.empty((0, 3)), np.array([[3.0, 0.0, 0.0]])],
        distance_threshold=0.15,
    )

    assert result.histories_used == 1
    assert result.changed_mask[0]


@pytest.mark.parametrize("ratio", [0.0, -0.1, 1.1])
def test_rejects_invalid_ratio(ratio):
    with pytest.raises(ValueError, match="min_changed_ratio"):
        changed_in_history_mask(
            np.empty((0, 3)), [], distance_threshold=0.15,
            min_changed_ratio=ratio)
