import csv

import numpy as np
import pytest

from response_evaluator import evaluate_response, write_response_csv
from response_policy import NearClusterStopPolicy


def _track(frame, x, status="matched"):
    return {
        "frame": frame,
        "centroid_x_m": x,
        "centroid_y_m": 0.0,
        "centroid_z_m": 0.0,
        "status": status,
    }


def test_evaluates_empty_frames_and_ignores_coasting_tracks():
    tracks = [_track(0, 1.0), _track(1, 1.0, "coasting")]
    positions = {frame: np.zeros(3) for frame in range(4)}
    stamps = {frame: float(frame) for frame in range(4)}
    policy = NearClusterStopPolicy(
        trigger_duration=0.0, clear_duration=0.0)

    rows, summary = evaluate_response(tracks, positions, stamps, policy)

    assert rows[0]["stop_requested"]
    assert rows[1]["n_current_tracks"] == 0
    assert not rows[1]["stop_requested"]
    assert summary["stop_transitions"] == 1
    assert summary["minimum_nearest_distance_m"] == pytest.approx(1.0)


def test_requires_odometry():
    with pytest.raises(ValueError):
        evaluate_response([], {}, {})


def test_writes_machine_readable_csv(tmp_path):
    rows = [{
        "frame": 1,
        "time_s": 0.0,
        "nearest_distance_m": None,
        "state": "clear",
        "stop_requested": False,
        "n_current_tracks": 0,
    }]
    path = tmp_path / "response.csv"
    write_response_csv(rows, path)
    written = list(csv.DictReader(open(path)))
    assert written[0]["state"] == "clear"
