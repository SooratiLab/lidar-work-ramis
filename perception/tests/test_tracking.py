"""
Unit tests for tracking.py's pure clustering/assignment/Kalman-filter logic.

These run against plain numpy arrays -- no rclpy, no ROS graph, no bag file
needed -- so they're the fast, repeatable check the project previously had
none of (verification so far had been "run it against a bag and read the
log"). Run with: python3 -m pytest perception/tests/
"""
import numpy as np
import pytest

from tracking import (
    CentroidTracker,
    KalmanTrack,
    assign_detections,
    cluster_moved_points,
)


# --- cluster_moved_points -----------------------------------------------

def test_cluster_moved_points_finds_two_separated_clusters():
    rng = np.random.default_rng(0)
    cluster_a = rng.normal(loc=[0.0, 0.0, 0.0], scale=0.02, size=(30, 3))
    cluster_b = rng.normal(loc=[5.0, 0.0, 0.0], scale=0.02, size=(30, 3))
    points = np.concatenate([cluster_a, cluster_b])

    clusters = cluster_moved_points(points, eps=0.5, min_points=10)

    assert len(clusters) == 2
    centroids = sorted(c["centroid"][0] for c in clusters)
    assert centroids[0] == pytest.approx(0.0, abs=0.05)
    assert centroids[1] == pytest.approx(5.0, abs=0.05)
    assert all(c["n_points"] == 30 for c in clusters)


def test_cluster_moved_points_drops_sparse_noise():
    # Fewer points than min_points anywhere -- should not form a cluster.
    points = np.array([[0.0, 0.0, 0.0], [10.0, 10.0, 10.0], [-5.0, 3.0, 1.0]])
    assert cluster_moved_points(points, eps=0.5, min_points=10) == []


def test_cluster_moved_points_below_min_points_short_circuits():
    # Fewer total points than min_points -- must not even call DBSCAN.
    points = np.zeros((3, 3))
    assert cluster_moved_points(points, eps=0.5, min_points=10) == []


# --- assign_detections ----------------------------------------------------

def test_assign_detections_matches_nearest_within_gate():
    track_positions = np.array([[0.0, 0.0, 0.0], [10.0, 0.0, 0.0]])
    detection_positions = np.array([[0.2, 0.0, 0.0], [10.3, 0.0, 0.0]])

    matches, unmatched_tracks, unmatched_detections = assign_detections(
        track_positions, detection_positions, max_distance=1.0)

    assert set(matches) == {(0, 0), (1, 1)}
    assert unmatched_tracks == []
    assert unmatched_detections == []


def test_assign_detections_globally_optimal_not_greedy():
    # Track 0 is closer to detection 1 (0.6) than detection 0 (0.9), and
    # track 1 is closer to detection 0 (0.1) than detection 1 (2.0).
    # A greedy "each detection grabs its nearest track" pass processing
    # detection 0 first would give detection 0 to track 0 (distance 0.9,
    # nearer than track 1's tiny gap only if track 1 isn't considered
    # first) -- the globally optimal assignment must instead pick the
    # pairing with the smaller *total* distance: (0,1)+(1,0) = 0.6+0.1=0.7,
    # versus (0,0)+(1,1) = 0.9+2.0 = 2.9.
    track_positions = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    detection_positions = np.array([[0.9, 0.0, 0.0], [0.6, 0.0, 0.0]])

    matches, unmatched_tracks, unmatched_detections = assign_detections(
        track_positions, detection_positions, max_distance=5.0)

    assert set(matches) == {(0, 1), (1, 0)}
    assert unmatched_tracks == []
    assert unmatched_detections == []


def test_assign_detections_rejects_pairs_beyond_gate():
    track_positions = np.array([[0.0, 0.0, 0.0]])
    detection_positions = np.array([[10.0, 0.0, 0.0]])

    matches, unmatched_tracks, unmatched_detections = assign_detections(
        track_positions, detection_positions, max_distance=1.0)

    assert matches == []
    assert unmatched_tracks == [0]
    assert unmatched_detections == [0]


def test_assign_detections_empty_inputs():
    empty = np.empty((0, 3))
    non_empty = np.array([[0.0, 0.0, 0.0]])

    matches, unmatched_tracks, unmatched_detections = assign_detections(
        empty, non_empty, max_distance=1.0)
    assert matches == [] and unmatched_tracks == [] and unmatched_detections == [0]

    matches, unmatched_tracks, unmatched_detections = assign_detections(
        non_empty, empty, max_distance=1.0)
    assert matches == [] and unmatched_tracks == [0] and unmatched_detections == []


# --- KalmanTrack -----------------------------------------------------------

def test_kalman_track_predicts_constant_velocity():
    track = KalmanTrack(0, np.array([0.0, 0.0, 0.0]),
                         position_variance=0.01, velocity_variance=4.0,
                         process_variance=1.0)
    # Feed a steady 1 m/s walk along x, one measurement per second, and
    # confirm the filter's velocity estimate converges towards 1 m/s.
    for step in range(1, 21):
        track.predict(dt=1.0)
        track.update(np.array([1.0 * step, 0.0, 0.0]))

    assert track.velocity[0] == pytest.approx(1.0, abs=0.05)
    assert track.position[0] == pytest.approx(20.0, abs=0.1)


def test_kalman_track_coasts_along_last_known_velocity():
    track = KalmanTrack(0, np.array([0.0, 0.0, 0.0]),
                         position_variance=0.01, velocity_variance=4.0,
                         process_variance=1.0)
    for step in range(1, 11):
        track.predict(dt=1.0)
        track.update(np.array([1.0 * step, 0.0, 0.0]))

    last_position = track.position.copy()
    last_velocity = track.velocity.copy()

    # No update this time -- coasting through a missed detection.
    track.predict(dt=1.0)

    assert track.position[0] == pytest.approx(last_position[0] + last_velocity[0], abs=0.05)


# --- CentroidTracker --------------------------------------------------------

def test_centroid_tracker_keeps_id_through_one_missed_frame():
    tracker = CentroidTracker(max_match_distance=1.5, max_missed_frames=2)

    result = tracker.step([{"centroid": np.array([0.0, 0.0, 0.0]), "n_points": 20}], dt=1.0)
    (track_id, info), = result.items()
    assert info["is_new"] is True

    # Frame 2: no detection at all (occlusion / detector miss).
    result = tracker.step([], dt=1.0)
    assert track_id in result
    assert result[track_id]["is_coasting"] is True

    # Frame 3: the person reappears near where the coasted prediction put them.
    result = tracker.step([{"centroid": np.array([0.3, 0.0, 0.0]), "n_points": 20}], dt=1.0)
    assert track_id in result
    assert result[track_id]["is_coasting"] is False
    assert result[track_id]["is_new"] is False


def test_centroid_tracker_drops_track_after_max_missed_frames():
    tracker = CentroidTracker(max_match_distance=1.5, max_missed_frames=2)

    result = tracker.step([{"centroid": np.array([0.0, 0.0, 0.0])}], dt=1.0)
    (track_id, _), = result.items()

    tracker.step([], dt=1.0)
    tracker.step([], dt=1.0)
    result = tracker.step([], dt=1.0)

    assert track_id not in result
    assert track_id not in tracker.tracks


def test_centroid_tracker_assigns_new_ids_to_two_people():
    tracker = CentroidTracker(max_match_distance=1.0)

    result = tracker.step([
        {"centroid": np.array([0.0, 0.0, 0.0])},
        {"centroid": np.array([5.0, 0.0, 0.0])},
    ], dt=1.0)
    assert len(result) == 2
    assert {info["track"].position[0] for info in result.values()} == {0.0, 5.0}

    # Both move a little; ids must stay attached to the same person.
    result = tracker.step([
        {"centroid": np.array([0.2, 0.0, 0.0])},
        {"centroid": np.array([5.3, 0.0, 0.0])},
    ], dt=1.0)
    assert len(result) == 2
    assert all(not info["is_new"] for info in result.values())
