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
    filter_plausible_detections,
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


# --- filter_plausible_detections -------------------------------------------

def test_filter_plausible_detections_drops_far_outliers():
    clusters = [
        {"centroid": np.array([5.0, 0.0, 0.0])},   # 5 m from sensor -- plausible
        {"centroid": np.array([1000.0, 0.0, 0.0])},  # drifted-odometry-scale outlier
    ]
    sensor_position = np.array([0.0, 0.0, 0.0])

    result = filter_plausible_detections(clusters, sensor_position, max_range=40.0)

    assert len(result) == 1
    np.testing.assert_allclose(result[0]["centroid"], [5.0, 0.0, 0.0])


def test_filter_plausible_detections_uses_sensor_position_not_origin():
    # A cluster 5 m from the sensor's *current* position should survive
    # even though it's far from the world origin -- the sensor has moved.
    clusters = [{"centroid": np.array([105.0, 0.0, 0.0])}]
    sensor_position = np.array([100.0, 0.0, 0.0])

    result = filter_plausible_detections(clusters, sensor_position, max_range=40.0)

    assert len(result) == 1


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


def test_centroid_tracker_drops_track_after_max_missed_seconds_even_with_few_frames():
    # A degraded/bursty scan rate can mean very few *frames* occur across a
    # long real gap. max_missed_frames alone wouldn't catch this -- the
    # wall-clock cap must drop the track before its coasted prediction runs
    # off to somewhere implausible.
    tracker = CentroidTracker(max_match_distance=1.5, max_missed_frames=10,
                               max_missed_seconds=5.0)

    result = tracker.step([{"centroid": np.array([0.0, 0.0, 0.0])}], dt=1.0)
    (track_id, _), = result.items()

    tracker.step([], dt=1.0)  # missed=1, missed_seconds=1 -- still coasting
    result = tracker.step([], dt=10.0)  # missed=2, missed_seconds=11 -- over the cap

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


# --- track confirmation (min_hits) -----------------------------------------

def test_single_frame_noise_never_becomes_confirmed():
    # A cluster that appears once and is never seen again (the dominant
    # false-positive pattern found when testing against recorded sessions --
    # long-range LiDAR noise, or a static object newly entering the field of
    # view on a moving sensor) should never cross is_confirmed=True at any
    # point in its short life.
    tracker = CentroidTracker(max_match_distance=1.5, max_missed_frames=3, min_hits=2)

    result = tracker.step([{"centroid": np.array([10.0, 10.0, 0.0])}], dt=1.0)
    (track_id, info), = result.items()
    assert info["is_confirmed"] is False

    for _ in range(3):
        result = tracker.step([], dt=1.0)
        assert result[track_id]["is_confirmed"] is False

    # And it's gone for good after coasting out.
    result = tracker.step([], dt=1.0)
    assert track_id not in result


def test_track_becomes_confirmed_after_min_hits_real_detections():
    tracker = CentroidTracker(max_match_distance=1.5, min_hits=2)

    result = tracker.step([{"centroid": np.array([0.0, 0.0, 0.0])}], dt=1.0)
    (track_id, info) = next(iter(result.items()))
    assert info["is_confirmed"] is False  # only 1 hit so far

    result = tracker.step([{"centroid": np.array([0.5, 0.0, 0.0])}], dt=1.0)
    assert result[track_id]["is_confirmed"] is True  # 2nd real hit

    # Stays confirmed while coasting through a later missed frame too.
    result = tracker.step([], dt=1.0)
    assert result[track_id]["is_confirmed"] is True
    assert result[track_id]["is_coasting"] is True


# --- re-identification (reid_max_distance / reid_window_seconds) -----------

def test_reidentification_revives_original_id_after_coasting_expires():
    tracker = CentroidTracker(max_match_distance=1.0, max_missed_frames=1,
                               min_hits=2, reid_max_distance=1.0,
                               reid_window_seconds=30.0)

    tracker.step([{"centroid": np.array([0.0, 0.0, 0.0])}], dt=1.0)
    result = tracker.step([{"centroid": np.array([0.1, 0.0, 0.0])}], dt=1.0)
    (track_id, info) = next(iter(result.items()))
    assert info["is_confirmed"] is True  # 2 real hits so far

    # Drop the track entirely -- coasts out within max_missed_frames=1.
    tracker.step([], dt=1.0)
    result = tracker.step([], dt=1.0)
    assert track_id not in result
    assert track_id not in tracker.tracks

    # Person reappears nearby a few seconds later (a real stop, not a
    # missed detection while still moving) -- same ID, not a new one, and
    # confirmed immediately since the hit count carries over.
    result = tracker.step([{"centroid": np.array([0.2, 0.0, 0.0])}], dt=5.0)
    assert track_id in result
    assert result[track_id]["is_reidentified"] is True
    assert result[track_id]["is_new"] is False
    assert result[track_id]["is_confirmed"] is True


def test_reidentification_rejects_match_beyond_distance_gate():
    tracker = CentroidTracker(max_match_distance=1.0, max_missed_frames=1,
                               min_hits=1, reid_max_distance=1.0,
                               reid_window_seconds=30.0)

    result = tracker.step([{"centroid": np.array([0.0, 0.0, 0.0])}], dt=1.0)
    (track_id, _) = next(iter(result.items()))

    tracker.step([], dt=1.0)
    tracker.step([], dt=1.0)  # dropped, now in the lost-tracks pool

    # Reappears 5 m away -- too far to plausibly be the same person having
    # just stopped in place, so this must start a new track, not reuse the
    # old ID.
    result = tracker.step([{"centroid": np.array([5.0, 0.0, 0.0])}], dt=1.0)
    (new_track_id, info) = next(iter(result.items()))
    assert new_track_id != track_id
    assert info["is_reidentified"] is False
    assert info["is_new"] is True


def test_reidentification_rejects_match_beyond_time_window():
    tracker = CentroidTracker(max_match_distance=1.0, max_missed_frames=1,
                               min_hits=1, reid_max_distance=2.0,
                               reid_window_seconds=5.0)

    result = tracker.step([{"centroid": np.array([0.0, 0.0, 0.0])}], dt=1.0)
    (track_id, _) = next(iter(result.items()))

    tracker.step([], dt=1.0)
    tracker.step([], dt=1.0)  # dropped

    # Reappears nearby, but well outside the 5-second reid_window_seconds.
    result = tracker.step([{"centroid": np.array([0.1, 0.0, 0.0])}], dt=10.0)
    (new_track_id, info) = next(iter(result.items()))
    assert new_track_id != track_id
    assert info["is_reidentified"] is False


def test_reidentification_carries_over_hit_count_not_velocity():
    tracker = CentroidTracker(max_match_distance=1.0, max_missed_frames=1,
                               min_hits=2, reid_max_distance=5.0,
                               reid_window_seconds=30.0)

    # Build up a track moving steadily along x so it has a nonzero
    # velocity estimate before it's lost. reid_max_distance is generous
    # here specifically so the test doesn't depend on exactly how far the
    # coasted prediction drifted before being dropped -- that's incidental
    # to what this test is actually checking (velocity resets on revival).
    tracker.step([{"centroid": np.array([0.0, 0.0, 0.0])}], dt=1.0)
    result = tracker.step([{"centroid": np.array([1.0, 0.0, 0.0])}], dt=1.0)
    (track_id, _) = next(iter(result.items()))
    assert result[track_id]["track"].velocity[0] > 0.0

    tracker.step([], dt=1.0)
    tracker.step([], dt=1.0)  # dropped -- person has stopped, not still moving

    result = tracker.step([{"centroid": np.array([1.2, 0.0, 0.0])}], dt=5.0)
    revived = result[track_id]["track"]
    # Velocity resets to zero on revival rather than carrying the pre-gap
    # estimate through the stop -- see the class docstring for why.
    np.testing.assert_allclose(revived.velocity, [0.0, 0.0, 0.0])


def test_reidentification_pool_entry_expires_and_is_not_reused_twice():
    tracker = CentroidTracker(max_match_distance=1.0, max_missed_frames=1,
                               min_hits=1, reid_max_distance=1.0,
                               reid_window_seconds=5.0)

    result = tracker.step([{"centroid": np.array([0.0, 0.0, 0.0])}], dt=1.0)
    (track_id, _) = next(iter(result.items()))

    tracker.step([], dt=1.0)
    tracker.step([], dt=1.0)  # dropped, pool entry age starts at 0

    # Let the pool entry expire (age exceeds reid_window_seconds=5) before
    # anything reappears.
    tracker.step([], dt=10.0)

    result = tracker.step([{"centroid": np.array([0.1, 0.0, 0.0])}], dt=1.0)
    (new_track_id, info) = next(iter(result.items()))
    assert new_track_id != track_id
    assert info["is_reidentified"] is False
