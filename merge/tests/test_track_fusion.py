"""
Unit tests for track_fusion.py's calibration and fusion logic.

Everything here runs against synthetic Observations built directly in
Python, not real exported sessions -- the pipeline wiring that produces
real track logs already has its own coverage in evaluation/tests/, and
merge/track_fusion.py's own CLI entry point (_run_pipeline_for_session)
is a thin wrapper around that, not logic worth re-testing here. What's
unique to this module -- frame-lag search, transform refit, mutual-vote
correspondence, and fused-row construction -- is exercised directly.
"""
import csv

import numpy as np
import pytest

from track_fusion import (
    Observation,
    apply_transform,
    best_frame_offset,
    build_transform,
    calibrate,
    fuse_tracks,
    group_by_frame,
    group_by_frame_track,
    load_track_rows,
    observations_from_track_rows,
    parse_offset_string,
    plot_fusion,
    refit_transform,
    write_fused_csv,
    _majority_correspondence,
)


# --- transform construction / parsing -----------------------------------

def test_build_transform_identity():
    T = build_transform(0.0, 0.0, 0.0, 0.0)
    assert np.allclose(T, np.eye(4))


def test_build_transform_translation_only():
    T = build_transform(1.0, 2.0, 0.5, 0.0)
    point = np.array([[0.0, 0.0, 0.0]])
    result = apply_transform(point, T)
    assert np.allclose(result, [[1.0, 2.0, 0.5]])


def test_build_transform_yaw_rotates_point():
    T = build_transform(0.0, 0.0, 0.0, 90.0)
    point = np.array([[1.0, 0.0, 0.0]])
    result = apply_transform(point, T)
    assert np.allclose(result, [[0.0, 1.0, 0.0]], atol=1e-9)


def test_parse_offset_string_roundtrips_build_transform():
    parsed = parse_offset_string("1.5,-2.0,0.1,30")
    expected = build_transform(1.5, -2.0, 0.1, 30.0)
    assert np.allclose(parsed, expected)


def test_parse_offset_string_rejects_wrong_count():
    with pytest.raises(ValueError):
        parse_offset_string("1,2,3")


def test_apply_transform_empty_input():
    result = apply_transform(np.empty((0, 3)), build_transform(1, 2, 3, 45))
    assert result.shape == (0, 3)


# --- refit_transform -----------------------------------------------------

def test_refit_transform_recovers_known_transform():
    rng = np.random.default_rng(0)
    true_transform = build_transform(3.0, -1.5, 0.2, 40.0)

    b_positions = rng.uniform(-5, 5, size=(10, 3))
    a_positions = apply_transform(b_positions, true_transform)
    pairs = list(zip(a_positions, b_positions))

    fitted = refit_transform(pairs)
    assert np.allclose(fitted, true_transform, atol=1e-6)


def test_refit_transform_needs_at_least_two_pairs():
    with pytest.raises(ValueError):
        refit_transform([(np.zeros(3), np.zeros(3))])


# --- frame-lag search ------------------------------------------------------

def _make_observations(frame_track_positions):
    """frame_track_positions: {frame: {track_id: (x, y, z)}} -> list[Observation]."""
    obs = []
    for frame, tracks in frame_track_positions.items():
        for track_id, position in tracks.items():
            obs.append(Observation(frame=frame, track_id=track_id,
                                    position_m=np.array(position), n_points=50, speed_m_s=1.0))
    return obs


def test_best_frame_offset_recovers_known_lag():
    # dog A sees a track moving along x at frames 0..9; dog B sees the same
    # motion but its own frame counter starts 2 frames later.
    a_frames = {f: {0: (float(f), 0.0, 0.0)} for f in range(10)}
    b_frames = {f: {0: (float(f - 2), 0.0, 0.0)} for f in range(2, 12)}

    obs_a_by_frame = group_by_frame(_make_observations(a_frames))
    obs_b_by_frame = group_by_frame(_make_observations(b_frames))

    lag = best_frame_offset(obs_a_by_frame, obs_b_by_frame, np.eye(4), max_distance=0.5, max_lag=5)
    assert lag == 2


def test_best_frame_offset_defaults_to_zero_with_no_overlap():
    obs_a_by_frame = group_by_frame(_make_observations({0: {0: (0.0, 0.0, 0.0)}}))
    obs_b_by_frame = group_by_frame(_make_observations({0: {0: (100.0, 100.0, 0.0)}}))

    lag = best_frame_offset(obs_a_by_frame, obs_b_by_frame, np.eye(4), max_distance=0.5, max_lag=3)
    assert lag == 0


# --- majority correspondence ------------------------------------------------

def test_majority_correspondence_keeps_mutual_best_matches():
    votes = {(1, 10): 5, (1, 11): 1, (2, 11): 4}
    correspondence = _majority_correspondence(votes, min_votes=2)
    assert correspondence == {1: 10, 2: 11}


def test_majority_correspondence_drops_below_min_votes():
    votes = {(1, 10): 1}
    correspondence = _majority_correspondence(votes, min_votes=2)
    assert correspondence == {}


# --- calibrate (end to end on synthetic data) -------------------------------

def test_calibrate_recovers_transform_and_correspondence():
    true_transform = build_transform(2.0, 1.0, 0.0, 15.0)

    # A single object walks a path over 20 frames. Dog A observes it
    # directly; dog B observes the same object in its own (transformed)
    # frame, with its frame counter offset by 3 (dog B's frame index runs
    # 3 ahead of dog A's for the same real content -- see
    # best_frame_offset's docstring/test above for the lag sign
    # convention).
    path = np.stack([np.linspace(0, 10, 20), np.zeros(20), np.zeros(20)], axis=1)
    a_frames = {f: {0: tuple(path[f])} for f in range(20)}
    b_positions = apply_transform(path, np.linalg.inv(true_transform))
    b_frames = {f + 3: {7: tuple(b_positions[f])} for f in range(20)}

    obs_a_by_frame = group_by_frame(_make_observations(a_frames))
    obs_b_by_frame = group_by_frame(_make_observations(b_frames))

    # Calibration refines a prior, it doesn't search for one from nothing
    # (matching merge_with_prior.py's own "reasonable initial-pose guess"
    # requirement) -- start from a rough guess close to the true
    # transform, not identity, since the true offset here (~2.2 m
    # translation) is well outside max_distance on its own.
    initial_guess = build_transform(1.7, 0.8, 0.0, 10.0)
    result = calibrate(obs_a_by_frame, obs_b_by_frame, initial_guess,
                        max_distance=1.0, max_lag=5, max_iterations=5, min_votes=2)

    assert result.lag == 3
    assert result.correspondence == {0: 7}
    assert result.n_matched_pairs > 0
    assert np.allclose(result.transform, true_transform, atol=1e-3)


def test_calibrate_reports_no_correspondence_when_dogs_never_agree():
    obs_a_by_frame = group_by_frame(_make_observations({0: {0: (0.0, 0.0, 0.0)}}))
    obs_b_by_frame = group_by_frame(_make_observations({0: {0: (500.0, 500.0, 0.0)}}))

    result = calibrate(obs_a_by_frame, obs_b_by_frame, np.eye(4), max_distance=1.0, max_lag=2)

    assert result.correspondence == {}
    assert result.n_matched_pairs == 0
    assert result.converged is False
    assert np.allclose(result.transform, np.eye(4))


def test_calibrate_no_refinement_when_max_iterations_zero():
    # A deliberately wrong prior that still matches within max_distance --
    # max_iterations=0 should report the correspondence but leave the
    # (wrong) prior transform untouched, matching --no-icp semantics.
    a_frames = {0: {0: (0.0, 0.0, 0.0)}}
    b_frames = {0: {5: (0.3, 0.0, 0.0)}}
    obs_a_by_frame = group_by_frame(_make_observations(a_frames))
    obs_b_by_frame = group_by_frame(_make_observations(b_frames))

    prior = build_transform(0.0, 0.0, 0.0, 0.0)
    result = calibrate(obs_a_by_frame, obs_b_by_frame, prior, max_distance=1.0, max_iterations=0)

    assert np.allclose(result.transform, prior)


# --- fuse_tracks -------------------------------------------------------------

def test_fuse_tracks_averages_corresponded_pair():
    a_frames = {0: {1: (0.0, 0.0, 0.0)}}
    b_frames = {0: {9: (2.0, 0.0, 0.0)}}  # 2 m apart before the calibration offset below
    obs_a_by_frame = group_by_frame(_make_observations(a_frames))
    obs_b_by_frame = group_by_frame(_make_observations(b_frames))

    # transform maps dog B into dog A's frame by shifting -2 m in x, so both
    # dogs report position (0, 0, 0) after transform.
    transform = build_transform(-2.0, 0.0, 0.0, 0.0)
    correspondence = {1: 9}

    rows = fuse_tracks(obs_a_by_frame, obs_b_by_frame, transform, lag=0, correspondence=correspondence)

    assert len(rows) == 1
    row = rows[0]
    assert row["source"] == "dog1+dog2"
    assert row["global_id"] == 1
    assert row["centroid_x_m"] == pytest.approx(0.0, abs=1e-9)
    assert row["n_points"] == 100  # 50 + 50 from _make_observations' default


def test_fuse_tracks_keeps_uncorresponded_tracks_from_both_dogs():
    a_frames = {0: {1: (0.0, 0.0, 0.0)}}
    b_frames = {0: {9: (50.0, 50.0, 0.0)}}  # nowhere near dog A's track
    obs_a_by_frame = group_by_frame(_make_observations(a_frames))
    obs_b_by_frame = group_by_frame(_make_observations(b_frames))

    rows = fuse_tracks(obs_a_by_frame, obs_b_by_frame, np.eye(4), lag=0, correspondence={})

    sources = {row["global_id"]: row["source"] for row in rows}
    assert sources[1] == "dog1_only"
    assert sources["b:9"] == "dog2_only"


def test_fuse_tracks_applies_lag():
    # lag=3 means dog B's frame index runs 3 ahead of dog A's for the same
    # real content (see best_frame_offset's docstring/test) -- dog A's
    # frame 5 corresponds to dog B's frame 8.
    a_frames = {5: {1: (0.0, 0.0, 0.0)}}
    b_frames = {8: {9: (0.0, 0.0, 0.0)}}
    obs_a_by_frame = group_by_frame(_make_observations(a_frames))
    obs_b_by_frame = group_by_frame(_make_observations(b_frames))

    rows = fuse_tracks(obs_a_by_frame, obs_b_by_frame, np.eye(4), lag=3, correspondence={1: 9})

    assert len(rows) == 1
    assert rows[0]["frame"] == 5
    assert rows[0]["source"] == "dog1+dog2"


# --- CSV I/O ------------------------------------------------------------------

def test_load_track_rows_round_trips_write_fused_csv(tmp_path):
    rows = [{
        "frame": 0, "global_id": 1, "dog1_track_id": 1, "dog2_track_id": "",
        "source": "dog1_only", "centroid_x_m": 1.0, "centroid_y_m": 2.0,
        "centroid_z_m": 0.0, "n_points": 42, "speed_m_s": 0.5,
    }]
    output_path = tmp_path / "fused.csv"
    write_fused_csv(rows, output_path)

    with open(output_path) as f:
        read_back = list(csv.DictReader(f))
    assert read_back[0]["global_id"] == "1"
    assert read_back[0]["source"] == "dog1_only"


def test_load_track_rows_parses_types(tmp_path):
    csv_path = tmp_path / "tracks.csv"
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=[
            "track_id", "frame", "time_s", "centroid_x_m", "centroid_y_m",
            "centroid_z_m", "n_points", "speed_m_s", "status"])
        writer.writeheader()
        writer.writerow({
            "track_id": 3, "frame": 7, "time_s": 7.0, "centroid_x_m": 1.5,
            "centroid_y_m": -2.0, "centroid_z_m": 0.3, "n_points": 20,
            "speed_m_s": 0.9, "status": "matched",
        })

    rows = load_track_rows(csv_path)
    assert rows[0]["track_id"] == 3
    assert rows[0]["frame"] == 7
    assert rows[0]["centroid_x_m"] == pytest.approx(1.5)

    observations = observations_from_track_rows(rows)
    assert observations[0].track_id == 3
    assert np.allclose(observations[0].position_m, [1.5, -2.0, 0.3])


# --- plotting (smoke test only -- no visual assertions) ---------------------

def test_group_by_frame_track_sorts_each_track_by_frame():
    obs_by_frame = group_by_frame(_make_observations({
        2: {1: (2.0, 0.0, 0.0)},
        0: {1: (0.0, 0.0, 0.0)},
        1: {1: (1.0, 0.0, 0.0)},
    }))
    by_track = group_by_frame_track(obs_by_frame)
    frames = [o.frame for o in by_track[1]]
    assert frames == [0, 1, 2]


def test_plot_fusion_writes_a_file(tmp_path):
    a_frames = {0: {1: (0.0, 0.0, 0.0)}, 1: {1: (1.0, 0.0, 0.0)}}
    b_frames = {0: {9: (0.0, 0.0, 0.0)}, 1: {9: (1.0, 0.0, 0.0)}}
    obs_a_by_frame = group_by_frame(_make_observations(a_frames))
    obs_b_by_frame = group_by_frame(_make_observations(b_frames))
    fused_rows = fuse_tracks(obs_a_by_frame, obs_b_by_frame, np.eye(4), lag=0, correspondence={1: 9})

    output_path = tmp_path / "trajectories.png"
    plot_fusion(obs_a_by_frame, obs_b_by_frame, np.eye(4), 0, fused_rows, output_path)

    assert output_path.exists()
    assert output_path.stat().st_size > 0
