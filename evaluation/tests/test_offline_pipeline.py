"""
Integration test for offline_pipeline.py -- checks that replaying an
exported PCD + poses session through the real tracking pipeline produces
a track that gets confirmed after a second detection, and none before it.
This exercises the wiring (PCD loading -> voxel downsample -> frame diff ->
clustering -> tracker -> confirmed-track rows), not the individual
algorithm components -- those already have their own direct unit tests in
perception/tests/ (cluster_moved_points, CentroidTracker, the visibility
gate). The visibility gate is disabled here (use_visibility_gate=False) to
keep the synthetic scene simple -- it needs a background surface behind
the moving cluster to demonstrate occlusion, which is exactly what
perception/tests/test_range_image.py already covers directly.
"""
import csv

import numpy as np

from test_helpers import write_binary_pcd
from offline_pipeline import PipelineParams, run_pipeline

# Static background, unchanged across every frame -- present so the
# tracker has something to diff against besides the moving cluster, and so
# the "did this session even wire background points through" question is
# covered too (a background-only frame should produce zero tracks).
_BACKGROUND_MM = np.array([[3000.0 + dx, 3000.0 + dy, 0.0]
                            for dx in range(0, 500, 100) for dy in range(0, 500, 100)])


def _cluster_mm(center_mm, n_side=4, spacing_mm=100.0):
    """A small grid of points centred on center_mm -- enough to clear
    cluster_min_points=10 after voxel downsampling at the default 0.05 m,
    since points are spaced 0.1 m apart."""
    span = (n_side - 1) * spacing_mm / 2.0
    offsets = [(i * spacing_mm - span, j * spacing_mm - span, 0.0)
               for i in range(n_side) for j in range(n_side)]
    return np.array([[center_mm[0] + dx, center_mm[1] + dy, center_mm[2] + dz]
                      for dx, dy, dz in offsets])


def _write_session(tmp_path, frames_mm, frame_timestamps):
    pcd_dir = tmp_path / "pcd"
    pcd_dir.mkdir()
    for i, points_mm in enumerate(frames_mm):
        write_binary_pcd(pcd_dir / f"frame_{i:06d}.pcd", points_mm)

    with open(tmp_path / "poses.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["frame", "timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"])
        for i, stamp in enumerate(frame_timestamps):
            writer.writerow([i, stamp, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    return tmp_path


def test_run_pipeline_confirms_a_track_on_its_second_detection(tmp_path):
    # Frame 0: background only (the "nothing to compare against yet" first
    # frame). Frame 1: background + a cluster at (1, 1, 0) m -- one real
    # detection, not yet confirmed (min_hits=2). Frame 2: background + the
    # same cluster shifted to (1.5, 1.5, 0) m, a plausible ~1 m/s walking
    # displacement over 1 s -- close enough to match the same track via
    # Hungarian assignment (max_match_distance=1.5 m default), which should
    # confirm it.
    frames_mm = [
        _BACKGROUND_MM,
        np.concatenate([_BACKGROUND_MM, _cluster_mm((1000.0, 1000.0, 0.0))]),
        np.concatenate([_BACKGROUND_MM, _cluster_mm((1500.0, 1500.0, 0.0))]),
    ]
    # Non-integer timestamps also verify that output time is real elapsed
    # sensor time, not an accidental alias of the frame index.
    session_dir = _write_session(
        tmp_path, frames_mm, frame_timestamps=[10.0, 10.4, 11.4])

    track_rows, frame_summaries = run_pipeline(
        session_dir, PipelineParams(use_visibility_gate=False))

    assert len(frame_summaries) == 3

    # Not confirmed after frame 1 (only one real detection so far).
    frame_1_rows = [r for r in track_rows if r["frame"] == 1]
    assert frame_1_rows == []

    # Confirmed after frame 2 (second real detection of the same track).
    frame_2_rows = [r for r in track_rows if r["frame"] == 2]
    assert len(frame_2_rows) == 1
    row = frame_2_rows[0]
    assert row["status"] == "matched"
    np.testing.assert_allclose(row["time_s"], 1.4)
    np.testing.assert_allclose([row["centroid_x_m"], row["centroid_y_m"]], [1.5, 1.5], atol=0.1)
    # ~0.5 m displacement over 1 s -> a walking-pace speed, not noise.
    assert 0.3 < row["speed_m_s"] < 1.0


def test_run_pipeline_reports_nothing_for_a_static_scene(tmp_path):
    frames_mm = [_BACKGROUND_MM, _BACKGROUND_MM, _BACKGROUND_MM]
    session_dir = _write_session(tmp_path, frames_mm, frame_timestamps=[0.0, 1.0, 2.0])

    track_rows, _ = run_pipeline(session_dir, PipelineParams(use_visibility_gate=False))

    assert track_rows == []


def test_run_pipeline_can_use_experimental_occlusion_accumulation(tmp_path):
    # The object moves towards the sensor while obscuring the same background
    # patch. Positive range differences therefore persist in nearby angular
    # bins and should provide two detections for normal tracker confirmation.
    frames_mm = [
        _BACKGROUND_MM,
        np.concatenate([
            _BACKGROUND_MM,
            _cluster_mm((2000.0, 2000.0, 0.0)),
        ]),
        np.concatenate([
            _BACKGROUND_MM,
            _cluster_mm((1500.0, 1500.0, 0.0)),
        ]),
    ]
    session_dir = _write_session(
        tmp_path, frames_mm, frame_timestamps=[0.0, 1.0, 2.0])

    track_rows, frame_summaries = run_pipeline(
        session_dir,
        PipelineParams(
            use_occlusion_accumulation=True,
            occlusion_azimuth_bins=72,
            occlusion_elevation_bins=36,
            occlusion_max_gap_bins=2,
            occlusion_activation_threshold=0.2,
            occlusion_activation_range_ratio=0.0,
        ),
    )

    assert "occlusion" in frame_summaries[1]
    assert frame_summaries[1]["occlusion"]["active_bins"] > 0
    assert len(frame_summaries[1]["moved"]) >= 10
    assert any(row["frame"] == 2 for row in track_rows)


def test_occlusion_pipeline_requires_pose_for_every_frame(tmp_path):
    session_dir = _write_session(
        tmp_path,
        [_BACKGROUND_MM, _BACKGROUND_MM],
        frame_timestamps=[0.0, 1.0],
    )
    # Keep the header but remove all pose rows.
    with open(session_dir / "poses.csv", "w", newline="") as handle:
        csv.writer(handle).writerow(
            ["frame", "timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"])

    with np.testing.assert_raises_regex(
        ValueError, "requires an odometry pose"):
        run_pipeline(
            session_dir,
            PipelineParams(use_occlusion_accumulation=True),
        )


def test_run_pipeline_can_use_experimental_free_space_detection(tmp_path):
    # Repeated background returns establish the ray volume in front of a
    # wall as free. A compact object then enters that volume in two
    # successive frames, providing the two measurements needed to confirm
    # a track.
    background = _cluster_mm((3000.0, 0.0, 0.0), n_side=7)
    frames_mm = [
        background,
        background,
        np.concatenate([
            background,
            _cluster_mm((2000.0, 0.0, 0.0)),
        ]),
        np.concatenate([
            background,
            _cluster_mm((1600.0, 0.0, 0.0)),
        ]),
    ]
    session_dir = _write_session(
        tmp_path, frames_mm, frame_timestamps=[0.0, 0.1, 0.2, 0.3])

    track_rows, frame_summaries = run_pipeline(
        session_dir,
        PipelineParams(
            use_free_space_detection=True,
            cluster_min_points=5,
            free_space_burn_in_observations=2,
            free_space_neighbor_connectivity=0,
        ),
    )

    assert "free_space" in frame_summaries[1]
    assert frame_summaries[1]["free_space"]["ever_free_voxels"] > 0
    assert len(frame_summaries[2]["moved"]) >= 5
    assert any(row["frame"] == 3 for row in track_rows)


def test_free_space_pipeline_requires_pose_for_every_frame(tmp_path):
    session_dir = _write_session(
        tmp_path,
        [_BACKGROUND_MM, _BACKGROUND_MM],
        frame_timestamps=[0.0, 1.0],
    )
    with open(session_dir / "poses.csv", "w", newline="") as handle:
        csv.writer(handle).writerow(
            ["frame", "timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"])

    with np.testing.assert_raises_regex(
        ValueError, "requires an odometry pose"):
        run_pipeline(
            session_dir,
            PipelineParams(use_free_space_detection=True),
        )


def test_experimental_detectors_are_mutually_exclusive(tmp_path):
    session_dir = _write_session(
        tmp_path,
        [_BACKGROUND_MM, _BACKGROUND_MM],
        frame_timestamps=[0.0, 1.0],
    )

    with np.testing.assert_raises_regex(ValueError, "mutually exclusive"):
        run_pipeline(
            session_dir,
            PipelineParams(
                use_occlusion_accumulation=True,
                use_free_space_detection=True,
            ),
        )


def test_temporal_consensus_rejects_one_frame_sampling_gap(tmp_path):
    intermittent = _cluster_mm((1000.0, 1000.0, 0.0))
    frames_mm = [
        np.concatenate([_BACKGROUND_MM, intermittent]),
        np.concatenate([_BACKGROUND_MM, intermittent]),
        _BACKGROUND_MM,
        np.concatenate([_BACKGROUND_MM, intermittent]),
    ]
    session_dir = _write_session(
        tmp_path, frames_mm, frame_timestamps=[0.0, 1.0, 2.0, 3.0])

    baseline_tracks, baseline_frames = run_pipeline(
        session_dir, PipelineParams(use_visibility_gate=False))
    consensus_tracks, consensus_frames = run_pipeline(
        session_dir,
        PipelineParams(
            use_visibility_gate=False,
            use_temporal_consensus=True,
            temporal_history_frames=3,
            temporal_min_changed_ratio=0.5,
        ),
    )

    assert len(baseline_frames[3]["moved"]) >= 10
    assert len(consensus_frames[3]["moved"]) == 0
    assert baseline_tracks == consensus_tracks == []


def test_temporal_consensus_preserves_persistent_motion(tmp_path):
    frames_mm = [
        _BACKGROUND_MM,
        _BACKGROUND_MM,
        _BACKGROUND_MM,
        np.concatenate([
            _BACKGROUND_MM,
            _cluster_mm((1000.0, 1000.0, 0.0)),
        ]),
        np.concatenate([
            _BACKGROUND_MM,
            _cluster_mm((1500.0, 1500.0, 0.0)),
        ]),
    ]
    session_dir = _write_session(
        tmp_path, frames_mm, frame_timestamps=[0, 1, 2, 3, 4])

    track_rows, frame_summaries = run_pipeline(
        session_dir,
        PipelineParams(
            use_visibility_gate=False,
            use_temporal_consensus=True,
            temporal_history_frames=3,
            temporal_min_changed_ratio=0.5,
        ),
    )

    assert len(frame_summaries[3]["moved"]) >= 10
    assert any(row["frame"] == 4 for row in track_rows)


def test_temporal_consensus_cannot_wrap_occlusion_accumulator(tmp_path):
    session_dir = _write_session(
        tmp_path,
        [_BACKGROUND_MM, _BACKGROUND_MM],
        frame_timestamps=[0.0, 1.0],
    )

    with np.testing.assert_raises_regex(ValueError, "mutually exclusive"):
        run_pipeline(
            session_dir,
            PipelineParams(
                use_occlusion_accumulation=True,
                use_temporal_consensus=True,
            ),
        )
