"""
offline_pipeline.py -- run perception/tracking.py's tracker directly against
an already-exported PCD + poses session, instead of over a live ROS graph.

kei-stuff/ros2-go2/scripts/export_fastlio.py replays a bag through
FastLIO and writes each accumulate-N window as a PCD frame (millimetres,
matching the rest of that pipeline's PCD convention) plus a poses.csv of
per-scan FastLIO odometry (metres, matching ROS convention) -- see that
script and lidar-perception/README.md's "Important: coordinate units" for
the full convention. That output is exactly the input
online_perception_node.py would see frame-by-frame if it were subscribed
live to the same bag replayed through FastLIO: same points, same odometry,
same accumulate window. This module replays that exported sequence through
perception/tracking.py's actual clustering/tracking code (which has no
rclpy dependency -- see tracking.py's own module docstring) instead of
re-deriving the algorithm or standing up a live ROS graph.

Why offline against already-exported frames rather than a fresh live
replay: running the identical tracking code against identical exported input
is a controlled comparison. Both pipelines see exactly the same points and
odometry, so their output differences are attributable to the perception
algorithms rather than two independent FastLIO runs producing slightly
different registration. A later re-check also confirmed that ROS 2 DDS bag
replay works on this machine, so networking is no longer a reason for this
choice.

Units: metres throughout the returned data (matching perception/'s own
convention) -- PCD millimetres are converted on load, immediately.
"""
import csv
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from pcd_io import load_pcd_xyz_mm

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "perception"))
from pointcloud import voxel_downsample
from tracking import CentroidTracker, cluster_moved_points, filter_plausible_detections
from range_image import build_range_image, previously_visible_mask
from occlusion_accumulation import OcclusionAccumulator


@dataclass
class PipelineParams:
    """
    The established detector and tracker fields mirror
    online_perception_node.py's declared ROS parameters and defaults (see
    that module's docstring for what each one does and why). Experimental
    occlusion fields are appended below and remain disabled by default.
    Keeping both configurations here makes an A/B run explicit and prevents
    the established half from quietly drifting away from the live node.
    """
    voxel_size: float = 0.05
    change_threshold: float = 0.15
    cluster_eps: float = 0.5
    cluster_min_points: int = 10
    z_max: float = 2.5
    max_match_distance: float = 1.5
    max_missed_frames: int = 3
    max_missed_seconds: float = 3.0
    min_hits: int = 2
    reid_max_distance: float = 2.0
    reid_window_seconds: float = 15.0
    max_sensor_range: float = 40.0
    use_visibility_gate: bool = True
    range_image_azimuth_bins: int = 72
    range_image_elevation_bins: int = 36
    range_image_tolerance: float = 0.3
    range_image_tolerance_ratio: float = 0.0
    kalman_position_std: float = 0.1
    kalman_velocity_std: float = 2.0
    kalman_process_std: float = 1.0
    # Experimental alternative to nearest-neighbour differencing plus the
    # one-frame visibility gate. Disabled by default so established results
    # and the live node's behaviour are unchanged.
    use_occlusion_accumulation: bool = False
    occlusion_azimuth_bins: int = 72
    occlusion_elevation_bins: int = 36
    occlusion_max_gap_bins: int = 1
    occlusion_completion_range_difference: float = 0.3
    occlusion_reappearance_floor: float = 0.10
    occlusion_reappearance_range_ratio: float = 0.10
    occlusion_activation_threshold: float = 0.30
    occlusion_activation_range_ratio: float = 0.30
    occlusion_point_depth_tolerance: float = 0.50


def load_frame_poses(poses_csv: Path):
    """
    Per-frame odometry position (m) and timestamp (s) -- the last pose row
    within each frame's accumulate-N group, matching
    online_perception_node.py's own convention of reading self._latest_odom
    "as of" the moment a frame's scan buffer fills (close to the end of the
    window, not its start or midpoint).
    """
    rows = list(csv.DictReader(open(poses_csv)))
    by_frame = {}
    for row in rows:
        by_frame.setdefault(int(row["frame"]), []).append(row)

    positions, stamps = {}, {}
    for frame_idx, frame_rows in by_frame.items():
        last = frame_rows[-1]
        positions[frame_idx] = np.array([float(last["x"]), float(last["y"]), float(last["z"])])
        stamps[frame_idx] = float(last["timestamp"])
    return positions, stamps


def run_pipeline(session_dir: Path, params: PipelineParams = PipelineParams()):
    """
    Replay one exported session (a directory containing pcd/frame_*.pcd and
    poses.csv, exactly matching export_fastlio.py's output layout) through
    the current tracking pipeline.

    Returns (track_rows, frame_summaries):
      track_rows: one dict per confirmed track per frame -- track_id,
                  frame, time_s, centroid_x/y/z_m, n_points, speed_m_s,
                  status ("new"/"matched"/"coasting"/"reidentified"). Same
                  shape as the rows online_perception_node.py logs per
                  track per frame, just collected into a list instead of
                  going to the ROS logger.
      frame_summaries: one dict per frame -- frame, points (voxel-
                  downsampled, z-cropped, metres), moved (this frame's
                  points flagged by the selected detector, metres). Used
                  by compare_pipelines.py to draw the point-cloud
                  animation; not needed if you only want the tracks CSV.
    """
    pcd_dir = session_dir / "pcd"
    pcd_paths = sorted(pcd_dir.glob("frame_*.pcd"))
    if not pcd_paths:
        raise FileNotFoundError(f"no frame_*.pcd files under {pcd_dir}")

    odom_positions, odom_stamps = load_frame_poses(session_dir / "poses.csv")

    tracker = CentroidTracker(
        max_match_distance=params.max_match_distance,
        max_missed_frames=params.max_missed_frames,
        max_missed_seconds=params.max_missed_seconds,
        min_hits=params.min_hits,
        position_variance=params.kalman_position_std ** 2,
        velocity_variance=params.kalman_velocity_std ** 2,
        process_variance=params.kalman_process_std ** 2,
        reid_max_distance=params.reid_max_distance,
        reid_window_seconds=params.reid_window_seconds,
    )
    occlusion_accumulator = None
    if params.use_occlusion_accumulation:
        occlusion_accumulator = OcclusionAccumulator(
            azimuth_bins=params.occlusion_azimuth_bins,
            elevation_bins=params.occlusion_elevation_bins,
            max_gap_bins=params.occlusion_max_gap_bins,
            completion_range_difference=(
                params.occlusion_completion_range_difference),
            reappearance_floor=params.occlusion_reappearance_floor,
            reappearance_range_ratio=(
                params.occlusion_reappearance_range_ratio),
            activation_threshold=params.occlusion_activation_threshold,
            activation_range_ratio=params.occlusion_activation_range_ratio,
            point_depth_tolerance=params.occlusion_point_depth_tolerance,
        )

    prev_frame = prev_stamp = prev_position = None
    first_frame_stamp = None
    track_rows = []
    frame_summaries = []

    for frame_idx, pcd_path in enumerate(pcd_paths):
        points_m = load_pcd_xyz_mm(pcd_path) / 1000.0
        points_m = voxel_downsample(points_m, params.voxel_size)
        if params.z_max is not None:
            points_m = points_m[points_m[:, 2] <= params.z_max]

        frame_position = odom_positions.get(frame_idx)
        frame_stamp = odom_stamps.get(frame_idx, float(frame_idx))
        if first_frame_stamp is None:
            first_frame_stamp = frame_stamp

        if prev_frame is None or len(prev_frame) == 0:
            if occlusion_accumulator is not None:
                if frame_position is None:
                    raise ValueError(
                        "occlusion accumulation requires an odometry pose "
                        f"for every frame; frame {frame_idx} has none")
                occlusion_accumulator.update(points_m, frame_position)
            frame_summaries.append({
                "frame": frame_idx,
                "points": points_m,
                "moved": np.empty((0, 3)),
            })
            prev_frame, prev_stamp, prev_position = points_m, frame_stamp, frame_position
            continue

        occlusion_diagnostics = None
        if occlusion_accumulator is not None:
            if frame_position is None:
                raise ValueError(
                    "occlusion accumulation requires an odometry pose "
                    f"for every frame; frame {frame_idx} has none")
            if prev_stamp is not None and frame_stamp < prev_stamp:
                occlusion_accumulator.reset()
            result = occlusion_accumulator.update(points_m, frame_position)
            moved = result.moved_points
            occlusion_diagnostics = {
                "positive_bins": result.n_positive_bins,
                "reappeared_bins": result.n_reappeared_bins,
                "active_bins": result.n_active_bins,
            }
        else:
            tree = cKDTree(prev_frame)
            distances, _ = tree.query(points_m, k=1)
            moved = points_m[distances > params.change_threshold]

            if params.use_visibility_gate and len(moved) > 0 and prev_position is not None:
                prev_range_image = build_range_image(
                    prev_frame, prev_position,
                    params.range_image_azimuth_bins, params.range_image_elevation_bins)
                keep = previously_visible_mask(
                    moved, prev_position, prev_range_image,
                    params.range_image_azimuth_bins, params.range_image_elevation_bins,
                    params.range_image_tolerance,
                    params.range_image_tolerance_ratio)
                moved = moved[keep]

        clusters = cluster_moved_points(moved, params.cluster_eps, params.cluster_min_points)
        if clusters and frame_position is not None:
            clusters = filter_plausible_detections(clusters, frame_position, params.max_sensor_range)

        dt = frame_stamp - prev_stamp if prev_stamp is not None else 0.0
        active_tracks = tracker.step(clusters, dt)

        for track_id, info in active_tracks.items():
            if not info["is_confirmed"]:
                continue
            track = info["track"]
            track_rows.append({
                "track_id": track_id,
                "frame": frame_idx,
                "time_s": float(frame_stamp - first_frame_stamp),
                "centroid_x_m": float(track.position[0]),
                "centroid_y_m": float(track.position[1]),
                "centroid_z_m": float(track.position[2]),
                "n_points": track.n_points,
                "speed_m_s": float(np.linalg.norm(track.velocity)),
                "status": ("new" if info["is_new"] else
                           "reidentified" if info["is_reidentified"] else
                           "coasting" if info["is_coasting"] else "matched"),
            })

        frame_summary = {
            "frame": frame_idx,
            "points": points_m,
            "moved": moved,
        }
        if occlusion_diagnostics is not None:
            frame_summary["occlusion"] = occlusion_diagnostics
        frame_summaries.append(frame_summary)
        prev_frame, prev_stamp, prev_position = points_m, frame_stamp, frame_position

    return track_rows, frame_summaries


def write_tracks_csv(track_rows, output_path: Path) -> None:
    fieldnames = ["track_id", "frame", "time_s", "centroid_x_m", "centroid_y_m",
                  "centroid_z_m", "n_points", "speed_m_s", "status"]
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(track_rows)
