#!/usr/bin/env python3
"""Screen the established detector against free-space intrusion detection.

The experimental path adapts Dynablox's central cue--a return entering
high-confidence free space--to a sparse Python voxel/ray map and adds it as a
temporal gate on the established nearest-neighbor/visibility detector. It is
not the upstream TSDF/Voxblox implementation. Both runs use identical PCD
frames, odometry, clustering, and tracking.

Use this only on one-scan exports. A multi-scan PCD has several physical ray
origins but only one exported frame pose, which would invalidate ray casting.
The recordings have no pointwise moving/static labels, so results are screening
measurements rather than accuracy metrics.
"""
import argparse
import json
import sys
import time
from dataclasses import asdict, replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from offline_pipeline import PipelineParams, run_pipeline, write_tracks_csv  # noqa: E402


def summarize(track_rows, frame_summaries, elapsed_s):
    measured_rows = [
        row for row in track_rows if row["status"] != "coasting"]
    moved_counts = [len(frame["moved"]) for frame in frame_summaries]
    diagnostics = [
        frame["free_space"] for frame in frame_summaries
        if "free_space" in frame
    ]
    summary = {
        "frames": len(frame_summaries),
        "runtime_s": elapsed_s,
        "mean_runtime_ms_per_frame": (
            elapsed_s * 1000.0 / len(frame_summaries)
            if frame_summaries else None
        ),
        "moved_points_total": sum(moved_counts),
        "moved_points_mean_per_frame": (
            sum(moved_counts) / len(moved_counts) if moved_counts else 0.0
        ),
        "moved_points_max_frame": max(moved_counts, default=0),
        "confirmed_track_ids": sorted({
            row["track_id"] for row in track_rows
        }),
        "distinct_confirmed_tracks": len({
            row["track_id"] for row in track_rows
        }),
        "measured_confirmed_rows": len(measured_rows),
        "first_confirmed_frame": min(
            (row["frame"] for row in measured_rows), default=None),
    }
    if diagnostics:
        summary["free_space_voxels"] = {
            "ray_voxels_mean_per_frame": (
                sum(item["ray_voxels"] for item in diagnostics)
                / len(diagnostics)
            ),
            "ever_free_voxels_final": diagnostics[-1][
                "ever_free_voxels"],
            "ever_free_voxels_max": max(
                item["ever_free_voxels"] for item in diagnostics),
            "reset_voxels_total": sum(
                item["reset_voxels"] for item in diagnostics),
        }
    return summary


def run_timed(session_dir, params):
    started = time.perf_counter()
    tracks, frames = run_pipeline(session_dir, params)
    return tracks, frames, time.perf_counter() - started


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session_dir", type=Path)
    parser.add_argument(
        "--output-dir", type=Path,
        help="default: output/<session-name>-free-space-comparison")
    parser.add_argument("--voxel", type=float, default=0.05)
    parser.add_argument("--change-threshold", type=float, default=0.05)
    parser.add_argument("--cluster-eps", type=float, default=0.5)
    parser.add_argument("--cluster-min-points", type=int, default=5)
    parser.add_argument("--min-hits", type=int, default=2)
    parser.add_argument("--free-space-voxel-size", type=float, default=0.20)
    parser.add_argument("--min-range", type=float, default=0.5)
    parser.add_argument("--max-range", type=float, default=20.0)
    parser.add_argument("--burn-in-observations", type=int, default=5)
    parser.add_argument("--temporal-buffer-frames", type=int, default=2)
    parser.add_argument(
        "--reset-after-occupied-frames", type=int, default=150)
    parser.add_argument(
        "--neighbor-connectivity", type=int, choices=(0, 6, 26), default=6)
    parser.add_argument("--ray-step-ratio", type=float, default=0.75)
    parser.add_argument(
        "--surface-margin-voxels", type=float, default=1.0)
    args = parser.parse_args()

    if args.voxel <= 0 or args.change_threshold < 0:
        parser.error("voxel must be positive and change threshold non-negative")
    if (
        args.cluster_eps <= 0
        or args.cluster_min_points < 1
        or args.min_hits < 1
    ):
        parser.error("clustering values and min-hits must be positive")

    output_dir = args.output_dir or (
        Path("output") / f"{args.session_dir.name}-free-space-comparison")
    baseline_params = replace(
        PipelineParams(),
        voxel_size=args.voxel,
        change_threshold=args.change_threshold,
        cluster_eps=args.cluster_eps,
        cluster_min_points=args.cluster_min_points,
        min_hits=args.min_hits,
    )
    experimental_params = replace(
        baseline_params,
        use_free_space_detection=True,
        free_space_voxel_size=args.free_space_voxel_size,
        free_space_min_range=args.min_range,
        free_space_max_range=args.max_range,
        free_space_burn_in_observations=args.burn_in_observations,
        free_space_temporal_buffer_frames=args.temporal_buffer_frames,
        free_space_reset_after_occupied_frames=(
            args.reset_after_occupied_frames),
        free_space_neighbor_connectivity=args.neighbor_connectivity,
        free_space_ray_step_ratio=args.ray_step_ratio,
        free_space_surface_margin_voxels=args.surface_margin_voxels,
    )

    baseline_tracks, baseline_frames, baseline_elapsed = run_timed(
        args.session_dir, baseline_params)
    experimental_tracks, experimental_frames, experimental_elapsed = (
        run_timed(args.session_dir, experimental_params)
    )
    report = {
        "session": str(args.session_dir),
        "interpretation": (
            "Screening only: this recording has no pointwise moving-object "
            "ground truth. The free-space path requires one scan and one "
            "pose per frame."
        ),
        "baseline": summarize(
            baseline_tracks, baseline_frames, baseline_elapsed),
        "free_space_detection": summarize(
            experimental_tracks,
            experimental_frames,
            experimental_elapsed,
        ),
        "shared_pipeline_parameters": {
            key: value for key, value in asdict(baseline_params).items()
            if not key.startswith(("occlusion_", "free_space_"))
        },
        "free_space_parameters": {
            key: value
            for key, value in asdict(experimental_params).items()
            if key.startswith("free_space_")
        },
    }

    output_dir.mkdir(parents=True, exist_ok=True)
    write_tracks_csv(baseline_tracks, output_dir / "baseline_tracks.csv")
    write_tracks_csv(
        experimental_tracks, output_dir / "free_space_tracks.csv")
    with (output_dir / "summary.json").open("w") as handle:
        json.dump(report, handle, indent=2)

    print(json.dumps(report, indent=2))
    print(f"wrote comparison to {output_dir}")


if __name__ == "__main__":
    main()
