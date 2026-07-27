#!/usr/bin/env python3
"""Compare the established detector with experimental occlusion accumulation.

Both runs use the same exported world-frame PCDs, odometry, DBSCAN settings,
and tracker. Only the moving-point detector changes. This makes the output a
useful screening experiment, not ground-truth accuracy: the recorded sessions
do not contain pointwise moving/static labels.
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
    confirmed_ids = sorted({row["track_id"] for row in track_rows})
    first_confirmed = min(
        (row["frame"] for row in measured_rows), default=None)
    moved_counts = [len(frame["moved"]) for frame in frame_summaries]
    summary = {
        "frames": len(frame_summaries),
        "runtime_s": elapsed_s,
        "mean_runtime_ms_per_frame": (
            elapsed_s * 1000.0 / len(frame_summaries)
            if frame_summaries else None
        ),
        "moved_points_total": sum(moved_counts),
        "moved_points_mean_per_frame": (
            sum(moved_counts) / len(moved_counts) if moved_counts else 0.0),
        "moved_points_max_frame": max(moved_counts, default=0),
        "confirmed_track_ids": confirmed_ids,
        "distinct_confirmed_tracks": len(confirmed_ids),
        "measured_confirmed_rows": len(measured_rows),
        "first_confirmed_frame": first_confirmed,
    }
    diagnostics = [
        frame["occlusion"] for frame in frame_summaries
        if "occlusion" in frame
    ]
    if diagnostics:
        summary["occlusion_bins"] = {
            key: {
                "total": sum(item[key] for item in diagnostics),
                "max_frame": max(item[key] for item in diagnostics),
            }
            for key in ("positive_bins", "reappeared_bins", "active_bins")
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
        help="default: output/<session-name>-occlusion-comparison")
    parser.add_argument("--azimuth-bins", type=int, default=72)
    parser.add_argument("--elevation-bins", type=int, default=36)
    parser.add_argument("--max-gap-bins", type=int, default=1)
    parser.add_argument(
        "--completion-range-difference", type=float, default=0.3)
    parser.add_argument("--contribution-floor", type=float, default=0.05)
    parser.add_argument(
        "--contribution-range-ratio", type=float, default=0.005)
    parser.add_argument("--reappearance-floor", type=float, default=0.10)
    parser.add_argument(
        "--reappearance-range-ratio", type=float, default=0.01)
    parser.add_argument("--activation-threshold", type=float, default=0.30)
    parser.add_argument("--activation-range-ratio", type=float, default=0.60)
    parser.add_argument("--point-depth-tolerance", type=float, default=0.50)
    args = parser.parse_args()

    output_dir = args.output_dir or (
        Path("output") / f"{args.session_dir.name}-occlusion-comparison")
    baseline_params = PipelineParams()
    experimental_params = replace(
        baseline_params,
        use_occlusion_accumulation=True,
        occlusion_azimuth_bins=args.azimuth_bins,
        occlusion_elevation_bins=args.elevation_bins,
        occlusion_max_gap_bins=args.max_gap_bins,
        occlusion_completion_range_difference=(
            args.completion_range_difference),
        occlusion_contribution_floor=args.contribution_floor,
        occlusion_contribution_range_ratio=args.contribution_range_ratio,
        occlusion_reappearance_floor=args.reappearance_floor,
        occlusion_reappearance_range_ratio=args.reappearance_range_ratio,
        occlusion_activation_threshold=args.activation_threshold,
        occlusion_activation_range_ratio=args.activation_range_ratio,
        occlusion_point_depth_tolerance=args.point_depth_tolerance,
    )

    baseline_tracks, baseline_frames, baseline_elapsed = run_timed(
        args.session_dir, baseline_params)
    experimental_tracks, experimental_frames, experimental_elapsed = run_timed(
        args.session_dir, experimental_params)
    baseline_summary = summarize(
        baseline_tracks, baseline_frames, baseline_elapsed)
    experimental_summary = summarize(
        experimental_tracks, experimental_frames, experimental_elapsed)

    output_dir.mkdir(parents=True, exist_ok=True)
    write_tracks_csv(baseline_tracks, output_dir / "baseline_tracks.csv")
    write_tracks_csv(
        experimental_tracks, output_dir / "occlusion_tracks.csv")
    report = {
        "session": str(args.session_dir),
        "interpretation": (
            "Screening comparison only: these recordings have no pointwise "
            "moving-object ground truth."
        ),
        "baseline": baseline_summary,
        "occlusion_accumulation": experimental_summary,
        "occlusion_parameters": {
            key: value for key, value in asdict(experimental_params).items()
            if key.startswith("occlusion_")
        },
    }
    with (output_dir / "summary.json").open("w") as handle:
        json.dump(report, handle, indent=2)

    print(json.dumps(report, indent=2))
    print(f"wrote comparison to {output_dir}")


if __name__ == "__main__":
    main()
