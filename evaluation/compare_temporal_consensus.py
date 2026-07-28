#!/usr/bin/env python3
"""Compare one-frame change detection with multi-frame temporal consensus.

The experiment adapts the historical voting shared by DOF-LIO and PGP-DOR to
this project's existing world-frame nearest-neighbor detector. Both runs use
identical visibility gating, clustering, tracking, and odometry. Recorded
sessions have no pointwise moving/static labels, so this is a screening
comparison rather than an accuracy benchmark.
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
    return {
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


def run_timed(session_dir, params):
    started = time.perf_counter()
    tracks, frames = run_pipeline(session_dir, params)
    return tracks, frames, time.perf_counter() - started


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session_dir", type=Path)
    parser.add_argument(
        "--output-dir", type=Path,
        help="default: output/<session-name>-temporal-consensus")
    parser.add_argument("--voxel", type=float, default=0.05)
    parser.add_argument("--change-threshold", type=float, default=0.15)
    parser.add_argument("--cluster-eps", type=float, default=0.5)
    parser.add_argument("--cluster-min-points", type=int, default=10)
    parser.add_argument("--min-hits", type=int, default=2)
    parser.add_argument("--history-frames", type=int, default=3)
    parser.add_argument("--min-changed-ratio", type=float, default=0.5)
    args = parser.parse_args()

    if args.voxel <= 0 or args.change_threshold < 0:
        parser.error("voxel must be positive and change threshold non-negative")
    if (
        args.cluster_eps <= 0
        or args.cluster_min_points < 1
        or args.min_hits < 1
    ):
        parser.error("clustering values and min-hits must be positive")
    if args.history_frames < 2:
        parser.error("history-frames must be at least 2")
    if not 0 < args.min_changed_ratio <= 1:
        parser.error("min-changed-ratio must be in (0, 1]")

    output_dir = args.output_dir or (
        Path("output") / f"{args.session_dir.name}-temporal-consensus")
    baseline_params = replace(
        PipelineParams(),
        voxel_size=args.voxel,
        change_threshold=args.change_threshold,
        cluster_eps=args.cluster_eps,
        cluster_min_points=args.cluster_min_points,
        min_hits=args.min_hits,
    )
    consensus_params = replace(
        baseline_params,
        use_temporal_consensus=True,
        temporal_history_frames=args.history_frames,
        temporal_min_changed_ratio=args.min_changed_ratio,
    )

    baseline_tracks, baseline_frames, baseline_elapsed = run_timed(
        args.session_dir, baseline_params)
    consensus_tracks, consensus_frames, consensus_elapsed = run_timed(
        args.session_dir, consensus_params)
    report = {
        "session": str(args.session_dir),
        "interpretation": (
            "Screening only: this recording has no pointwise moving-object "
            "ground truth."
        ),
        "baseline": summarize(
            baseline_tracks, baseline_frames, baseline_elapsed),
        "temporal_consensus": summarize(
            consensus_tracks, consensus_frames, consensus_elapsed),
        "shared_pipeline_parameters": {
            key: value for key, value in asdict(baseline_params).items()
            if not key.startswith("temporal_")
            and key != "use_temporal_consensus"
        },
        "temporal_parameters": {
            "history_frames": args.history_frames,
            "min_changed_ratio": args.min_changed_ratio,
        },
    }

    output_dir.mkdir(parents=True, exist_ok=True)
    write_tracks_csv(baseline_tracks, output_dir / "baseline_tracks.csv")
    write_tracks_csv(
        consensus_tracks, output_dir / "temporal_consensus_tracks.csv")
    with (output_dir / "summary.json").open("w") as handle:
        json.dump(report, handle, indent=2)

    print(json.dumps(report, indent=2))
    print(f"wrote comparison to {output_dir}")


if __name__ == "__main__":
    main()
