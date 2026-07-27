#!/usr/bin/env python3
"""A/B test fixed and range-adaptive visibility-gate tolerances.

Both runs use the same exported points, odometry, clustering, and tracking.
The experiment changes only the visibility tolerance from a fixed metre value
to ``max(floor_m, ratio * candidate_range_m)``. Recorded sessions have no
pointwise labels, so outputs are screening diagnostics rather than accuracy.
"""
import argparse
import json
import sys
import time
from dataclasses import asdict, replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from offline_pipeline import PipelineParams, run_pipeline, write_tracks_csv  # noqa: E402


def _run_and_summarize(session_dir, params):
    started = time.perf_counter()
    track_rows, frames = run_pipeline(session_dir, params)
    elapsed = time.perf_counter() - started
    measured = [row for row in track_rows if row["status"] != "coasting"]
    moved_counts = [len(frame["moved"]) for frame in frames]
    return track_rows, {
        "frames": len(frames),
        "runtime_s": elapsed,
        "mean_runtime_ms_per_frame": (
            elapsed * 1000.0 / len(frames) if frames else None),
        "moved_points_total": sum(moved_counts),
        "moved_points_max_frame": max(moved_counts, default=0),
        "distinct_confirmed_tracks": len({
            row["track_id"] for row in track_rows}),
        "measured_confirmed_rows": len(measured),
        "first_confirmed_frame": min(
            (row["frame"] for row in measured), default=None),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session_dir", type=Path)
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="default: output/<session-name>-visibility-tolerance",
    )
    parser.add_argument("--tolerance-floor", type=float, default=0.3)
    parser.add_argument("--tolerance-range-ratio", type=float, default=0.02)
    args = parser.parse_args()
    if args.tolerance_floor < 0 or args.tolerance_range_ratio < 0:
        parser.error("visibility tolerances must be non-negative")

    output_dir = args.output_dir or (
        Path("output") / f"{args.session_dir.name}-visibility-tolerance")
    baseline_params = replace(
        PipelineParams(),
        range_image_tolerance=args.tolerance_floor,
        range_image_tolerance_ratio=0.0,
    )
    adaptive_params = replace(
        baseline_params,
        range_image_tolerance_ratio=args.tolerance_range_ratio,
    )

    baseline_rows, baseline_summary = _run_and_summarize(
        args.session_dir, baseline_params)
    adaptive_rows, adaptive_summary = _run_and_summarize(
        args.session_dir, adaptive_params)

    output_dir.mkdir(parents=True, exist_ok=True)
    write_tracks_csv(baseline_rows, output_dir / "fixed_tracks.csv")
    write_tracks_csv(adaptive_rows, output_dir / "adaptive_tracks.csv")
    report = {
        "session": str(args.session_dir),
        "interpretation": (
            "Screening comparison only: recordings have no pointwise "
            "moving-object ground truth."
        ),
        "fixed_tolerance": baseline_summary,
        "range_adaptive_tolerance": adaptive_summary,
        "adaptive_parameters": {
            key: value for key, value in asdict(adaptive_params).items()
            if key.startswith("range_image_")
        },
    }
    with (output_dir / "summary.json").open("w") as handle:
        json.dump(report, handle, indent=2)

    print(json.dumps(report, indent=2))
    print(f"wrote comparison to {output_dir}")


if __name__ == "__main__":
    main()
