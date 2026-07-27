#!/usr/bin/env python3
"""Evaluate the near-cluster response policy against an exported session.

This consumes the same confirmed track rows produced by ``offline_pipeline``
and the session's per-frame FastLIO poses. It writes a frame-by-frame response
CSV, a compact JSON summary, and a threshold/state plot. Coasted tracks are
excluded to match the live node's ``/online_perception/tracks`` contract.
"""
import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np

from offline_pipeline import load_frame_poses, run_pipeline

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "perception"))
from response_policy import NearClusterStopPolicy  # noqa: E402


def load_track_rows(path: Path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def evaluate_response(
    track_rows,
    odom_positions,
    odom_stamps,
    policy=None,
):
    """Return frame rows and summary for one exported session.

    Track positions and odometry positions are metres in FastLIO's shared map
    frame. Timestamps are seconds. Frames without a current measured track are
    still evaluated as empty observations so clearing behaviour is represented.
    """
    if policy is None:
        policy = NearClusterStopPolicy()
    if not odom_positions:
        raise ValueError("response evaluation requires per-frame odometry")

    tracks_by_frame = {}
    for row in track_rows:
        if row.get("status") == "coasting":
            continue
        frame = int(row["frame"])
        tracks_by_frame.setdefault(frame, []).append([
            float(row["centroid_x_m"]),
            float(row["centroid_y_m"]),
            float(row["centroid_z_m"]),
        ])

    frames = sorted(odom_positions)
    first_stamp = odom_stamps[frames[0]]
    response_rows = []
    stop_transitions = 0
    previous_stop = False
    stop_duration = 0.0

    for index, frame in enumerate(frames):
        stamp = odom_stamps[frame]
        decision = policy.update(
            tracks_by_frame.get(frame, []),
            odom_positions[frame],
            stamp,
        )
        if decision.stop_requested and not previous_stop:
            stop_transitions += 1
        if previous_stop and index > 0:
            previous_frame = frames[index - 1]
            stop_duration += max(0.0, stamp - odom_stamps[previous_frame])
        previous_stop = decision.stop_requested
        response_rows.append({
            "frame": frame,
            "time_s": stamp - first_stamp,
            "nearest_distance_m": decision.nearest_distance,
            "state": decision.state,
            "stop_requested": decision.stop_requested,
            "n_current_tracks": len(tracks_by_frame.get(frame, [])),
        })

    nearest = [
        row["nearest_distance_m"]
        for row in response_rows
        if row["nearest_distance_m"] is not None
    ]
    summary = {
        "frames_evaluated": len(response_rows),
        "duration_s": odom_stamps[frames[-1]] - first_stamp,
        "stop_transitions": stop_transitions,
        "stop_duration_s": stop_duration,
        "minimum_nearest_distance_m": min(nearest) if nearest else None,
        "stop_distance_m": policy.stop_distance,
        "clear_distance_m": policy.clear_distance,
        "trigger_duration_s": policy.trigger_duration,
        "clear_duration_s": policy.clear_duration,
        "distance_mode": "planar" if policy.planar_distance else "3d",
    }
    return response_rows, summary


def write_response_csv(rows, path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "frame",
        "time_s",
        "nearest_distance_m",
        "state",
        "stop_requested",
        "n_current_tracks",
    ]
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def plot_response(rows, summary, path: Path):
    import matplotlib.pyplot as plt

    times = np.array([row["time_s"] for row in rows])
    nearest = np.array([
        np.nan if row["nearest_distance_m"] is None
        else row["nearest_distance_m"]
        for row in rows
    ])
    stopped = np.array([row["stop_requested"] for row in rows], dtype=float)

    fig, (distance_ax, state_ax) = plt.subplots(
        2, 1, figsize=(11, 6), sharex=True,
        gridspec_kw={"height_ratios": [3, 1]},
    )
    distance_ax.plot(times, nearest, ".-", label="nearest current track")
    distance_ax.axhline(
        summary["stop_distance_m"], color="tab:red", linestyle="--",
        label="stop threshold")
    distance_ax.axhline(
        summary["clear_distance_m"], color="tab:green", linestyle=":",
        label="clear threshold")
    distance_ax.set_ylabel("planar distance (m)")
    distance_ax.grid(alpha=0.25)
    distance_ax.legend()

    state_ax.fill_between(times, 0, stopped, step="post", alpha=0.4)
    state_ax.set_yticks([0, 1], labels=["clear", "stop"])
    state_ax.set_xlabel("session time (s)")
    state_ax.set_ylabel("request")
    state_ax.grid(alpha=0.25)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "session", type=Path,
        help="exported session containing pcd/ and poses.csv")
    parser.add_argument(
        "--tracks-csv", type=Path,
        help="existing current-pipeline tracks CSV; otherwise rerun pipeline")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--stop-distance", type=float, default=2.0)
    parser.add_argument("--clear-distance", type=float, default=2.5)
    parser.add_argument("--trigger-duration", type=float, default=1.0)
    parser.add_argument("--clear-duration", type=float, default=1.0)
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()

    session = args.session.resolve()
    if args.tracks_csv:
        track_rows = load_track_rows(args.tracks_csv)
    else:
        track_rows, _ = run_pipeline(session)
    positions, stamps = load_frame_poses(session / "poses.csv")
    policy = NearClusterStopPolicy(
        stop_distance=args.stop_distance,
        clear_distance=args.clear_distance,
        trigger_duration=args.trigger_duration,
        clear_duration=args.clear_duration,
    )
    rows, summary = evaluate_response(track_rows, positions, stamps, policy)

    output_dir = (
        args.output_dir
        if args.output_dir
        else Path(__file__).resolve().parent.parent / "output" / session.name
    )
    write_response_csv(rows, output_dir / "response.csv")
    with open(output_dir / "response_summary.json", "w") as f:
        json.dump(summary, f, indent=2)
        f.write("\n")
    if not args.no_plot:
        plot_response(rows, summary, output_dir / "response.png")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
