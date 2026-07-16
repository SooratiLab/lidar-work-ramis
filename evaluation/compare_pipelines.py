#!/usr/bin/env python3
"""
compare_pipelines.py -- visual + numeric comparison between Kei's offline
track_motion.py output and the current perception/tracking.py pipeline, run
against the same exported PCD + poses session (see offline_pipeline.py for
why this runs offline against an already-exported frame sequence rather
than a fresh live bag replay).

Always produces (under --output-dir, default output/<session name>/):
  <session>_tracks.csv     current pipeline's confirmed tracks, one row per
                            track per frame -- same shape as Kei's own
                            *_tracks.csv, in metres instead of millimetres
  frame_comparison.gif     top-down point cloud per frame with the current
                            pipeline's active track centroids overlaid

Only produced if --kei-tracks is given (a *_tracks.csv from Kei's own
track_motion.py run against the same session, found under
kei-stuff/lidar-perception/output/):
  trajectories.png         top-down path of every track, both pipelines,
                            against the sensor's own path
  speed_profiles.png       speed (m/s) over time per persistent/confirmed
                            track, both pipelines
  summary.png              track-ID-count and per-track persistence bars

Usage:
    python3 evaluation/compare_pipelines.py <session_dir> [options]

Examples:
    # current pipeline only (no reference to compare against)
    python3 evaluation/compare_pipelines.py data/2026-05-13_dds_test_dog1

    # full comparison against Kei's reference run
    python3 evaluation/compare_pipelines.py data/2026-05-12_soton_indoor_dog1 \\
        --kei-tracks ../kei-stuff/lidar-perception/output/2026-05-12_indoor/soton_indoor_dog1_tracks.csv

    # re-tune a parameter and skip the (slower) gif while iterating
    python3 evaluation/compare_pipelines.py data/2026-05-12_fallback_dog1 \\
        --kei-tracks ../kei-stuff/lidar-perception/output/2026-05-12_indoor/fallback_dog1_tracks.csv \\
        --min-hits 3 --no-gif
"""
import argparse
import csv
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from offline_pipeline import PipelineParams, run_pipeline, write_tracks_csv

KEI_COLOUR = "#d62728"    # red
CURRENT_COLOUR = "#1f77b4"  # blue
SENSOR_COLOUR = "#888888"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Compare the current tracking pipeline against Kei's offline reference on one session.")
    parser.add_argument("session", type=Path,
                         help="Session directory containing pcd/frame_*.pcd and poses.csv "
                              "(e.g. data/2026-05-12_soton_indoor_dog1)")
    parser.add_argument("--kei-tracks", type=Path, default=None,
                         help="Kei's *_tracks.csv for the same session (mm) -- enables the "
                              "trajectory/speed/summary comparison plots. Without this, only "
                              "the current pipeline's own tracks CSV + gif are produced.")
    parser.add_argument("--output-dir", type=Path, default=None,
                         help="Output directory (default: output/<session name>/)")
    parser.add_argument("--no-gif", action="store_true",
                         help="Skip the frame-by-frame point cloud animation (the slowest step)")
    parser.add_argument("--gif-fps", type=float, default=2.5,
                         help="Playback rate for frame_comparison.gif (default: 2.5, roughly "
                              "matching a 1s-per-frame accumulate window at less-than-real-time "
                              "for readability)")
    parser.add_argument("--voxel", type=float, default=PipelineParams.voxel_size,
                         help="Voxel downsample size in metres (default: %(default)s)")
    parser.add_argument("--threshold", type=float, default=PipelineParams.change_threshold,
                         help="Moved-point distance threshold in metres (default: %(default)s)")
    parser.add_argument("--eps", type=float, default=PipelineParams.cluster_eps,
                         help="DBSCAN cluster radius in metres (default: %(default)s)")
    parser.add_argument("--min-points", type=int, default=PipelineParams.cluster_min_points,
                         help="DBSCAN minimum cluster size (default: %(default)s)")
    parser.add_argument("--z-max", type=float, default=PipelineParams.z_max,
                         help="Ceiling crop height in metres (default: %(default)s)")
    parser.add_argument("--min-hits", type=int, default=PipelineParams.min_hits,
                         help="Real detections needed before a track is confirmed (default: %(default)s)")
    parser.add_argument("--no-visibility-gate", action="store_true",
                         help="Disable the odometry-referenced visibility gate (range_image.py) "
                              "-- useful for reproducing the moving-sensor false-positive problem "
                              "it fixes, for comparison")
    return parser.parse_args()


def params_from_args(args) -> PipelineParams:
    return PipelineParams(
        voxel_size=args.voxel,
        change_threshold=args.threshold,
        cluster_eps=args.eps,
        cluster_min_points=args.min_points,
        z_max=args.z_max,
        min_hits=args.min_hits,
        use_visibility_gate=not args.no_visibility_gate,
    )


def load_kei_tracks(kei_tracks_csv: Path):
    """
    Kei's track_motion.py has no confirmation step (see tracking.py's
    CentroidTracker docstring for why the current pipeline added
    min_hits) -- every DBSCAN cluster that ever matched across a frame
    pair gets its own permanent track ID in the raw CSV, including
    clusters seen in exactly one frame and never again. Splitting into
    "persistent" (>=2 observations -- what a min_hits=2 gate would have
    confirmed) and "singleton" tracks makes this a fair comparison against
    the current pipeline's already-confirmed-only output, rather than
    comparing a filtered count against an unfiltered one.
    """
    rows = list(csv.DictReader(open(kei_tracks_csv)))
    tracks = {}
    for row in rows:
        tid = int(row["track_id"])
        tracks.setdefault(tid, []).append({
            "frame": int(row["frame"]),
            "time_s": float(row["time_s"]),
            "x": float(row["centroid_x_mm"]) / 1000.0,
            "y": float(row["centroid_y_mm"]) / 1000.0,
            "z": float(row["centroid_z_mm"]) / 1000.0,
            "speed": float(row["speed_m_s"]),
            "n_points": int(row["n_points"]),
        })
    persistent = {tid: obs for tid, obs in tracks.items() if len(obs) >= 2}
    singleton = {tid: obs for tid, obs in tracks.items() if len(obs) < 2}
    return persistent, singleton


def load_current_tracks(track_rows):
    tracks = {}
    for row in track_rows:
        tracks.setdefault(row["track_id"], []).append({
            "frame": row["frame"], "time_s": row["time_s"],
            "x": row["centroid_x_m"], "y": row["centroid_y_m"], "z": row["centroid_z_m"],
            "speed": row["speed_m_s"], "n_points": row["n_points"], "status": row["status"],
        })
    return tracks


def load_sensor_path(poses_csv: Path):
    """
    Returns an (N, 2) array of the sensor's x/y path, or an empty (0, 2)
    array if poses.csv has no rows -- confirmed on the fallback_dog1
    session (87 PCD frames, but export_fastlio.py wrote no pose rows at
    all for that run; fallback_dog2 from the same session has the full
    931). Plotting code below skips the sensor-path overlay rather than
    failing on an empty array, matching online_perception_node.py's own
    fail-open behaviour when no /Odometry has arrived (see its
    _apply_visibility_gate/_drop_implausible_clusters docstrings) --
    the pipeline still runs, just without odometry-gated filtering.
    """
    rows = list(csv.DictReader(open(poses_csv)))
    if not rows:
        return np.empty((0, 2))
    return np.array([[float(r["x"]), float(r["y"])] for r in rows])


def plot_trajectories(kei_persistent, kei_singleton, current_tracks, sensor_path, session_name, out_dir):
    # Per-track text labels stop being readable well before a session with
    # ~150 "persistent" tracks that are mostly spurious (see
    # plot_speed_profiles's MAX_LEGEND_ENTRIES note -- same underlying
    # sessions hit this) -- past this many, skip the annotation/start/end
    # markers and let the raw density of criss-crossing lines make the
    # point instead.
    MAX_LABELLED_TRACKS = 20

    fig, axes = plt.subplots(1, 2, figsize=(14, 7), sharex=True, sharey=True)
    has_sensor_path = len(sensor_path) > 0

    ax = axes[0]
    if has_sensor_path:
        ax.plot(sensor_path[:, 0], sensor_path[:, 1], color=SENSOR_COLOUR,
                linewidth=1, linestyle="--", label="sensor (dog) path", zorder=1)
        ax.scatter(sensor_path[0, 0], sensor_path[0, 1], color=SENSOR_COLOUR,
                   marker="s", s=60, zorder=2, label="sensor start")
    if kei_singleton:
        noise_xy = np.array([(o[0]["x"], o[0]["y"]) for o in kei_singleton.values()])
        ax.scatter(noise_xy[:, 0], noise_xy[:, 1], color="orange", marker="x",
                   s=40, zorder=3, label=f"single-frame noise ({len(kei_singleton)})", alpha=0.5)
    label_kei = len(kei_persistent) <= MAX_LABELLED_TRACKS
    for tid, obs in sorted(kei_persistent.items()):
        xs, ys = [o["x"] for o in obs], [o["y"] for o in obs]
        ax.plot(xs, ys, color=KEI_COLOUR, marker="o", markersize=4 if label_kei else 2,
                linewidth=2 if label_kei else 0.8, alpha=1.0 if label_kei else 0.5, zorder=4)
        if label_kei:
            ax.annotate(f"track {tid}", (xs[0], ys[0]), color=KEI_COLOUR, fontsize=9,
                        fontweight="bold", xytext=(5, 5), textcoords="offset points")
            ax.scatter(xs[0], ys[0], color=KEI_COLOUR, marker="^", s=70, zorder=5)
            ax.scatter(xs[-1], ys[-1], color=KEI_COLOUR, marker="s", s=70, zorder=5)
    ax.set_title("Kei's offline pipeline (track_motion.py)\n"
                 f"{len(kei_persistent)} persistent track(s) (\u22652 obs) + "
                 f"{len(kei_singleton)} single-frame noise ID(s), no confirmation step")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_aspect("equal"); ax.grid(True, alpha=0.3)
    if ax.get_legend_handles_labels()[0]:
        ax.legend(loc="upper left", fontsize=8)

    ax = axes[1]
    if has_sensor_path:
        ax.plot(sensor_path[:, 0], sensor_path[:, 1], color=SENSOR_COLOUR,
                linewidth=1, linestyle="--", label="sensor (dog) path", zorder=1)
        ax.scatter(sensor_path[0, 0], sensor_path[0, 1], color=SENSOR_COLOUR,
                   marker="s", s=60, zorder=2, label="sensor start")
    label_current = len(current_tracks) <= MAX_LABELLED_TRACKS
    for tid, obs in sorted(current_tracks.items()):
        xs, ys = [o["x"] for o in obs], [o["y"] for o in obs]
        ax.plot(xs, ys, color=CURRENT_COLOUR, marker="o", markersize=4 if label_current else 2,
                linewidth=2 if label_current else 0.8, alpha=1.0 if label_current else 0.5, zorder=3)
        if label_current:
            ax.annotate(f"track {tid}", (xs[0], ys[0]), color=CURRENT_COLOUR, fontsize=9,
                        fontweight="bold", xytext=(5, 5), textcoords="offset points")
            ax.scatter(xs[0], ys[0], color=CURRENT_COLOUR, marker="^", s=70, zorder=4)
            ax.scatter(xs[-1], ys[-1], color=CURRENT_COLOUR, marker="s", s=70, zorder=4)
    ax.set_title("Current pipeline (tracking.py)\n"
                 f"{len(current_tracks)} confirmed track(s), min_hits + visibility gate already applied")
    ax.set_xlabel("x (m)")
    ax.set_aspect("equal"); ax.grid(True, alpha=0.3)
    if ax.get_legend_handles_labels()[0]:
        ax.legend(loc="upper left", fontsize=8)

    fig.suptitle(f"{session_name} -- track trajectories (\u25b2 first observation, \u25a0 last)", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    out_path = out_dir / "trajectories.png"
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"wrote {out_path}")


def plot_speed_profiles(kei_persistent, current_tracks, session_name, out_dir):
    # A per-track legend stops being useful well before a session like
    # lab_walk_with_stops's ~160 persistent-but-mostly-spurious tracks
    # (see DOCS.md's "Testing against more sessions" -- this is the same
    # session, still producing hundreds of raw track IDs with no
    # confirmation step) -- past this many entries matplotlib can't even
    # fit the legend in the figure, and every colour repeats anyway.
    MAX_LEGEND_ENTRIES = 20

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    for ax, tracks, title in (
        (axes[0], kei_persistent, "Kei's offline pipeline (persistent tracks only)"),
        (axes[1], current_tracks, "Current pipeline (confirmed tracks)"),
    ):
        show_legend = len(tracks) <= MAX_LEGEND_ENTRIES
        for tid, obs in sorted(tracks.items()):
            times = [o["time_s"] for o in obs]
            speeds = [o["speed"] for o in obs]
            label = f"track {tid}" if show_legend else None
            ax.plot(times, speeds, marker="o", markersize=3, alpha=0.6 if not show_legend else 1.0,
                    label=label)
        ax.axhspan(0.8, 1.8, color="green", alpha=0.08, label="typical walking pace")
        ax.set_title(f"{title} -- speed per track ({len(tracks)} tracks)")
        ax.set_ylabel("speed (m/s)")
        ax.grid(True, alpha=0.3)
        if show_legend:
            ax.legend(fontsize=8, loc="upper right")
        else:
            ax.legend(handles=[], labels=[], loc="upper right")  # keep only the walking-pace band
            ax.text(0.99, 0.95, f"{len(tracks)} tracks -- too many to label individually",
                    transform=ax.transAxes, ha="right", va="top", fontsize=8, style="italic")

    axes[1].set_xlabel("time (s)")
    fig.suptitle(session_name, fontsize=12)
    fig.tight_layout()
    out_path = out_dir / "speed_profiles.png"
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"wrote {out_path}")


def plot_summary(kei_persistent, kei_singleton, current_tracks, session_name, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))

    labels = ["Kei's offline\npipeline", "Current\npipeline"]
    real_counts = [len(kei_persistent), len(current_tracks)]
    noise_counts = [len(kei_singleton), 0]

    axes[0].bar(labels, real_counts, color=[KEI_COLOUR, CURRENT_COLOUR],
                label="persistent/confirmed tracks")
    axes[0].bar(labels, noise_counts, bottom=real_counts, color="orange",
                label="single-frame noise IDs")
    axes[0].set_ylabel("distinct track IDs reported")
    axes[0].set_title("Track IDs reported\n(lower noise bar = fewer false tracks to filter by hand)")
    axes[0].legend(fontsize=7)
    for i, (r, n) in enumerate(zip(real_counts, noise_counts)):
        axes[0].text(i, r + n + 0.3, str(r + n), ha="center", fontweight="bold")

    kei_spans = [len(obs) for obs in kei_persistent.values()]
    current_spans = [len(obs) for obs in current_tracks.values()]
    axes[1].boxplot([kei_spans, current_spans], labels=labels)
    axes[1].set_ylabel("frames observed per persistent/confirmed track")
    axes[1].set_title("Per-track persistence\n(both pipelines' real tracks, noise excluded)")

    fig.suptitle(session_name, fontsize=12)
    fig.tight_layout()
    out_path = out_dir / "summary.png"
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"wrote {out_path}")


def make_frame_comparison_gif(frame_summaries, current_tracks, session_name, out_dir, fps):
    """
    Frame-by-frame top-down point cloud (grey) with the current pipeline's
    confirmed track centroids overlaid (coloured, labelled). Single-
    pipeline, not side-by-side with Kei's output -- his track_motion.py
    output has no equivalent per-frame point cloud on hand without
    re-running its own playback.py/render_deliverable.py path, and the
    numeric comparison in trajectories.png/speed_profiles.png already
    covers the actual pipeline-vs-pipeline comparison.
    """
    by_frame_tracks = {}
    for tid, obs in current_tracks.items():
        for o in obs:
            by_frame_tracks.setdefault(o["frame"], []).append((tid, o))

    all_points = np.concatenate([s["points"] for s in frame_summaries if len(s["points"])])
    limit = float(np.abs(all_points[:, :2]).max()) + 1.0 if len(all_points) else 8.0

    fig, ax = plt.subplots(figsize=(7, 7))

    def update(i):
        ax.clear()
        summary = frame_summaries[i]
        pts = summary["points"]
        if len(pts):
            ax.scatter(pts[:, 0], pts[:, 1], s=1, color="lightgrey", zorder=1)
        for tid, o in by_frame_tracks.get(summary["frame"], []):
            colour = plt.cm.tab10(tid % 10)
            marker = "o" if o["status"] != "coasting" else "x"
            ax.scatter(o["x"], o["y"], color=colour, s=120, marker=marker, zorder=3)
            ax.annotate(f"track {tid}\n{o['speed']:.2f} m/s", (o["x"], o["y"]),
                        color=colour, fontsize=8, fontweight="bold",
                        xytext=(6, 6), textcoords="offset points")
        ax.set_xlim(-limit, limit)
        ax.set_ylim(-limit, limit)
        ax.set_aspect("equal")
        ax.set_title(f"{session_name} -- current pipeline -- frame {summary['frame']}")
        ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
        ax.grid(True, alpha=0.3)

    anim = animation.FuncAnimation(fig, update, frames=len(frame_summaries), interval=1000 / fps)
    out_path = out_dir / "frame_comparison.gif"
    anim.save(out_path, writer="pillow", fps=fps)
    plt.close(fig)
    print(f"wrote {out_path}")


def main():
    args = parse_args()
    session_name = args.session.name
    out_dir = args.output_dir or Path("output") / session_name
    out_dir.mkdir(parents=True, exist_ok=True)

    params = params_from_args(args)
    print(f"running current pipeline against {args.session} "
          f"(eps={params.cluster_eps} min_points={params.cluster_min_points} "
          f"threshold={params.change_threshold} z_max={params.z_max} "
          f"min_hits={params.min_hits} visibility_gate={params.use_visibility_gate})")
    track_rows, frame_summaries = run_pipeline(args.session, params)
    current_tracks = load_current_tracks(track_rows)

    tracks_csv_path = out_dir / f"{session_name}_tracks.csv"
    write_tracks_csv(track_rows, tracks_csv_path)
    print(f"wrote {tracks_csv_path} ({len(track_rows)} rows, "
          f"{len(current_tracks)} confirmed track(s))")

    if args.kei_tracks:
        kei_persistent, kei_singleton = load_kei_tracks(args.kei_tracks)
        sensor_path = load_sensor_path(args.session / "poses.csv")
        if len(sensor_path) == 0:
            print(f"note: {args.session}/poses.csv has no rows -- this export has no odometry "
                  f"at all (confirmed not a copying artifact, see DOCS.md's data inventory notes). "
                  f"Trajectory plot will omit the sensor path; the visibility/plausibility gates "
                  f"ran fail-open for this session, same as online_perception_node.py would.")
        plot_trajectories(kei_persistent, kei_singleton, current_tracks, sensor_path, session_name, out_dir)
        plot_speed_profiles(kei_persistent, current_tracks, session_name, out_dir)
        plot_summary(kei_persistent, kei_singleton, current_tracks, session_name, out_dir)
        print(f"\n--- {session_name} summary ---")
        print(f"Kei's pipeline:   {len(kei_persistent)} persistent tracks (\u22652 obs) + "
              f"{len(kei_singleton)} single-frame noise IDs "
              f"({len(kei_persistent) + len(kei_singleton)} total track IDs reported)")
        print(f"Current pipeline: {len(current_tracks)} confirmed tracks, "
              f"0 unconfirmed tracks reported (filtered internally by min_hits)")
    else:
        print("no --kei-tracks given -- skipping trajectories.png/speed_profiles.png/summary.png")

    if not args.no_gif:
        make_frame_comparison_gif(frame_summaries, current_tracks, session_name, out_dir, args.gif_fps)
    else:
        print("--no-gif given -- skipping frame_comparison.gif")


if __name__ == "__main__":
    main()
