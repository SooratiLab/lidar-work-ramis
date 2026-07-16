#!/usr/bin/env python3
"""
track_fusion.py -- fuse two dogs' independently-tracked objects into one
shared-frame track log, instead of merging raw point clouds.

Point-cloud-level merging (kei-stuff/lidar-perception/scripts/icp_merge.py,
merge_with_prior.py) was tried on real two-dog field data and didn't work
reliably: each dog's FastLIO runs in its own unregistered world frame,
overlap between what the two dogs actually see is often low, and FastLIO's
sparse output (thousands of points per frame, not the hundreds of
thousands a stationary bench scan produces) starves feature-based
registration of what it needs. None of that changes by moving to
track-level fusion, but the object of registration does: instead of
aligning two dense, noisy point clouds, this aligns two short lists of
already-tracked object positions (perception/tracking.py's confirmed
tracks) -- far sparser, but far more semantically meaningful, since a
"track" is already the thing both dogs are independently trying to agree
on (a person, not a point).

The approach, in three pieces:

1. Time alignment. Each dog's track log timestamps are frame indices, not
   wall-clock time (see evaluation/offline_pipeline.py's run_pipeline --
   "time_s" is float(frame_idx)), so this searches over a small integer
   frame-lag window instead of trusting the two dogs' recordings to have
   started on exactly the same accumulate-window boundary.
2. Spatial calibration. Starting from a supplied prior (a one-off
   deployment measurement, or identity if genuinely unknown -- see
   parse_offset_string), refine it against whichever tracks the two dogs
   actually agree are the same object at a given frame. This is the
   track-level analogue of ICP-with-prior: instead of iterating
   nearest-point correspondences over dense clouds, it iterates
   nearest-track correspondences over confirmed detections.
3. Fusion. Once tracks are corresponded, build one row per frame per
   global object: a fused position (averaged, if both dogs currently see
   it) or a single dog's own position (if only one currently does -- the
   complementary-coverage case Kei's cardboard-box session demonstrated
   informally, one dog blind behind an obstruction the other isn't).

Both spatial calibration and fusion can end up with no usable
correspondence at all -- e.g. the two dogs never actually saw the same
object, or the supplied prior is wrong by more than max_distance. That's
reported honestly (empty correspondence, calibration returns the prior
unchanged) rather than producing something that looks like a refined
answer with nothing behind it.

Units: metres and seconds throughout, matching perception/tracking.py's
own convention. This module never touches PCD files or the millimetre
convention those use.
"""
import argparse
import csv
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "perception"))
from tracking import assign_detections  # noqa: E402  (path set up above)


# ---------------------------------------------------------------------------
# Calibration transform: translation (m) + yaw only, matching
# merge_with_prior.py's own "only yaw matters on level ground" assumption,
# carried over here for track centroids instead of raw points.
# ---------------------------------------------------------------------------

def rotation_matrix_yaw(yaw_deg: float) -> np.ndarray:
    """3x3 rotation about the vertical (z) axis."""
    yaw = np.radians(yaw_deg)
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def build_transform(x_m: float, y_m: float, z_m: float, yaw_deg: float) -> np.ndarray:
    """
    4x4 homogeneous transform, translation in METRES.

    kei-stuff/lidar-perception/scripts/merge_with_prior.py's build_transform
    takes millimetres, matching the PCD files it operates on -- this
    module never touches a PCD file, only perception/tracking.py's
    metre-based track centroids, so there is no millimetre convention to
    match here. Keep this straight if the two are ever compared side by
    side.
    """
    T = np.eye(4)
    T[:3, :3] = rotation_matrix_yaw(yaw_deg)
    T[:3, 3] = [x_m, y_m, z_m]
    return T


def parse_offset_string(s: str) -> np.ndarray:
    """Parse 'x_m,y_m,z_m,yaw_deg' (a one-off deployment measurement, or a
    previously-saved calibration result) into a 4x4 transform."""
    parts = [p.strip() for p in s.split(",")]
    if len(parts) != 4:
        raise ValueError(
            f"--offset must be 'x_m,y_m,z_m,yaw_deg' (got {len(parts)} comma-separated values)")
    return build_transform(*(float(p) for p in parts))


def apply_transform(positions_m: np.ndarray, transform: np.ndarray) -> np.ndarray:
    """Apply a 4x4 transform to an (N, 3) array of positions."""
    if len(positions_m) == 0:
        return positions_m.reshape(0, 3)
    homogeneous = np.hstack([positions_m, np.ones((len(positions_m), 1))])
    return (transform @ homogeneous.T).T[:, :3]


# ---------------------------------------------------------------------------
# Track log loading -- the shape evaluation/offline_pipeline.py's
# write_tracks_csv already produces (track_id, frame, centroid_x/y/z_m,
# n_points, speed_m_s, status). Read directly here rather than importing
# that module, so the pure fusion logic below has no dependency on
# perception/evaluation beyond assign_detections -- a CSV in this shape
# from any source works, not just this repo's own pipeline.
# ---------------------------------------------------------------------------

@dataclass
class Observation:
    """One confirmed track's position at one frame, in its own dog's frame
    (untransformed) -- the atomic unit both calibration and fusion work
    with."""
    frame: int
    track_id: int
    position_m: np.ndarray
    n_points: int
    speed_m_s: float


def load_track_rows(csv_path: Path) -> list:
    """Read a track-log CSV in evaluation/offline_pipeline.write_tracks_csv's
    format. Returns a list of dicts with the same fields, numeric columns
    converted from string."""
    rows = []
    with open(csv_path, newline="") as f:
        for row in csv.DictReader(f):
            rows.append({
                "track_id": int(row["track_id"]),
                "frame": int(row["frame"]),
                "centroid_x_m": float(row["centroid_x_m"]),
                "centroid_y_m": float(row["centroid_y_m"]),
                "centroid_z_m": float(row["centroid_z_m"]),
                "n_points": int(row["n_points"]),
                "speed_m_s": float(row["speed_m_s"]),
                "status": row["status"],
            })
    return rows


def observations_from_track_rows(track_rows: list) -> list:
    """Convert loaded/in-memory track rows (see load_track_rows, or
    evaluation/offline_pipeline.run_pipeline's own return value directly)
    into Observations."""
    return [
        Observation(
            frame=row["frame"],
            track_id=row["track_id"],
            position_m=np.array([row["centroid_x_m"], row["centroid_y_m"], row["centroid_z_m"]]),
            n_points=row["n_points"],
            speed_m_s=row["speed_m_s"],
        )
        for row in track_rows
    ]


def group_by_frame(observations: list) -> dict:
    """{frame_index: [Observation, ...]}."""
    by_frame = {}
    for obs in observations:
        by_frame.setdefault(obs.frame, []).append(obs)
    return by_frame


# ---------------------------------------------------------------------------
# Frame-lag search
# ---------------------------------------------------------------------------

def _match_at_lag(obs_a_by_frame: dict, obs_b_by_frame: dict, transform: np.ndarray,
                   lag: int, max_distance: float) -> list:
    """
    Per-frame Hungarian matches between dog A's observations and dog B's
    (transformed) observations, with dog B's frame index shifted by `lag`
    (dog B's frame `f + lag` is compared against dog A's frame `f`).

    Returns a list of (frame, obs_a, obs_b) for every accepted match
    across every shared frame.
    """
    matched = []
    for frame, obs_a in obs_a_by_frame.items():
        obs_b = obs_b_by_frame.get(frame + lag)
        if not obs_b:
            continue
        positions_a = np.array([o.position_m for o in obs_a])
        positions_b = apply_transform(np.array([o.position_m for o in obs_b]), transform)
        matches, _, _ = assign_detections(positions_a, positions_b, max_distance)
        for a_idx, b_idx in matches:
            matched.append((frame, obs_a[a_idx], obs_b[b_idx]))
    return matched


def best_frame_offset(obs_a_by_frame: dict, obs_b_by_frame: dict, transform: np.ndarray,
                       max_distance: float, max_lag: int = 5) -> int:
    """
    Search integer frame lags in [-max_lag, max_lag] for the one that
    maximises total matched pairs under the supplied transform, breaking
    ties toward zero lag (no reason to assume a lag exists if the data
    doesn't support one).

    This exists because the two dogs' track logs are indexed by their own
    export's frame counter, not a shared clock -- see the module
    docstring. A one- or two-frame lag from the two recordings not
    starting on exactly the same accumulate-window boundary is a
    real, expected possibility, not a bug to route around some other way.
    """
    best_lag, best_count = 0, -1
    for lag in range(-max_lag, max_lag + 1):
        count = len(_match_at_lag(obs_a_by_frame, obs_b_by_frame, transform, lag, max_distance))
        if count > best_count or (count == best_count and abs(lag) < abs(best_lag)):
            best_lag, best_count = lag, count
    return best_lag


# ---------------------------------------------------------------------------
# Spatial calibration refinement
# ---------------------------------------------------------------------------

def refit_transform(matched_position_pairs: list) -> np.ndarray:
    """
    Least-squares rigid transform (yaw-only rotation + translation) that
    best maps dog B's raw (untransformed) positions onto dog A's, given a
    list of (position_a, position_b_raw) correspondences.

    Restricted to yaw because both dogs are assumed deployed on roughly
    level ground -- the same assumption
    kei-stuff/lidar-perception/scripts/merge_with_prior.py's
    rotation_matrix_zyx docstring makes for point-cloud priors ("for
    typical level-ground deployments, only yaw matters"), carried over
    here. The horizontal (x, y) component is solved via 2D Kabsch/Umeyama
    on the correspondences' centroids; the vertical (z) component is a
    plain mean offset, solved independently since a level-ground
    assumption means there's no rotational coupling between z and the
    horizontal plane to account for.

    Needs at least 2 correspondences to be well-posed at all, and in
    practice needs several spread out in different directions to
    constrain yaw reliably -- 2 correspondences on top of each other, or
    colinear, can produce a yaw estimate driven entirely by noise. Callers
    (calibrate(), below) are responsible for deciding whether there's
    enough to trust; this function will compute *something* from as few
    as 2 pairs without complaint.
    """
    if len(matched_position_pairs) < 2:
        raise ValueError(
            f"need at least 2 correspondences to fit a transform, got {len(matched_position_pairs)}")

    a = np.array([pair[0] for pair in matched_position_pairs])
    b = np.array([pair[1] for pair in matched_position_pairs])

    a_xy, b_xy = a[:, :2], b[:, :2]
    a_centroid, b_centroid = a_xy.mean(axis=0), b_xy.mean(axis=0)
    a_centred, b_centred = a_xy - a_centroid, b_xy - b_centroid

    H = b_centred.T @ a_centred
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        # Reflection, not a rotation -- SVD's sign ambiguity. Flip the
        # smaller singular vector, the standard Kabsch fix.
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    yaw_deg = float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))
    t_xy = a_centroid - R @ b_centroid
    z_offset = float(np.mean(a[:, 2] - b[:, 2]))

    return build_transform(t_xy[0], t_xy[1], z_offset, yaw_deg)


@dataclass
class CalibrationResult:
    transform: np.ndarray
    lag: int
    correspondence: dict       # {dog_a_track_id: dog_b_track_id}
    n_matched_frames: int
    n_matched_pairs: int
    converged: bool


def _majority_correspondence(votes: dict, min_votes: int = 2) -> dict:
    """
    {(a_id, b_id): count} -> {a_id: b_id}, keeping only pairs that are
    each other's most-voted match (mutual best, not just "a matched b at
    least once") and that cleared min_votes -- a real cross-dog
    correspondence should agree frame after frame, not just once by
    chance on a single lucky frame.
    """
    best_b_for_a, best_a_for_b = {}, {}
    for (a_id, b_id), count in votes.items():
        if count > best_b_for_a.get(a_id, (None, -1))[1]:
            best_b_for_a[a_id] = (b_id, count)
        if count > best_a_for_b.get(b_id, (None, -1))[1]:
            best_a_for_b[b_id] = (a_id, count)

    correspondence = {}
    for a_id, (b_id, count) in best_b_for_a.items():
        if count >= min_votes and best_a_for_b.get(b_id, (None, -1))[0] == a_id:
            correspondence[a_id] = b_id
    return correspondence


def calibrate(obs_a_by_frame: dict, obs_b_by_frame: dict, initial_transform: np.ndarray,
              max_distance: float = 2.0, max_lag: int = 5, max_iterations: int = 5,
              min_votes: int = 2, convergence_tolerance_m: float = 1e-3) -> CalibrationResult:
    """
    Iteratively refine initial_transform against whichever tracks the two
    dogs actually agree on, the track-level analogue of ICP-with-prior.

    Each iteration: match dog A's and dog B's (transformed) observations
    frame by frame (globally-optimal, gated by max_distance -- see
    _match_at_lag), refit the transform from the resulting
    correspondences (refit_transform), and repeat until the transform
    stops moving by more than convergence_tolerance_m or max_iterations is
    reached. Set max_iterations=0 to skip refinement entirely and use
    initial_transform as-is (matching merge_with_prior.py's --no-icp) --
    the frame-lag search and correspondence vote still run in that case,
    just not the spatial refit.

    Frame lag is estimated once, up front, from initial_transform, and
    held fixed through every refinement iteration -- a genuinely wrong
    initial spatial prior could bias that estimate, but re-searching the
    lag after every spatial refit would let the two searches chase each
    other with no guarantee of converging to anything. If the initial
    prior is good enough to get a first correspondence at all, the lag it
    implies is almost certainly right; if it isn't, no amount of spatial
    refinement fixes a lag problem anyway.

    Returns the prior unchanged (converged=False, empty correspondence,
    zero matched pairs) if not even one frame ever produces a match --
    there's no information to refine from, and pretending otherwise would
    be worse than saying so plainly.
    """
    lag = best_frame_offset(obs_a_by_frame, obs_b_by_frame, initial_transform, max_distance, max_lag)

    transform = initial_transform.copy()
    votes, matched_pairs_raw = {}, []
    converged = False

    for iteration in range(max(max_iterations, 0) + 1):
        matched = _match_at_lag(obs_a_by_frame, obs_b_by_frame, transform, lag, max_distance)
        if not matched:
            return CalibrationResult(initial_transform, lag, {}, 0, 0, converged=False)

        votes = {}
        matched_pairs_raw = []
        for _frame, obs_a, obs_b in matched:
            key = (obs_a.track_id, obs_b.track_id)
            votes[key] = votes.get(key, 0) + 1
            matched_pairs_raw.append((obs_a.position_m, obs_b.position_m))

        if iteration >= max_iterations:
            break

        try:
            new_transform = refit_transform(matched_pairs_raw)
        except ValueError:
            break  # not enough correspondences to refit further -- keep current transform

        delta = float(np.linalg.norm(new_transform[:3, 3] - transform[:3, 3]))
        transform = new_transform
        if delta < convergence_tolerance_m:
            converged = True
            break

    correspondence = _majority_correspondence(votes, min_votes)
    n_matched_frames = len({frame for frame, _, _ in
                             _match_at_lag(obs_a_by_frame, obs_b_by_frame, transform, lag, max_distance)})
    return CalibrationResult(transform, lag, correspondence, n_matched_frames,
                              len(matched_pairs_raw), converged)


# ---------------------------------------------------------------------------
# Fusion
# ---------------------------------------------------------------------------

def fuse_tracks(obs_a_by_frame: dict, obs_b_by_frame: dict, transform: np.ndarray,
                lag: int, correspondence: dict) -> list:
    """
    Build one fused row per (frame, global object).

    A global object is either:
      - a corresponded pair (dog A's track_id maps to a dog B track_id via
        `correspondence`): fused position is the mean of both dogs'
        estimates (in dog A's frame, dog B's transformed by `transform`)
        on frames both currently see it, or whichever dog does on frames
        only one currently does -- correspondence doesn't require every
        single frame to have both dogs matched, only that they agreed
        often enough overall (see calibrate()'s min_votes).
      - a track with no correspondent at all: only one dog has ever been
        assigned to it. Reported as-is with source="dog1_only"/
        "dog2_only" rather than dropped -- this is the complementary-
        coverage case (one dog's view blocked, the other's isn't), not
        noise, and dropping it would throw away exactly the information
        multi-dog coverage is meant to add.

    Global object IDs are dog A's own track_id where a correspondence
    exists (arbitrary but stable choice -- dog A is nominally the
    reference frame everything else gets expressed in), or
    "b:<dog_b_track_id>" for a dog-B-only track with no dog-A counterpart,
    to keep the two id spaces from colliding.
    """
    b_to_a = {b_id: a_id for a_id, b_id in correspondence.items()}
    rows = []

    frames = sorted(set(obs_a_by_frame) | {f - lag for f in obs_b_by_frame})
    for frame in frames:
        by_global_id = {}

        for obs in obs_a_by_frame.get(frame, []):
            by_global_id[obs.track_id] = {"a": obs, "b": None}

        for obs in obs_b_by_frame.get(frame + lag, []):
            global_id = b_to_a.get(obs.track_id, f"b:{obs.track_id}")
            entry = by_global_id.setdefault(global_id, {"a": None, "b": None})
            entry["b"] = obs

        for global_id, entry in by_global_id.items():
            obs_a, obs_b = entry["a"], entry["b"]
            positions, n_points, speeds = [], 0, []
            if obs_a is not None:
                positions.append(obs_a.position_m)
                n_points += obs_a.n_points
                speeds.append(obs_a.speed_m_s)
            if obs_b is not None:
                positions.append(apply_transform(obs_b.position_m.reshape(1, 3), transform)[0])
                n_points += obs_b.n_points
                speeds.append(obs_b.speed_m_s)

            fused_position = np.mean(positions, axis=0)
            source = ("dog1+dog2" if obs_a is not None and obs_b is not None else
                      "dog1_only" if obs_a is not None else "dog2_only")

            rows.append({
                "frame": frame,
                "global_id": global_id,
                "dog1_track_id": obs_a.track_id if obs_a is not None else "",
                "dog2_track_id": obs_b.track_id if obs_b is not None else "",
                "source": source,
                "centroid_x_m": float(fused_position[0]),
                "centroid_y_m": float(fused_position[1]),
                "centroid_z_m": float(fused_position[2]),
                "n_points": n_points,
                "speed_m_s": float(np.mean(speeds)),
            })

    return rows


def write_fused_csv(rows: list, output_path: Path) -> None:
    fieldnames = ["frame", "global_id", "dog1_track_id", "dog2_track_id", "source",
                  "centroid_x_m", "centroid_y_m", "centroid_z_m", "n_points", "speed_m_s"]
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def plot_fusion(obs_a_by_frame: dict, obs_b_by_frame: dict, transform: np.ndarray,
                 lag: int, fused_rows: list, output_path: Path) -> None:
    """
    Top-down (x/y) plot: dog 1's raw tracks, dog 2's tracks after the
    calibration transform, and the fused result, coloured by source. The
    point of this isn't a polished deliverable (see
    evaluation/compare_pipelines.py's trajectories.png for that side of
    things once tracking output feeds a real rendering pipeline) -- it's
    the fastest way to sanity-check a calibration result by eye: a
    correspondence that "coloured itself in a straight line" is more
    convincing than the same information as CSV rows, and a wrong
    calibration usually looks visibly wrong here (parallel offset tracks
    that never actually meet) before you'd notice it in a track-count
    summary.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 8))

    for track_id, observations in group_by_frame_track(obs_a_by_frame).items():
        positions = np.array([o.position_m for o in observations])
        ax.plot(positions[:, 0], positions[:, 1], "o-", color="#1f77b4", alpha=0.4, markersize=3)

    for track_id, observations in group_by_frame_track(obs_b_by_frame).items():
        positions = apply_transform(np.array([o.position_m for o in observations]), transform)
        ax.plot(positions[:, 0], positions[:, 1], "s-", color="#d62728", alpha=0.4, markersize=3)

    fused_by_global_id = {}
    for row in fused_rows:
        fused_by_global_id.setdefault(row["global_id"], []).append(row)
    both_dogs_labelled = False
    for global_id, rows in fused_by_global_id.items():
        rows = sorted(rows, key=lambda r: r["frame"])
        xs = [r["centroid_x_m"] for r in rows]
        ys = [r["centroid_y_m"] for r in rows]
        both = [r["source"] == "dog1+dog2" for r in rows]
        ax.plot(xs, ys, "-", color="#2ca02c", linewidth=1)
        both_xs = [x for x, b in zip(xs, both) if b]
        both_ys = [y for y, b in zip(ys, both) if b]
        if both_xs:
            ax.scatter(both_xs, both_ys, color="#2ca02c", s=25, zorder=3,
                       label="both dogs" if not both_dogs_labelled else None)
            both_dogs_labelled = True

    ax.plot([], [], "o-", color="#1f77b4", alpha=0.4, label="dog 1 (raw)")
    ax.plot([], [], "s-", color="#d62728", alpha=0.4, label="dog 2 (calibrated)")
    ax.set_xlabel("x (m, dog 1's frame)")
    ax.set_ylabel("y (m, dog 1's frame)")
    ax.set_title(f"Track fusion (frame lag={lag:+d})")
    ax.legend(loc="best")
    ax.set_aspect("equal", adjustable="datalim")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def group_by_frame_track(obs_by_frame: dict) -> dict:
    """{frame: [Observation]} -> {track_id: [Observation, ...]} sorted by
    frame, for plotting one continuous line per track rather than per
    frame."""
    by_track = {}
    for observations in obs_by_frame.values():
        for obs in observations:
            by_track.setdefault(obs.track_id, []).append(obs)
    for track_id in by_track:
        by_track[track_id].sort(key=lambda o: o.frame)
    return by_track


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _run_pipeline_for_session(session_dir: Path):
    """Run evaluation/offline_pipeline.py's tracker against one dog's
    already-exported session. Imported lazily (only needed by the CLI, not
    by the pure fusion logic above or its unit tests) to keep this
    module's core importable without evaluation/'s own dependencies."""
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "evaluation"))
    from offline_pipeline import run_pipeline, write_tracks_csv  # noqa: E402
    track_rows, _frame_summaries = run_pipeline(session_dir)
    return track_rows, write_tracks_csv


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Fuse two dogs' independently-tracked objects into one shared-frame "
            "track log, using a calibration prior refined against whichever "
            "tracks the two dogs actually agree on -- see this module's "
            "docstring for why this operates on tracks rather than raw point "
            "clouds."
        )
    )
    parser.add_argument("dog1_session", type=Path,
                         help="Exported session dir for dog 1 (pcd/frame_*.pcd + poses.csv) "
                              "-- this dog's frame is the reference everything else gets expressed in.")
    parser.add_argument("dog2_session", type=Path,
                         help="Exported session dir for dog 2, covering the same time window.")
    parser.add_argument("--offset", type=str, default="0,0,0,0",
                         help="Calibration prior as 'x_m,y_m,z_m,yaw_deg' -- dog 2's frame "
                              "expressed in dog 1's frame (default: identity, i.e. 'assume "
                              "co-located and aligned unless told otherwise').")
    parser.add_argument("--max-distance", type=float, default=2.0,
                         help="Max metres between two dogs' track centroids to consider them "
                              "the same object (default: 2.0, matching perception/tracking.py's "
                              "own reid_max_distance -- both are 'same object, brief gap or "
                              "cross-dog view' gates).")
    parser.add_argument("--max-lag", type=int, default=5,
                         help="Search this many frames either side of zero lag for the best "
                              "frame-index alignment between the two dogs' track logs (default: 5).")
    parser.add_argument("--max-iterations", type=int, default=5,
                         help="Max calibration refinement iterations (default: 5). Set to 0 to "
                              "use --offset as-is, matching merge_with_prior.py's --no-icp.")
    parser.add_argument("--min-votes", type=int, default=2,
                         help="Minimum frames two tracks must mutually match on to be treated "
                              "as a confirmed cross-dog correspondence (default: 2) -- a single "
                              "lucky match on one frame isn't enough evidence.")
    parser.add_argument("--output", type=Path, default=Path("output/fused_tracks.csv"),
                         help="Output path for the fused track CSV.")
    parser.add_argument("--save-transform", type=Path, default=None,
                         help="Optional path to save the final 4x4 calibration transform as a "
                              "text file (reloadable as a future --offset via --initial-matrix-"
                              "style tooling, or just for the record).")
    parser.add_argument("--plot", type=Path, default=None,
                         help="Optional path to save a top-down PNG plot of both dogs' raw/"
                              "calibrated tracks plus the fused result -- the fastest way to "
                              "sanity-check a calibration by eye. Requires matplotlib.")
    args = parser.parse_args()

    print("=" * 60)
    print("STEP 1: Running the tracking pipeline against each dog's session")
    print("=" * 60)
    track_rows_a, _ = _run_pipeline_for_session(args.dog1_session)
    track_rows_b, _ = _run_pipeline_for_session(args.dog2_session)
    print(f"  dog 1 ({args.dog1_session.name}): {len(track_rows_a)} confirmed-track rows, "
          f"{len({r['track_id'] for r in track_rows_a})} distinct tracks")
    print(f"  dog 2 ({args.dog2_session.name}): {len(track_rows_b)} confirmed-track rows, "
          f"{len({r['track_id'] for r in track_rows_b})} distinct tracks")

    obs_a_by_frame = group_by_frame(observations_from_track_rows(track_rows_a))
    obs_b_by_frame = group_by_frame(observations_from_track_rows(track_rows_b))

    print("\n" + "=" * 60)
    print("STEP 2: Calibrating dog 2's frame onto dog 1's")
    print("=" * 60)
    initial_transform = parse_offset_string(args.offset)
    result = calibrate(
        obs_a_by_frame, obs_b_by_frame, initial_transform,
        max_distance=args.max_distance, max_lag=args.max_lag,
        max_iterations=args.max_iterations, min_votes=args.min_votes,
    )
    print(f"  frame lag: {result.lag:+d} (dog 2's frame f+{result.lag} matched against dog 1's frame f)")
    print(f"  matched pairs used for calibration: {result.n_matched_pairs} across "
          f"{result.n_matched_frames} frames")
    print(f"  cross-dog track correspondence: {result.correspondence or '(none found)'}")
    print(f"  refinement converged: {result.converged}")
    print(f"  final transform:\n{np.array2string(result.transform, precision=4, suppress_small=True)}")
    if args.save_transform:
        args.save_transform.parent.mkdir(parents=True, exist_ok=True)
        np.savetxt(str(args.save_transform), result.transform, fmt="%.6f")
        print(f"  saved to {args.save_transform}")

    if result.n_matched_pairs == 0:
        print("\n  No frame ever matched a dog-1 track to a dog-2 track under this prior and "
              "max-distance -- either the two dogs never observed the same object during this "
              "session, or --offset is wrong by more than --max-distance. Fusing anyway (every "
              "row will be dog1_only/dog2_only, no global correspondence), but treat this as a "
              "negative calibration result, not a working one.")

    print("\n" + "=" * 60)
    print("STEP 3: Fusing tracks")
    print("=" * 60)
    fused_rows = fuse_tracks(obs_a_by_frame, obs_b_by_frame, result.transform, result.lag,
                              result.correspondence)
    n_both = sum(1 for r in fused_rows if r["source"] == "dog1+dog2")
    n_dog1_only = sum(1 for r in fused_rows if r["source"] == "dog1_only")
    n_dog2_only = sum(1 for r in fused_rows if r["source"] == "dog2_only")
    print(f"  {len(fused_rows)} fused rows: {n_both} seen by both dogs, "
          f"{n_dog1_only} dog-1-only, {n_dog2_only} dog-2-only")

    write_fused_csv(fused_rows, args.output)
    print(f"\nSaved fused track log to {args.output}")

    if args.plot:
        plot_fusion(obs_a_by_frame, obs_b_by_frame, result.transform, result.lag, fused_rows, args.plot)
        print(f"Saved trajectory plot to {args.plot}")


if __name__ == "__main__":
    main()
