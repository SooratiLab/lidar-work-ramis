# evaluation

Visual and numeric comparison between Kei's offline `track_motion.py`
pipeline (`kei-stuff/lidar-perception/scripts/`) and the current
`perception/tracking.py` pipeline, run against the same exported LiDAR
sessions. Answers a specific question the rest of this repo's testing
doesn't: not just "does the new pipeline still find the known-good real
tracks" (already covered in `perception/README.md`), but "how does its
*output* actually compare, visually and in numbers, to what the original
pipeline reported on the same data."

## What's in here

```
pcd_io.py               binary PCD reader (x/y/z, matching export_fastlio.py's output)
offline_pipeline.py     replays an exported session through perception/tracking.py
compare_pipelines.py    CLI: run + compare + visualise, one command per session
tests/                  unit tests for pcd_io.py and offline_pipeline.py
```

## Why this runs offline, not over a live bag replay

The obvious way to compare would be replaying a bag through this repo's
`docker/` FastLIO image with `perception/online_perception_node.py`
subscribed live, the same way `docker compose --profile replay` normally
works. That doesn't work on this machine: ROS 2 DDS discovery doesn't
function here at all, confirmed with a plain `rclpy` publisher/subscriber
pair failing to discover each other even within a single process, not just
across the driver/FastLIO/perception container boundary docker-compose
spans. Worth knowing if this ever needs debugging again, but not worth
chasing further against a dev machine that was never the deployment target
-- the Jetson is, and DDS there is a separate, real, already-tracked
question of its own.

**Update**: re-checked this while building `export/`, since that needed
the same cross-container DDS path to test against real data. DDS
discovery works on this machine now -- a bare `ros2 topic pub`/`hz` pair
discovers fine, and a full `docker compose --profile export up` run
(`fastlio`, `bag`, and a third container all talking to each other) also
worked end to end with no discovery issues. Neither test controlled for
what changed since the finding above, so this isn't a root-cause
explanation -- but the live `replay` profile and a fresh live-bag-replay
run of this comparison are both worth actually retrying rather than
continuing to assume they're blocked here. Not retried yet as part of
this change; the rest of this section's reasoning for why the offline
replay is arguably the *better* comparison regardless (identical input to
both pipelines, no double-FastLIO-registration variance) still holds
either way.

Instead, this replays `perception/tracking.py`'s actual clustering/tracking
code (no `rclpy` dependency -- see that module's own docstring) directly
against sessions Kei's `export_fastlio.py` already exported to PCD frames +
a poses CSV. That export is exactly what `online_perception_node.py` would
see frame-by-frame subscribed live to the same bag replayed through
FastLIO: same points, same odometry, same accumulate window. This is
arguably a *cleaner* comparison than a fresh live replay besides: both
pipelines then see identical points and odometry, so any difference in
output is attributable to the tracking algorithm, not to two independent
FastLIO runs producing slightly different registration.

## Usage

Needs a session directory (under `data/`) containing `pcd/frame_*.pcd` and
`poses.csv`, matching `export_fastlio.py`'s own output layout exactly --
copy one over from `kei-stuff/lidar-perception/data/<session>/` if it
isn't already in `data/`.

```bash
# current pipeline only, no reference to compare against
python3 evaluation/compare_pipelines.py data/2026-05-13_dds_test_dog1

# full comparison against Kei's own track_motion.py output for the same session
python3 evaluation/compare_pipelines.py data/2026-05-12_soton_indoor_dog1 \
    --kei-tracks ../kei-stuff/lidar-perception/output/2026-05-12_indoor/soton_indoor_dog1_tracks.csv

# re-tune a parameter and skip the gif (the slowest step) while iterating
python3 evaluation/compare_pipelines.py data/2026-05-12_fallback_dog1 \
    --kei-tracks ../kei-stuff/lidar-perception/output/2026-05-12_indoor/fallback_dog1_tracks.csv \
    --min-hits 3 --no-gif
```

| Argument             | Type  | Default                | Description                                                        |
|----------------------|-------|-------------------------|----------------------------------------------------------------------|
| `session`            | path  | --                      | Directory with `pcd/frame_*.pcd` + `poses.csv` (required)           |
| `--kei-tracks`       | path  | None                    | Kei's `*_tracks.csv` for the same session -- enables the comparison plots |
| `--output-dir`       | path  | `output/<session name>/` | Where artifacts are written                                        |
| `--no-gif`           | flag  | off                     | Skip `frame_comparison.gif` (the slowest step)                     |
| `--gif-fps`          | float | 2.5                     | Playback rate for the gif                                          |
| `--voxel`            | float | 0.05                    | Voxel downsample size (m) -- matches `online_perception_node.py`   |
| `--threshold`        | float | 0.15                    | Moved-point distance threshold (m)                                 |
| `--eps`              | float | 0.5                     | DBSCAN cluster radius (m)                                          |
| `--min-points`       | int   | 10                      | DBSCAN minimum cluster size                                        |
| `--z-max`            | float | 2.5                     | Ceiling crop height (m)                                            |
| `--min-hits`         | int   | 2                       | Real detections needed before a track is confirmed                 |
| `--no-visibility-gate` | flag | off                    | Disable the odometry-referenced visibility gate                    |

Every default matches `online_perception_node.py`'s own declared parameter
defaults exactly (`offline_pipeline.py`'s `PipelineParams` dataclass exists
specifically so this can't quietly drift from what the live node would
actually run) -- the flags above are for deliberately re-tuning a
comparison, not a separately-maintained copy of the same numbers.

### Output

Always written to `--output-dir` (default `output/<session name>/`):

- `<session>_tracks.csv` -- the current pipeline's confirmed tracks, one
  row per track per frame. Same shape as Kei's own `*_tracks.csv`, in
  metres instead of millimetres.
- `frame_comparison.gif` -- top-down point cloud per frame with the
  current pipeline's active track centroids overlaid, coloured by track
  ID, coasting tracks marked with an `x` instead of a dot.

Only written if `--kei-tracks` is given:

- `trajectories.png` -- top-down path of every track, both pipelines,
  against the sensor's own path. Kei's raw single-frame noise clusters are
  drawn too (small orange `x` marks) -- seeing where they land relative to
  the two pipelines' real tracks is more informative than a bare count.
- `speed_profiles.png` -- speed (m/s) over time per persistent/confirmed
  track, both pipelines, with a shaded walking-pace band for reference.
- `summary.png` -- track-ID-count (split into persistent/confirmed vs
  single-frame noise) and a per-track persistence boxplot.

## A fair comparison needs Kei's pipeline run with matching parameters

Kei's `track_motion.py` has no confirmation step -- see
`perception/tracking.py`'s `CentroidTracker` docstring for why the current
pipeline added `min_hits`. Every DBSCAN cluster that ever matched across a
frame pair gets its own permanent track ID in the raw CSV, including
clusters seen in exactly one frame and never again. `compare_pipelines.py`
splits Kei's tracks into "persistent" (>=2 observations -- what a
`min_hits=2` gate would have confirmed) and "singleton" before comparing,
so the numbers are a like-for-like comparison against the current
pipeline's already-confirmed-only output rather than a filtered count
against an unfiltered one.

That still assumes Kei's reference CSV was generated with the
FastLIO-tuned parameters (`--eps 500 --min-points 10 --threshold 150
--z-max 2500`, matching this repo's own defaults) rather than
`track_motion.py`'s un-tuned defaults (`--eps 200 --min-points 50`), which
`lidar-perception/README.md` itself notes "produces zero clusters" at
FastLIO's point density. `2026-04-24_walk_test`'s originally-committed
`2026-04-24_walk_trajectory.csv` turned out to be exactly that case (8 rows
total, all single-frame, evidently an early/untuned run) -- re-running
`track_motion.py` with the documented tuned parameters against the same
exported frames (`kei_reference_tracks_retuned.csv` in that session's
`data/` directory) produced the expected result instead (589 raw track
IDs, matching the same false-positive-heavy pattern `DOCS.md` already
documents for this session). Check which case a given `--kei-tracks` file
is before trusting a "current pipeline wins by 10x" number -- it might
just mean the reference was run with the wrong parameters for this data,
not that the tracker actually improved that much.

## Testing

```bash
cd evaluation
python3 -m pytest tests/
```

`test_pcd_io.py` covers the binary PCD reader directly. `test_offline_pipeline.py`
is an integration test against a small synthetic session (a static
background plus a cluster that appears at two positions one second apart)
checking that the replay wiring -- PCD loading, voxel downsampling, frame
diffing, clustering, and the tracker -- reports nothing after the first
detection and a confirmed, walking-pace track after the second. It
deliberately doesn't re-test the individual algorithm components
(clustering, the Kalman tracker, the visibility gate) that already have
their own direct unit tests in `perception/tests/`.
