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
compare_occlusion_detector.py  A/B test experimental range-image accumulation
compare_visibility_tolerance.py  A/B test fixed vs range-adaptive visibility
response_evaluator.py   applies the response policy to tracks + odometry
tests/                  unit tests for the offline pipeline and response evaluator
```

### Experimental occlusion accumulation

`compare_occlusion_detector.py` changes only the moving-point detector. The
baseline uses the established nearest-neighbour difference plus one-frame
visibility gate; the experiment uses the temporal range-image accumulator in
`perception/occlusion_accumulation.py`. Both then use identical DBSCAN and
tracking parameters:

```bash
python3 evaluation/compare_occlusion_detector.py \
    data/2026-05-12_soton_indoor_dog1
```

It writes `baseline_tracks.csv`, `occlusion_tracks.csv`, and `summary.json`
under `output/<session>-occlusion-comparison/`. The summary includes moved
point counts, confirmed IDs, first-confirmed frame, runtime, and accumulator
pixel diagnostics. These are screening measurements, not accuracy metrics:
the recordings have no pointwise moving/static ground truth.

The implementation adapts Kim et al.'s 2025 occlusion-accumulation paper; see
`../CITATIONS.md` for the citation and precise implementation differences.
The first implementation retained weak sub-threshold differences across
frames. Reviewing the paper's equations showed that its truncation instead
sets evidence below `alpha * range` to zero on every sequence; otherwise small
pose and sampling errors can eventually accumulate into motion. The corrected
implementation follows that ordering and uses the paper's starting ratios
(`alpha=0.3`, `beta=0.1`) with an explicit 0.3 m activation floor.

Corrected screening on the original ten-scan exports (runtime is wall-clock
and will vary with machine load):

| session | established tracks | experimental tracks | established / experimental moved points | established / experimental mean runtime |
|---|---:|---:|---:|---:|
| `soton_indoor_dog1` | 2 | 2 | 797 / 995 | ~9 / ~70 ms |
| `walk_test` | 13 | 41 | 5,593 / 13,146 | ~10 / ~96 ms |
| `fallback_dog2` | 7 | 10 | 3,894 / 7,052 | ~8 / ~95 ms |

This remains a **negative result for the aggregate input**. Disabling gap
completion reduces runtime to roughly 6–11 ms/frame, but still produces
34/10/2 tracks on the three sessions respectively, so completion is neither
the sole cause nor worth its Python prototype cost.

The likely structural mismatch is that each exported PCD frame combines ten
registered scans acquired from different sensor poses. The paper updates one
organized scan from one pose at a time; treating a ten-origin aggregate as
one spherical image creates range discontinuities that its assumptions do not
cover. Ground segmentation and the paper's full-object expansion are also
absent.

A new export of `soton_indoor_dog1` with `--accumulate 1` provides 242
individual scans and 242 matching poses. With `cluster_min_points=5`, no gap
completion, and a 0.05 m baseline change threshold:

- the baseline confirms one track with four measured rows;
- the accumulator confirms one track with 28 measured rows;
- the accumulator trajectory follows the same `y ~= -1.59 m` path as one
  known aggregate-frame track; and
- mean runtime is about 2.2 ms/frame for the accumulator versus 2.1 ms/frame
  for the baseline.

This is promising evidence for the lightweight core, not an accuracy claim:
the recording lacks point labels and this configuration does not recover the
other known aggregate trajectory. Enabling one-bin completion adds a second,
intermittent track but raises runtime to roughly 48 ms/frame; that track is not
trusted without manual labels.

The live node therefore retains the established detector by default. A guarded
`use_occlusion_accumulation:=true` mode exists solely for physical A/B testing,
requires `accumulate_scans:=1` and `accumulate_stride:=1`, and uses no gap
completion. A suitable starting command-line parameter set is:

```text
-p use_occlusion_accumulation:=true
-p accumulate_scans:=1
-p accumulate_stride:=1
-p cluster_min_points:=5
```

Do not connect this mode to actuation during initial tests. Record both the
normal and experimental track topics against labelled walking, crossing,
standing, and no-person scenes first.

### Experimental range-adaptive visibility tolerance

`compare_visibility_tolerance.py` screens an inexpensive extension of the
established visibility gate:

```bash
python3 evaluation/compare_visibility_tolerance.py \
    data/2026-04-24_walk_test --tolerance-range-ratio 0.02
```

The effective tolerance is
`max(range_image_tolerance, ratio * candidate_range)`. Zero is the live
default and exactly preserves existing behaviour. Ratios of 0.01, 0.02, and
0.03 changed no moved points or tracks on any of the three exports because
the existing 0.3 m floor dominated at their detection ranges. A ratio of 0.05
removed only 9–26 points per session and changed no confirmed track count;
0.10 removed more points but still changed no track count. There is no
recorded-data justification for enabling it by default, though the option is
available for a genuinely longer-range dataset.

### Evaluating the stop response

The response evaluator uses current, measured tracks only (coasted
predictions are excluded, matching the live ROS topic) and each frame's
FastLIO pose. It writes `response.csv`, `response_summary.json`, and
`response.png`:

```bash
python3 evaluation/response_evaluator.py \
    data/2026-05-12_soton_indoor_dog1 \
    --tracks-csv output/2026-05-12_soton_indoor_dog1/2026-05-12_soton_indoor_dog1_tracks.csv
```

Omit `--tracks-csv` to rerun the current perception pipeline first. Response
parameters have matching flags (`--stop-distance`, `--clear-distance`,
`--trigger-duration`, and `--clear-duration`) for an explicit A/B test.

Default-policy checks against the exported data currently present produced:

| session | duration | stop transitions | requested-stop time | minimum planar range |
|---|---:|---:|---:|---:|
| `2026-04-24_walk_test` | 74.4 s | 2 | 17.0 s | 0.60 m |
| `2026-05-12_fallback_dog2` | 92.0 s | 4 | 19.0 s | 1.12 m |
| `2026-05-12_soton_indoor_dog1` | 23.2 s | 1 | 5.0 s | 1.66 m |

These establish that the policy fires and clears against recorded tracks;
they are not false-positive or stopping-performance measurements because
the recordings have no response ground truth and the robot was not being
commanded. `fallback_dog1` cannot be evaluated: its known-empty `poses.csv`
provides no sensor position, and the evaluator fails clearly rather than
silently measuring distance from the map origin.

## Why this runs offline

This replays `perception/tracking.py`'s actual clustering/tracking code (no
`rclpy` dependency) directly against the PCD frames and poses CSV produced by
Kei's `export_fastlio.py`. Both alternatives therefore see identical points
and odometry, so output differences are attributable to the algorithms
rather than small differences between two independent FastLIO runs.

ROS 2 DDS bag replay has also been re-checked successfully on this machine:
the perception container reproduced the two `soton_indoor` tracks. Offline
evaluation remains a methodological choice for controlled A/B testing, not a
networking workaround. Live replay is still necessary for integration and
timing checks on the deployment hardware.

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
IDs, matching the false-positive-heavy moving-sensor pattern documented in
`perception/README.md`). Check which case a given `--kei-tracks` file
is before trusting a "current pipeline wins by 10x" number -- it might
just mean the reference was run with the wrong parameters for this data,
not that the tracker actually improved that much.

## Testing

From the repository root, all suites can now be collected together:

```bash
python3 -m pytest perception/tests evaluation/tests export/tests merge/tests
```

`test_pcd_io.py` covers the binary PCD reader directly.
`test_offline_pipeline.py` uses small synthetic sessions to check both the
established detector and the optional accumulator through PCD loading,
voxel downsampling, clustering, and tracking. It also verifies that the
accumulator fails clearly without the odometry required for reprojection.
The accumulator's completion, persistence, pose compensation, clearing, and
reset behaviour have focused tests in
`perception/tests/test_occlusion_accumulation.py`. The suite deliberately
doesn't duplicate clustering, Kalman tracking, and one-frame visibility-gate
tests already present in `perception/tests/`.
`test_response_evaluator.py` covers empty frames, coast exclusion, required
odometry, and the machine-readable response CSV.
