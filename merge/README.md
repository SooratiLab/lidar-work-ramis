# merge

Fuses two dogs' independently-tracked objects into a single shared-frame
track log, instead of merging raw point clouds. This is Phase 3
(multi-dog map/track merging) -- previously just planning notes, this is
the first working code for it.

## Why track-level, not point-cloud-level

Kei's handover (`kei-stuff/lidar-perception/Multi-LiDAR Sensing.pdf`)
already tried point-cloud merging (`icp_merge.py`'s FPFH+RANSAC,
`merge_with_prior.py`'s ICP-with-prior) on real two-dog field data and it
didn't work reliably, for three compounding reasons: each dog's FastLIO
runs in its own unregistered world frame with no shared reference or
calibration step; the two dogs often observe different halves of a scene,
leaving too little overlap for feature matching; and FastLIO's sparse
output (thousands of points per frame, not the hundreds of thousands a
stationary bench scan produces) further starves feature-based
registration of what it needs. None of that changes by switching to
tracks instead of points -- but what's being aligned does: a "track" is
already the thing both dogs are independently trying to agree on (a
person, a moving object), so there's far less data to align but it's far
more semantically meaningful than a raw point.

## What's in here

```
track_fusion.py    core library + CLI: calibrate two dogs' frames against
                   each other, then fuse their tracks
tests/             unit tests against synthetic observations
```

## How it works

1. **Run the tracking pipeline independently per dog.** Each dog's
   exported session (`pcd/frame_*.pcd` + `poses.csv`, same layout
   `evaluation/` and `export/` already use) goes through
   `evaluation/offline_pipeline.py`'s `run_pipeline()` unchanged -- this
   module doesn't re-implement or duplicate any tracking logic, only
   consumes its output.
2. **Search for a frame-index lag.** Both dogs' track logs are indexed by
   their own export's frame counter, not a shared clock (see
   `offline_pipeline.py` -- `time_s` is actually `float(frame_idx)`, not
   wall-clock time). `best_frame_offset()` searches a small window of
   integer lags (default +/-5 frames) for the one that produces the most
   cross-dog matches under the current calibration guess, rather than
   assuming the two recordings started on exactly the same accumulate-
   window boundary.
3. **Calibrate.** Starting from a supplied prior (`--offset x_m,y_m,z_m,
   yaw_deg`, a one-off deployment measurement -- or identity if genuinely
   unknown, though see the caveat below), `calibrate()` iteratively
   refines it: match dog A's and dog B's (transformed) tracks frame by
   frame (globally-optimal Hungarian assignment, reusing
   `perception/tracking.py`'s own `assign_detections` rather than
   reimplementing it), refit the transform from whichever tracks
   correspond (`refit_transform` -- 2D Kabsch/Umeyama for yaw + xy
   translation, a plain mean for the z offset, both restricted to a
   level-ground assumption matching `merge_with_prior.py`'s own), and
   repeat until the transform stops moving or `--max-iterations` is hit.
   This is the track-level analogue of ICP-with-prior: nearest-track
   correspondences instead of nearest-point ones.
4. **Fuse.** Once tracks are corresponded (a mutual-best-match vote across
   every frame they were matched on, not just a single lucky frame --
   see `_majority_correspondence`), `fuse_tracks()` builds one row per
   frame per global object: the mean of both dogs' position estimates
   where both currently see it, or a single dog's own estimate where only
   one does. Uncorresponded tracks aren't dropped -- a track only one dog
   ever saw is the complementary-coverage case (one dog's view blocked,
   the other's isn't) the cardboard-box session below demonstrates, not
   noise.

## Usage

```bash
python3 merge/track_fusion.py data/2026-05-12_fallback_dog1 data/2026-05-12_fallback_dog2 \
    --output output/fallback_fused_tracks.csv \
    --plot output/fallback_fused_trajectories.png
```

| Argument          | Type  | Default                     | Description                                                                 |
|-------------------|-------|------------------------------|-------------------------------------------------------------------------------|
| `dog1_session`    | path  | --                            | Exported session dir for dog 1 -- the frame everything else is expressed in |
| `dog2_session`    | path  | --                            | Exported session dir for dog 2, covering the same time window               |
| `--offset`        | str   | `0,0,0,0`                     | Calibration prior, `x_m,y_m,z_m,yaw_deg` (dog 2's frame expressed in dog 1's)|
| `--max-distance`  | float | `2.0`                         | Max metres between two dogs' centroids to consider them the same object     |
| `--max-lag`       | int   | `5`                           | Frame-lag search window (see step 2 above)                                 |
| `--max-iterations`| int   | `5`                           | Calibration refinement iterations; `0` uses `--offset` as-is (no refine)    |
| `--min-votes`     | int   | `2`                           | Frames two tracks must mutually match on to count as corresponded          |
| `--output`        | path  | `output/fused_tracks.csv`     | Fused track CSV                                                            |
| `--save-transform`| path  | None                          | Optional: save the final 4x4 transform to a text file                      |
| `--plot`          | path  | None                          | Optional: top-down PNG of raw/calibrated/fused trajectories                |

### Output

`--output`: one row per frame per global object -- `frame`, `global_id`,
`dog1_track_id`, `dog2_track_id` (blank if that dog didn't see it this
frame), `source` (`dog1+dog2` / `dog1_only` / `dog2_only`), fused
`centroid_x/y/z_m`, summed `n_points`, averaged `speed_m_s`.

`--plot` (if given): dog 1's raw tracks (blue), dog 2's tracks after the
final calibration transform (red), and the fused result (green, with
solid dots marking frames both dogs contributed to). The fastest way to
sanity-check a calibration by eye -- a correspondence that traces one
continuous path is far more convincing than the same information as CSV
rows, and a wrong calibration usually looks visibly wrong here (parallel
offset tracks that never actually meet).

## Validated against real data: the cardboard-box session

Kei's handover describes `2026-05-12_fallback_cardbox1` informally as a
small demo of complementary coverage: one dog's view of a person blocked
by cardboard boxes, the other's not. It's also the only two-dog session
currently exported to the `pcd/` + `poses.csv` layout this module needs
(`data/2026-05-12_fallback_dog1`, `data/2026-05-12_fallback_dog2`) -- see
`export/README.md` if a different two-dog bag needs exporting first.

Running with no manual prior at all (`--offset 0,0,0,0`, i.e. "assume
co-located and aligned unless told otherwise") and `--max-iterations 30`
(the default 5 gets close but hasn't fully converged by the tolerance
check yet -- see "Honest caveats" below):

```
dog 1: 110 confirmed-track rows, 14 distinct tracks
dog 2: 63 confirmed-track rows, 7 distinct tracks

frame lag: -1
matched pairs used for calibration: 41 across 41 frames
cross-dog track correspondence: {2: 0, 5: 1, 13: 3, 22: 6, 24: 10}
refinement converged: True
final transform: ~2.26 m x, -0.11 m y, 0.13 m z, -5.5 deg yaw

140 fused rows: 33 seen by both dogs, 77 dog-1-only, 30 dog-2-only
```

The plot (`output/fallback_fused_trajectories.png`) shows why this is a
believable result, not just a plausible-looking number: dog 1's track 2
and dog 2's track 0 trace one continuous loop in the fused plot (frames
2-11 dog-1-only as the object enters dog 1's view, frames 6-11
corresponded once dog 2 picks it up too, tracing a smooth, physically
sensible path throughout) -- not two disconnected fragments that
happened to satisfy a distance threshold once. Several other tracks stay
entirely uncorresponded (a dog-1-only track near (-5, -5.5) and a
dog-2-only one out past (6, -3)) -- consistent with the scene's actual
complementary-coverage premise, where much of the session should *not*
have both dogs seeing the same thing.

### Honest caveats

- **Only one two-dog session currently has exported data.** This is a
  real validated result, not a synthetic one, but it's one session, not a
  systematic test across scenarios -- treat the specific numbers above as
  a demonstration that the approach works on real field data, not as
  tuned defaults proven to generalise.
- **`fallback_dog1`'s `poses.csv` has zero rows** (a known, separately
  tracked export bug -- see `TODO.md`'s housekeeping section), so dog 1's
  visibility gate and odometry-plausibility filter both ran fail-open for
  this session (`offline_pipeline.py`'s `frame_position` is `None` for
  every frame). Dog 1's 14 tracks for what's a fairly simple scene likely
  includes more false positives than a session with working odometry
  would -- re-export `fallback_dog1` if a cleaner comparison is ever
  needed, this module doesn't work around that gap.
- **No independent ground truth for the calibration.** The final
  transform above (~2.26 m apart, ~5.5 degree yaw) is *plausible* for two
  dogs a few metres apart facing similar directions, and the visual
  trajectory match is real supporting evidence, but nothing here
  independently confirms the two dogs' actual relative deployment pose --
  there's no tape measure or surveyed reference for this session.
- **The default `--max-iterations 5` doesn't fully converge on this
  session** (`converged: False`, though the transform is already close --
  compare the two runs above) -- the convergence tolerance
  (`convergence_tolerance_m`, 1 mm) is tight relative to how much a
  yaw-only 2D Kabsch fit moves per iteration on 5 correspondences. Worth
  raising `--max-iterations` rather than trusting the default blindly
  until this is checked against a session with more correspondences.
- **Correspondence found only 5 of dog 1's 14 tracks and 5 of dog 2's 7**
  -- most tracks in this session are genuinely single-dog-only, matching
  the scene's own premise, but this also means the calibration itself is
  resting on a fairly small number of correspondences (41 matched frames
  across 5 track pairs). A session with more sustained double-coverage
  would be a better test of calibration stability specifically.
- **No NTP/clock-sync investigation done here.** `TODO.md` flags this as
  an open Phase 3 question -- frame-index lag search is a coarse stand-in
  for real clock synchronisation, tolerant of the "~100ms drift at
  1-second frames" Kei's handover calls tolerable, but not a substitute
  for actually checking it if frame duration ever shrinks for a realtime
  version of this.

## Testing

```bash
cd merge
python3 -m pytest tests/
```

Everything is tested against synthetic `Observation`s built directly in
Python (known transforms, known lags, deliberately-unrelated positions
for the "dogs never agree" case) rather than real exported sessions --
the pipeline wiring that produces real track logs already has its own
coverage in `evaluation/tests/`, and this module's own CLI entry point
(`_run_pipeline_for_session`) is a thin wrapper around that, not logic
worth re-testing here. What's unique to this module -- frame-lag search,
transform refit, mutual-vote correspondence, fused-row construction -- is
exercised directly, including the two "calibration finds nothing" and
"skip refinement" cases that matter for using this honestly, not just the
success path.

## What this doesn't do (yet)

- **No more than two dogs.** Extending the pairwise correspondence/fusion
  above to three or more dogs isn't just "run it three times" --
  transitive correspondence (dog A matches dog C via dog B, but does A
  match C directly?) and picking which dog's frame is the reference for
  more than two need actual design, not just a loop. Not attempted here;
  only two Mid-360 units exist in the lab currently anyway (see
  `DOCS.md`'s "Wider project context"), so this hasn't been a practical
  gap yet.
- **No automatic prior-free calibration.** `--offset 0,0,0,0` happened to
  work on the cardboard-box session because the true offset was within
  `--max-distance` of identity by chance (the loop trajectory above still
  matched at the ungated identity guess). A genuinely unknown, larger
  relative pose (dogs facing different directions, further apart) would
  need either a real prior or a proper global search this module doesn't
  attempt -- `calibrate()` refines a prior, it doesn't search for one from
  nothing (see its docstring). `TODO.md`'s Phase 3 notes list a one-time
  deployment measurement or a shared landmark as the intended source for
  that prior; this module is what consumes it once available, not a
  replacement for measuring it.
- **No live/realtime version.** This runs against already-exported
  sessions, the same way `evaluation/` does, for the same reason: it's
  the fastest way to validate the actual fusion logic against real data
  before building anything that needs two live dogs and a real-time
  correspondence step running simultaneously.
