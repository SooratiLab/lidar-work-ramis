# perception

Live/online counterpart to `kei-stuff/lidar-perception/scripts/track_motion.py`.
That script batch-processes a folder of pre-exported PCD files; this
subscribes to FastLIO's `/cloud_registered` and `/Odometry` directly and
runs the same accumulate -> diff -> cluster pipeline as each message
arrives, with an improved tracker on top -- see "Tracking" below for what
changed and why.

## Layout

- `online_perception_node.py` -- the ROS 2 node: subscribes to FastLIO's
  topics, accumulates scans into frames, publishes confirmed tracks and
  RViz markers. The only
  non-ROS logic left in it is a trivial colour-picker for track markers,
  not worth splitting out for its own tests.
- `pointcloud.py` -- PointCloud2 parsing and voxel downsampling. Plain
  numpy, no rclpy dependency, so it's unit-testable on its own.
- `tracking.py` -- DBSCAN clustering of the moved points, and the
  frame-to-frame centroid tracker (Kalman filter + Hungarian assignment +
  coasting + re-identification after longer gaps). Plain numpy/scipy/
  scikit-learn, no rclpy dependency.
- `range_image.py` -- the odometry-referenced visibility gate that
  suppresses "moved" points a moving sensor's own viewpoint change
  produced, rather than something actually moving -- see "Visibility gate"
  below. Plain numpy, no rclpy dependency.
- `occlusion_accumulation.py` -- experimental pose-compensated temporal
  range-image detector adapted from Kim et al. (2025). It is used only by
  the offline A/B evaluator, not the live node; see
  `../evaluation/README.md` for the mixed initial result and
  `../CITATIONS.md` for technical attribution.
- `free_space.py` -- experimental sparse free-space history adapted from
  Dynablox's central motion cue. Repeated pose-aligned rays establish
  trustworthy free voxels; the established detector can then reject candidates
  that did not enter that space. This is a lightweight Python adaptation, not
  Dynablox's TSDF/Voxblox implementation.
- `cluster_response_node.py` / `response_policy.py` -- convert confirmed,
  current tracks into a time-debounced stop request, with stale-input
  handling. The ROS node publishes the request and status; the plain-numpy
  policy holds the thresholds and hysteresis.
- `stop_actuation_node.py` / `actuation_policy.py` -- disabled-by-default
  boundary from a stop request to Unitree's `StopMove` API. Its default
  dry-run mode logs what it would send without creating a Unitree publisher.
- `tests/` -- unit tests for the plain Python perception and response
  modules (see "Testing" below).

## Output and basic response

`online_perception_node.py` publishes three views of each confirmed track:

- `/online_perception/markers` (`visualization_msgs/MarkerArray`) for RViz.
- `/online_perception/tracks` (`geometry_msgs/PoseArray`) as the
  machine-readable control boundary. It is published every processed
  frame, including an empty array when nothing is detected. Coasted
  predictions are excluded: a predicted position with no current
  measurement is useful for display and identity continuity, but is not
  strong enough evidence to trigger a robot response.
- `/online_perception/track_observations` (`std_msgs/String` containing
  JSON) for the optional inter-dog prototype. It preserves the local track
  ID, filtered velocity, cluster extent, point count, source timestamp,
  sensor position, frame name, and unique `DOG_ID`; coasted predictions are
  excluded. See `../merge/README.md` for the signed low-bandwidth exchange
  and shared-frame matching path.

Adding the rich JSON publisher does not change the existing MarkerArray or
PoseArray contents. The response policy still consumes only the local
PoseArray, so remote/shared tracks cannot affect actuation.

`cluster_response_node.py` measures those track positions from the current
FastLIO `/Odometry` position. A track within `stop_distance` (2.0 m by
default) continuously for `trigger_duration` (1.0 s) requests a stop.
Clearing requires no track within `clear_distance` (2.5 m) continuously for
`clear_duration` (1.0 s). Durations replace the original two-frame counters:
recorded degraded streams showed that a frame count can represent wildly
different real elapsed times. Distance is horizontal XY range by default,
not full 3D range, because centroid height should not make a ground-level
collision threat appear farther away.

Outputs are:

- `/online_perception/stop_requested` (`std_msgs/Bool`), preserving the
  original machine-readable boundary.
- `/online_perception/response_status` (`std_msgs/String` containing JSON),
  with the state, nearest range, and input ages for live diagnosis.

Tracks and odometry must both remain fresh (2.5 s defaults). Missing or stale
input requests a stop by default rather than interpreting a dead perception
process as a clear path. `fail_safe_stop:=false` exists for controlled
diagnostics, but should not be used when actuation is enabled.

`stop_actuation_node.py` is the separate robot boundary. It runs in both
Compose profiles but defaults to `enabled:=false`: dry-run mode logs
`would_send_stop_move` and never imports or publishes a Unitree request.
With `ACTUATION_ENABLED=true`, it publishes API ID 1003 (`StopMove`) on
`/api/sport/request`, once on the stop edge and then at a bounded repeat
rate. It also fails safe to StopMove if the response topic itself becomes
stale. Clearing the request never sends an automatic resume command; the
operator must resume manually after checking the scene.

This changes response semantics relative to the first prototype (elapsed
seconds and planar distance instead of frame counts and 3D distance) and
adds fail-safe stop requests on stale input. The detector/tracker and
`/online_perception/tracks` output are unchanged. Validate the status and
dry-run logs on bags and a stationary Go2 before setting
`ACTUATION_ENABLED=true`.

Validated end to end on 27 July 2026 with the rebuilt Docker image and the
`soton_indoor` bag: FastLIO produced 24 processed frames, perception
reproduced the same two known tracks, the response changed
`clear -> pending_stop -> stop` after the track remained within 2 m for
1 s, and cleared after it retreated. The adapter received the requests over
DDS and logged `would_send_stop_move` only. The pinned `unitree_api` package
also built and was discoverable with `ros2 interface show`. This validates
bag replay and dry-run wiring, not physical StopMove behaviour; enabled
actuation has not been run.

## Why this exists

The eventual goal is a pipeline that runs live on the Jetson rather than
"record bag -> ship to laptop -> replay through FastLIO -> run Python
pipeline" after the fact. Building that against a live Mid-360 isn't
possible yet (no LiDAR/Jetson reachable so far), so this runs the same
live-subscription code against a **replayed rosbag standing in for a live
sensor** instead. `ros2 bag play` publishes at real time by default, so
from this node's point of view a replayed bag looks the same as a live
sensor feed -- the only difference is where the messages originate. This is
a genuine step towards running on the Jetson, not a simulation of one; the
same script runs unmodified once a live sensor is available.

## Tracking

The offline pipeline this node ports from, and this node's own first
version, matched clusters between frames by greedily assigning each
detection to its nearest previous-frame centroid in array order, then
reported velocity as a raw two-point finite difference. That has three
concrete failure modes:

1. **Greedy matching isn't globally optimal.** If two tracked objects are
   near each other, whichever gets processed first can grab the other's
   correct match, leaving both slightly wrong instead of both correct.
2. **No motion model.** A track that goes unmatched for a single frame
   (occlusion, a gap in the point cloud, dropping below `min_points`) ends
   immediately; reappearing starts a new ID. There's no way to tell "this
   is the same person, briefly occluded" from "this is someone new."
3. **Raw finite-difference velocity is noisy.** DBSCAN's cluster mean moves
   by a few centimetres between frames from noise alone, even for someone
   standing still, and that noise goes straight into the reported speed.

`tracking.py` replaces this with:

- **Hungarian assignment** (`scipy.optimize.linear_sum_assignment`) between
  every active track's predicted position and this frame's detections,
  gated by `max_match_distance` -- the solver considers all pairings at
  once and picks the one that minimises total distance, rather than
  whichever pairing greedy iteration order happens to produce first.
- **A constant-velocity Kalman filter per track** (`KalmanTrack`), so
  position and velocity are both filtered state estimates rather than a
  single measurement and a two-point difference. This is also what gives a
  track a principled predicted position to coast on.
- **Coasting**: a track that goes unmatched keeps predicting forward for up
  to `max_missed_frames` frames, or `max_missed_seconds` of real elapsed
  time, whichever comes first (drawn faded out in RViz, see
  `online_perception_node.py`'s marker code) before being dropped, instead
  of ending on the first missed frame. Both limits matter: a frame *count*
  alone isn't reliable when the scan rate is degraded or bursty (see
  "Testing against more sessions" below) -- a handful of frames can span
  tens of real seconds, long enough that a constant-velocity coast runs off
  to somewhere implausible.
- **Track confirmation** (`min_hits`): a track isn't logged or published
  until it has `min_hits` real detections, not just its first one. Testing
  against several recorded sessions found that single-frame noise clusters
  -- long-range LiDAR noise, or on a moving sensor, a static object newly
  entering the field of view and looking like motion -- are common, and
  every single one of them got exactly one real detection and was never
  seen again. Requiring a second real detection before reporting a track
  filters this whole category out, at the cost of one extra frame of
  latency (with the default `min_hits=2`) before a genuinely new object is
  reported.
- **Odometry-based plausibility gate** (`filter_plausible_detections`,
  gated by `max_sensor_range`): detections farther than `max_sensor_range`
  from the robot's current `/Odometry` position are dropped before they
  ever reach the tracker. This is the one place `/Odometry` is actually
  used for something beyond logging -- see "Testing against more sessions"
  below for the failure mode it guards against.
- **Re-identification** (`reid_max_distance`, `reid_window_seconds`): a
  track dropped after coasting out keeps its last known position on hand
  for up to `reid_window_seconds`, and a new, otherwise-unmatched detection
  within `reid_max_distance` of it revives the original ID (and hit count)
  instead of starting a fresh one. This is deliberately a second, longer
  mechanism layered on top of coasting rather than an extension of it --
  see "Re-identification" below for why extending coasting itself would be
  the wrong fix, and for what this does and doesn't bridge.

## Re-identification: bridging genuine stops without trusting a long coast

Coasting (above) is deliberately short -- a second or two -- because a
constant-velocity extrapolation stops being trustworthy beyond that. A
real, deliberate stop (someone pausing, or a genuine multi-second
occlusion) lasts longer, so the track above gets dropped and the same
person gets a new ID on reappearing. This was flagged as an open gap in
the methodology review below (see `fallback_cardbox1`'s physical-occlusion
scenario) and is the clearest remaining identity-continuity problem the
visibility gate above doesn't touch -- it fixes false detections, not
this.

Rather than extending `max_missed_frames`/`max_missed_seconds` themselves
(which would mean coasting a Kalman prediction across the whole gap on
an increasingly untrustworthy velocity estimate), a dropped track's last
known *position* is kept in a short-lived pool (`CentroidTracker`'s
`_lost_tracks`) for up to `reid_window_seconds`. A new, otherwise-unmatched
detection within `reid_max_distance` of a pool entry revives that track's
original ID -- and its hit count, so a previously-confirmed track doesn't
lose its confirmed status and get reported with an extra frame of latency
as if it were brand new. Velocity resets to zero on revival rather than
carrying over the pre-gap estimate: the point of this path is bridging a
stop, so trusting the old velocity would defeat it.

**This deliberately only helps a genuine stop-in-place.** A detection
missed while the person kept walking -- rather than actually stopping --
can end up metres from where the track was lost by the time it
reappears, which no static-position gate can or should bridge (a generous
enough gate to catch that would just as easily misattribute an unrelated
object's detection to an old, stale ID). This is a real, observed
distinction, not a hypothetical caveat: manually reconstructing gaps in
`lab_walk_with_stops`'s track log found one gap where the last and next
positions were 7.6 m apart over a 5 s window (~1.5 m/s -- the person kept
walking, undetected, rather than stopping) alongside several genuine
stops with sub-2-metre drift over similar or longer gaps -- exactly the
distinction `reid_max_distance` is meant to draw.

**Validated by rerunning the same four sessions**, `reid_window_seconds`
set to 0 (which structurally disables re-identification -- there's no
route by which a same-step revival could apply, since a track can only
enter the lost-tracks pool after the assignment step has already run) for
the "off" comparison:

- Re-identification fired 12 times across 7 distinct original track IDs in
  `lab_walk_with_stops`, and once in `fallback_cardbox1`; zero times in
  `soton_indoor` or `lab_walk_no_stop` (both short sessions without a real
  stop-length gap). Observed real gaps it successfully bridged ranged from
  1 s to 12 s -- direct evidence for `reid_window_seconds=15.0`'s default
  being in the right range, not just reasoned from first principles.
- **The effect on the *distinct confirmed track* count is smaller than
  might be expected, and it's worth being honest about why.** Comparing
  internal track-ID allocation (not just what got confirmed and logged)
  tells the real story: in `lab_walk_with_stops`, IDs climbed to 61 with
  re-identification off versus only 24 with it on -- far fewer identities
  were ever allocated in the first place, because repeatedly-flickering
  detections kept reclaiming their original ID instead of spinning up a
  new one each time. But the *logged* count barely moved (14 -> 13),
  because `min_hits` was already suppressing most of those extra
  reappearances from ever being confirmed and logged in the "off" case too
  -- a flickering detection that gets only one hit before disappearing
  again isn't reported either way. Re-identification's real benefit here
  is fewer wasted internal tracks and immediate confirmed status on
  revival (since the hit count carries over), not a dramatically smaller
  reported track count on this particular dataset.
- No processing-time regression: 10-14ms mean, under 28ms worst case
  across all four sessions with re-identification on.

Unit tests (`perception/tests/test_tracking.py`) cover revival within the
distance/time gates, rejection beyond either gate, velocity resetting on
revival, and pool-entry expiry.

## Visibility gate: fixing the moving-sensor false-positive problem

The methodology review below (see "Testing against more recorded
sessions") identified and quantified a real problem the fixes above don't
touch: a moving sensor produces far more false "moved" detections than a
stationary one, because frame-to-frame nearest-neighbour change detection
can't tell "this direction was outside the previous frame's view" from
"something moved here." Rounding a corner, or just walking a metre closer
to a wall, brings genuinely new static geometry into `/cloud_registered`
that has no nearby point in the previous frame either -- exactly what the
change-detection test is looking for in a real object.

`range_image.py`'s `previously_visible_mask`, applied to the "moved" points
in `online_perception_node.py` right after the existing nearest-neighbour
test and before clustering, fixes this using `/Odometry` rather than
tracking-side filtering (which can narrow the problem but, as the
methodology review found, can't fully solve it). The approach:

1. Bin the previous frame's points into a spherical grid (azimuth/
   elevation) around the previous frame's own `/Odometry` position --
   this is a range image, the same representation used for LiDAR
   occlusion/dynamic-object reasoning in the wider literature (e.g.
   Removert, ERASOR). Only position is needed, not orientation -- see the
   module docstring for why the direction from a fixed origin to a
   world-frame point doesn't depend on which way the sensor body happened
   to be facing.
2. For each "moved" candidate point in the current frame, look up the
   previous frame's range image in the same direction from that same
   origin. No entry -> the previous scan never reached that direction
   (out of range, occluded, or just missed by sparse sampling -- doesn't
   matter which) -> not evidence of motion, drop it. An entry that's
   roughly the same range or farther -> the same static background,
   possibly revealed at a new range because whatever used to be in front
   of it (if anything) is no longer relevant -> also not motion, drop it.
   Only a point genuinely closer than the previous range along that same
   ray survives -- something now blocking a line of sight the previous
   scan had clear to a farther surface, the actual signature of an object
   moving into view.

`range_image_azimuth_bins`/`range_image_elevation_bins` (default 72/36,
5 degrees/bin) control the grid resolution. The first attempt used 2
degrees/bin (180/90) and made things worse, not better: FastLIO's sparse
~5k-points-per-frame output couldn't fill a grid that fine, so most of a
real walking-pace track's own bins came up empty in the previous frame
purely from sampling sparsity, not genuine unvisited directions, and the
gate wrongly dropped most of a real track along with the false positives
it was meant to catch (confirmed by rerunning `soton_indoor` and watching
one of its two known-good tracks disappear after frame 6). Coarsening to
5 degrees/bin fixed this -- rerunning the same session recovered both
tracks at comparable point counts and speeds to the ungated result. This
is a real tuning tradeoff, not a solved parameter: finer bins would
localise occlusion boundaries more precisely if FastLIO's output were
denser; these defaults are sized to the point density actually measured
against recorded sessions, not derived from the Mid-360's spec sheet.

`use_visibility_gate` (default `True`) turns this off entirely, for
before/after comparison against the same recorded session without a code
change -- see "Testing against more recorded sessions" below for the
comparison this was validated with.

`range_image_tolerance_ratio` (default `0.0`) optionally changes the fixed
0.3 m gate to `max(0.3 m, ratio * candidate range)`. Offline sweeps from
0.01 to 0.03 were exact no-ops on three recordings; 0.05 removed only 9–26
points per session and changed no confirmed track count. It remains
experimental and disabled because current data provides no benefit.

### Per-scan occlusion-accumulation experiment

`use_occlusion_accumulation` defaults to `False` and does not alter the
established detector. When enabled it replaces nearest-neighbour differencing
and the visibility gate with the corrected temporal range-image accumulator.
The node refuses this mode unless both `accumulate_scans` and
`accumulate_stride` are 1: running it on a ten-scan aggregate violates the
method's one-scan/one-pose assumption and performed badly offline.

The live experiment also disables range-image gap completion. On a 242-scan
`soton_indoor` export this lightweight configuration ran at about 2.2
ms/frame and followed one known trajectory for 28 measured frames, compared
with four measurements from the retuned per-scan baseline. Completion was
roughly twenty times slower and introduced a questionable intermittent
track. These are screening results without pointwise ground truth, so the
mode is for tomorrow's labelled A/B test, not response or actuation.

Start with:

```text
-p use_occlusion_accumulation:=true
-p accumulate_scans:=1
-p accumulate_stride:=1
-p cluster_min_points:=5
```

### Experimental free-space history

`use_free_space_detection` defaults to `False`. When enabled, it keeps the
established nearest-neighbour difference and visibility gate, then adds a
longer-term test adapted from Dynablox: a candidate must occupy a voxel that
repeated earlier rays established as free. Unknown space never activates;
spatially supported free evidence needs five observations by default; and
persistent occupancy clears stale free labels so odometry drift or a genuine
static-scene change does not poison the map permanently.

This is deliberately an adaptation rather than a port of the cloned Dynablox
repository. Dynablox uses ROS 1, Voxblox, and a fused TSDF; this project keeps a
sparse Python ray/voxel map so the idea can be screened inside the existing
ROS 2 and offline pipelines without adding a second mapping stack. The missing
TSDF surface model matters: using sparse free-space intrusion as a standalone
detector produced many static-boundary tracks, so the implemented mode uses it
only as an additional gate on the working detector.

The mode requires `accumulate_scans=1`, `accumulate_stride=1`, and odometry.
It is mutually exclusive with `use_occlusion_accumulation`. A controlled run
on the 242-scan `soton_indoor_dog1_scan1` export produced:

| metric | established | established + free-space gate |
|---|---:|---:|
| moved points | 7,514 | 3,696 |
| confirmed tracks | 1 | 1 |
| measured rows on that track | 4 | 4 |
| first confirmed frame | 90 | 90 |
| mean runtime per frame | 2.5 ms | 49.4 ms |

The same known trajectory survives and candidate points roughly halve, but no
track-level benefit is demonstrated and runtime is substantially higher. Keep
the mode disconnected from actuation and default-off until labelled
no-person, walking, crossing, and moving-dog recordings show whether the
discarded candidates are false positives rather than useful object support.

Start a live screening run with:

```text
-p use_free_space_detection:=true
-p accumulate_scans:=1
-p accumulate_stride:=1
-p cluster_min_points:=5
```

## Testing against more recorded sessions

Beyond the `soton_indoor` session used to validate the initial rewrite (see
"What's verified" below), the tracker was run against several other
recorded sessions to build confidence before attempting a Jetson port.
Full methodology: `docker compose --profile replay up` against each bag in
turn, log kept, track lifetimes reconstructed from the log (birth/death
time, position, real-detection count) and compared across runs.

**Processing time has a large margin.** Across every session tested, mean
processing time per frame was 8-40ms and the worst single frame was
70ms -- against a 1-second-per-frame budget (`accumulate_scans=10` at
FastLIO's ~10 Hz), that's under 10% even in the worst case seen, measured
on a laptop x86_64 CPU. This is optimistic for a Jetson's weaker
single-core performance and isn't proof the Jetson port will have the same
margin, but a pipeline already close to its budget on a laptop would have
been a hard "no" -- this instead leaves real headroom to work with. The
node logs a warning if any frame's processing time exceeds 50% of the
frame's real time span, as an early signal if this changes on different
hardware.

**Coasting works as designed for brief real gaps.** Rerunning the
`fallback_cardbox1` session (a person walking with intermittent occlusion
behind cardboard boxes) shows tracks correctly coasting through single
missed detections and re-matching, the same behaviour confirmed on
`soton_indoor`. It does *not*, by design, bridge a multi-second occlusion
or a deliberate stop -- a person hidden or stationary for longer than
`max_missed_frames`/`max_missed_seconds` gets a new track ID when they
reappear. Extending those limits to cover longer gaps isn't free (a longer
constant-velocity extrapolation is also a less trustworthy one -- see the
`max_missed_seconds` fix below); a proper fix would be re-identifying a new
detection against a recently-dropped track's last known position rather
than just coasting longer, which hasn't been built yet.

**A moving sensor produces substantially more false positives than a
stationary one -- confirmed and quantified, not just anecdotal.** Comparing
sessions:

| session | sensor | duration | distinct tracks logged (before confirmation filter) | after `min_hits` | after `min_hits` + visibility gate |
|---|---|---|---|---|---|
| `soton_indoor` | stationary | ~24s | 7 | 3 | 2 |
| `fallback_cardbox1` | stationary | ~86s | 38 | 11 | 6 |
| `lab_walk_no_stop` | walking, continuous | ~38s | 22 | 7 | 3 |
| `lab_walk_with_stops` | walking, stop-and-go | ~75s | 361 | 207 | 15 |

The last column is the visibility gate described above, added after this
table's original findings identified the residual problem `min_hits`
couldn't solve -- see "Visibility gate" above for the fix and "Validating
the visibility gate against the moving-sensor problem" below for the
validation run these numbers come from.

`lab_walk_with_stops` stood out sharply: mean "moved" points per frame was
843 (max 1548) versus `lab_walk_no_stop`'s mean of 263 (max 386) -- roughly
3x higher despite both being a walking dog. The most likely explanation is
the abrupt stop-start motion itself: each transition is a jolt that FastLIO's
registration has to re-settle after, and any brief registration jitter
shows up as widespread spurious "moved" points across the whole scene, not
just at the person's location -- this is a plausible mechanism, not
independently confirmed against FastLIO's internal state, since no ground
truth pose is available for this session. Either way, this was on top of the
already-documented issue (Kei's handover,
`kei-stuff/Multi-LiDAR Sensing.pdf`) that a moving sensor's own field of
view changing between frames makes newly-visible static geometry
indistinguishable from a moving object, since the change detector only
asks "was there a point near here in the previous frame," not "could this
location have been outside both frames' shared field of view." **Track
confirmation (`min_hits`) filtered the single-frame-noise half of this
problem well** (`lab_walk_with_stops` dropped from 361 to 207 distinct
tracks from that fix alone), but a large residual remained: 140 of those
tracks, spot-checked, were spatially and temporally consistent enough to
survive 2-3 real detections without being a real object -- tracking-side
fixes (better assignment, filtering, coasting) couldn't fully solve a
problem that originates in the detection step. The visibility gate, which
does address the detection step directly, took `lab_walk_with_stops` from
207 to 15 -- see "Validating the visibility gate against the moving-sensor
problem" below for the full before/after validation.

**Degraded/pre-DDS-fix recordings expose two real gaps, now fixed.**
Replaying `dog1/2026-05-13_14_41_dds_test` (recorded before the DDS
multicast whitelist and LiDAR timestamp fixes landed) showed FastLIO's own
registration failing intermittently on this data (`No point, skip this
scan!` and `lidar loop back, clear buffer` in its log) and, once, drifting
badly enough to put `/cloud_registered` points over a kilometre from
anywhere real. Before the fixes below, the tracker treated this exactly
like a real detection -- clustering and tracking phantom "objects" at
those positions with no indication anything was wrong. Two fixes address
this directly:

- `max_sensor_range` drops any detection implausibly far from the robot's
  current `/Odometry` position before it reaches the tracker.
- `max_missed_seconds` caps how long a track can coast in real time, not
  just in frame count -- the same session showed a track survive a
  42-second real gap because only 3 accumulated *frames* happened to occur
  in that stretch (the scan rate itself was degraded and bursty), long
  enough for its coasted Kalman prediction to run 15+ metres from its last
  real position. Re-running the same session after the fix confirms the
  track gets dropped instead.

This doesn't mean pre-DDS-fix recordings are good test data for tuning
detection accuracy -- they're testing exactly the pathological conditions
the DDS/timestamp fixes exist to prevent, and the node correctly does very
little useful work on them. What it does confirm is that a live sensor's
own instability (a bad scan, a registration hiccup, a temporary DDS issue)
won't silently produce confident-looking phantom detections -- it fails
safe instead of failing invisibly.

## Validating the visibility gate against the moving-sensor problem

Follow-up to the methodology review above, and the reason the visibility
gate exists. The problem it targets was the single biggest remaining
accuracy risk identified there: a moving sensor produces far more false
positives than a stationary one, and it's a detection-side problem
`min_hits` narrows but can't fully solve. Since a field deployment almost
certainly means a walking dog, this was worth resolving, not just noting,
before trusting this pipeline live.

Validated by rerunning all four sessions from the table above -- same
bags, same FastLIO instance, `use_visibility_gate` toggled with no other
code changes, both runs subscribing to the same live replay so the
comparison is against identical input, not two separate bag plays that
could differ in timing:

- **`soton_indoor` (stationary, the known-good baseline session): both
  real tracks survive.** 3 confirmed tracks (after `min_hits`) drop to 2,
  matching the handover's documented "2 real tracks" exactly -- the third
  was the false positive being removed, not a real detection lost. Track
  positions/speeds for the two survivors are within a few centimetres and
  a few hundredths of a m/s of the ungated run's own numbers, confirming
  the gate isn't just coincidentally arriving at the right count while
  changing which detections make it through.
- **Moving sessions show the same pattern the methodology review
  predicted, at scale.** `lab_walk_no_stop` 7 -> 3, `fallback_cardbox1`
  11 -> 6, `lab_walk_with_stops` 207 -> 15 (a 93% reduction on the session
  that motivated this work in the first place). Total "moved" points
  before clustering dropped 84-91% across all four sessions, including the
  *stationary* ones -- some of what the plain nearest-neighbour test
  flags even on a still sensor turns out to be edge/discretisation noise
  the visibility gate also catches, not exclusively a moving-sensor
  problem.
- **The surviving tracks look like real detections, not an
  over-aggressive filter left with nothing.** Every surviving track across
  every session reports a walking-pace speed (0.24-1.61 m/s) and a
  substantial point count (10-155 points), and in `lab_walk_no_stop`
  specifically, the surviving tracks' positions trace the same continuous
  path through the scene as the equivalent (higher-numbered, noisier)
  tracks in the ungated run -- e.g. gated track 1's path from (0.79, 2.42)
  to (3.67, 1.83) matches ungated track 10's (0.70, 2.43) to (4.13, 1.91)
  almost exactly. This is the same real walking person in both runs, not a
  different, coincidentally-plausible-looking detection.
- **Getting there took one real tuning correction, not just picking a
  resolution and moving on.** The first attempt (2 degrees/bin) made
  `soton_indoor` *worse*: one of the two known-good tracks disappeared
  after frame 6 because FastLIO's sparse ~5k-points-per-frame output
  couldn't fill a grid that fine, so many of a real track's own bins came
  up empty in the previous frame from sampling sparsity alone, not genuine
  unvisited directions -- the gate dropped real motion along with the
  false positives. Coarsening to 5 degrees/bin (the shipped default)
  fixed this specific failure and is what the numbers above reflect; see
  "Visibility gate" above for the reasoning and the resolution/density
  tradeoff this exposed.
- **Processing time stayed well inside budget.** 10-13ms mean, under 19ms
  worst case across all four sessions with the gate on -- if anything
  slightly lower than the pre-gate numbers in the table above, likely
  because clustering has far fewer points to work with once the gate has
  run. No frame in any session triggered the node's own 50%-of-budget
  warning.

**What this doesn't claim:** the gate is not proven to eliminate every
moving-sensor false positive, only to substantially reduce them, and
"looks like a plausible walking-pace track" (the check used above) is not
the same as independently confirmed ground truth -- no session here has
one. `lab_walk_with_stops`'s 15 surviving tracks, spread across a session
with only one person walking, are consistent with the known,
separately-documented re-identification gap (a person stopped for more
than `max_missed_seconds` gets a new track ID on resuming, rather than
being a fresh, unrelated false positive each time) rather than 15 distinct
real objects -- plausible given the session, not independently verified
detection-by-detection. The gate also only uses `/Odometry` position, not
orientation (see `range_image.py`'s docstring for why that's sufficient
for this specific check), and is tuned against four recorded sessions on
one sensor's characteristic point density, not a live sensor or a broader
set of scenes.

## Overlapping frames: an investigated, rejected optimisation

`accumulate_stride` (see the module docstring) lets frames overlap --
processing every `accumulate_stride` scans instead of waiting for a fresh
`accumulate_scans`-scan window each time, e.g. `accumulate_scans=10,
accumulate_stride=5` for 50% overlap. This was implemented and A/B tested
specifically to check whether it reduces detection latency (how long
after a new object appears before its track reaches `min_hits` and gets
reported), since more frequent frames means more frequent chances to
accumulate a confirming second hit.

**Tested against `lab_walk_no_stop`** (two instances of the node
subscribing to the same live FastLIO+bag-replay run, `accumulate_stride=10`
vs `accumulate_stride=5`, everything else default): the overlapping run
processed exactly 2x the frames (76 vs 38) for roughly 1.9x the total CPU
time (781ms vs 412ms across the whole session) -- unsurprising, and still
trivial against the processing budget either way. What it did *not* do is
reduce detection latency: the three real tracks were first confirmed
within about a second of each other between the two runs, in both
directions (one track was confirmed slightly *later* with overlap
enabled). The reason shows up directly in the per-frame "moved" point
counts: mean moved points per frame roughly halved (27.2 -> 11.3) under
50% overlap, because each frame now spans half the real time, so a moving
object has covered half the distance and displaced half as many points
past `change_threshold` by the time each frame is compared. Reporting
twice as often but with roughly half the signal each time nets out to
about the same *time* to reach `min_hits` confirmations, not half of it,
because confirmation latency here is paced by real elapsed time and
object speed, not by frame count.

**Kept as an opt-in parameter, not adopted as the default.** The
experiment was worth running -- it's exactly the kind of architecture
question flagged as open in earlier planning -- but it doubles compute
for no measured benefit on the one session tested, and "keep the working
non-overlapping behaviour as default, leave a tested escape hatch in
place" fits this project's priority right now (a good working single-LiDAR
implementation before more moving pieces) better than carrying extra
runtime complexity for an unproven win. Worth revisiting if a live sensor
ever shows different timing characteristics than this recorded-bag test,
or if `change_threshold`/`cluster_min_points` get re-tuned specifically
for a shorter inter-frame interval rather than reused unchanged from the
non-overlapping defaults, as they were here.

## Outdoor deployment session: a data-quality finding, not a clean test

`kei-stuff/ros2-go2/bag/2026-05-08_12_10_dog_2_outdoor_deployment` is the
largest recorded session (842 MB) and the only outdoor one with the kind
of scene this pipeline eventually needs to handle in the field (per the
handover's artefact catalogue: two walkers and a cyclist, dog 2). It
hadn't been tested against any version of this pipeline yet, so it was a
natural next session to run -- but what it actually revealed was about the
recording itself, not the tracker.

**The bag was missing its `metadata.yaml`** -- `ros2 bag play` can't open
a bag without one. `ros2 bag reindex <bag_dir>` (works directly against
the `.db3`, ROS 2 Humble) reconstructed it on a scratch copy without
touching the original file under `kei-stuff/`.

**LiDAR data only covers the first ~186 s of a ~1724 s (28.7-minute)
recording** -- confirmed directly from the bag's own message timestamps,
not inferred: `/livox/imu` spans the full 1724 s at a steady ~200 Hz with
no large gaps, while `/livox/lidar` stops entirely at the 186 s mark and
never resumes, despite IMU recording continuing for another 26 minutes.
The most likely explanation is the LiDAR driver dropping outdoors (WiFi/
DDS fragility is a documented risk elsewhere in this project) without
whoever was recording noticing before stopping the bag. This means the
two-walkers-and-a-cyclist scene described in the artefact catalogue, if it
happened later in the walk, is outside what this recording can actually
test -- worth flagging as a concrete argument for a "is `/livox/lidar`
still publishing" sanity check during future field recording, not just at
setup.

**FastLIO showed the same degraded-registration symptoms as the
pre-DDS-fix sessions** (`No Effective Points!`, `lidar loop back, clear
buffer`) during the ~186 s that did have LiDAR data -- so this recording
is closer to the pathological `dds_test` category than a clean baseline,
and its numbers shouldn't be read as representative outdoor performance.

**What was still worth checking, and did check out**: running the current
pipeline (visibility gate + re-identification, defaults) against this
segment (bounded to ~210 s of playback, matching the ~186 s of actual
LiDAR data) produced 17 confirmed tracks, all with plausible walking-pace
speeds (0-1.8 m/s, no cyclist-speed readings -- consistent with the
cyclist likely being later in the walk than this data covers) and
positions bounded within a ~20 m x ~13 m region, not scattered or
diverging. `max_sensor_range` never dropped a single detection as
implausible, meaning FastLIO's registration -- despite the warnings --
never actually produced a wildly drifted `/cloud_registered` point during
this window. This matches the design goal from the visibility-gate work:
degraded input produces a plausible-looking result or a clean rejection,
not a confident-looking phantom. It does **not** confirm the pipeline
correctly detected two walkers and a cyclist -- there's no ground truth
for this specific 186-second segment to check against.

## Performance profile and library choices

Every prior round of testing measured *total* per-frame processing time
(8-40ms mean, well inside budget); this round broke that down by pipeline
stage, using real per-frame timing added temporarily to
`online_perception_node.py` and run against `lab_walk_with_stops` (the
most demanding session so far), to answer the question "if this needs to
be faster on a Jetson, or point density increases, where would that time
actually have to come from" with data instead of a guess:

| stage | mean | share of total |
|---|---|---|
| KD-tree build + nearest-neighbour query (`scipy.spatial.cKDTree`) | 6.2ms | ~46% |
| Kalman predict/update + Hungarian assignment (`tracking.py`) | 2.4ms | ~17% |
| Voxel downsample (`pointcloud.py`) | 2.2ms | ~16% |
| Visibility gate (`range_image.py`) | 1.7ms | ~13% |
| DBSCAN clustering (`sklearn.cluster.DBSCAN`) | 1.1ms | ~8% |

**No library swap is justified right now.** Total mean processing time
(~13.6ms) is roughly 1% of the accumulate-scans real-time budget
(~1 second) on this laptop's x86_64 CPU -- even a generously pessimistic
5-10x slowdown for a Jetson's weaker per-core performance leaves 90%+ of
the budget spare. Specifically:

- **The KD-tree query is the single biggest cost, but scipy's `cKDTree`
  is already a well-optimised compiled implementation** -- there's no
  obvious faster off-the-shelf alternative for exact nearest-neighbour
  queries at this scale, and approximate-neighbour libraries (e.g.
  FAISS) trade accuracy for speed the pipeline doesn't need yet. Worth
  re-profiling if point density increases substantially (query cost
  scales with point count), since density is the improvement Kei's
  handover and this project's own findings both flag as the most
  impactful lever for detection quality generally -- but that's a reason
  to *re-profile later*, not to pre-optimise now.
- **DBSCAN turned out to be the cheapest stage, not a bottleneck worth
  replacing** -- worth recording since it's the opposite of what its
  reputation as "the slow clustering algorithm" might suggest; at this
  point count (a few hundred to low thousands of "moved" points per
  frame, not the tens of thousands DBSCAN's complexity concerns usually
  apply to) it's a non-issue.
- **The hand-rolled Kalman filter and Hungarian assignment were
  considered against `filterpy` (a well-tested Kalman filter library)
  and `lap`/`lapjv` (faster Hungarian solvers for large cost matrices)
  and rejected**, not overlooked: `tracking.py`'s filter is already
  unit-tested against known-good behaviour (constant-velocity
  convergence, coasting), simple enough to audit by reading it, and
  `scipy.optimize.linear_sum_assignment` is solving matrices with at
  most a handful of tracks -- `lap`'s speed advantage only matters at a
  scale (hundreds of simultaneous tracks) this pipeline is nowhere near.
  Adding a dependency to solve a performance problem that doesn't exist
  would be net-negative for maintainability, not neutral.

**The honest general conclusion**: this pipeline's bottleneck, if it
turns out to have one on real hardware, is far more likely to be FastLIO
itself (a heavier C++ SLAM system this project doesn't control the
internals of) than anything in `perception/`'s few milliseconds of numpy/
scipy/scikit-learn work. Effort is better spent on point-cloud density
and detection accuracy -- both flagged repeatedly elsewhere in this
project's findings as the real levers -- than on speeding up a stage
that's already using a tiny fraction of its budget.

## Testing

```bash
pip install numpy scipy scikit-learn pytest   # or: apt install python3-numpy python3-scipy python3-sklearn
python3 -m pytest perception/tests evaluation/tests export/tests merge/tests
```

These cover the pure clustering/assignment/Kalman-filter/plausibility-gate
logic in `tracking.py`, the PointCloud2 parsing/downsampling in
`pointcloud.py`, the range-image visibility check in `range_image.py`, and
the response/actuation state machines
-- no ROS install, no bag file, no running node needed, so they're the
fast repeatable check to run after touching any of the three modules. They
do not exercise `online_perception_node.py` itself (the ROS wiring) -- that
still needs the bag-replay check below.

## Running it

Needs the `docker/` image built first (`docker compose --profile hardware
build` there -- the `--profile` flag is required, see `docker/README.md`).
Easiest path is `docker/`'s `replay` profile, which runs FastLIO, this node,
and a bag replay together:

```bash
cd ../docker
cp .env.example .env
# edit .env: set BAG_PATH to a directory containing metadata.yaml + a .db3
# (e.g. kei-stuff/ros2-go2/bag/dog1/2026-05-12_16_21_soton_indoor)
docker compose --profile replay up
```

Or run the three pieces by hand, each in host networking so they share the
DDS domain:

```bash
# 1. FastLIO (subscribes to /livox/lidar + /livox/imu, publishes
#    /cloud_registered + /Odometry)
docker run -d --rm --name fastlio --network host go2-lidar-humble:latest fastlio

# 2. This node (subscribes to FastLIO's output, publishes
#    /online_perception/markers)
docker run -d --rm --name perception --network host \
    -v "$(pwd)":/opt/perception:ro \
    go2-lidar-humble:latest python3 /opt/perception/online_perception_node.py

# 3. Whatever's standing in for the sensor -- a bag, for now:
docker run --rm --network host \
    -v <path-to-a-kei-stuff-bag-directory>:/bag:rw \
    go2-lidar-humble:latest bash -c "ros2 bag play /bag"
```

Note the `:rw` mount on the bag, not `:ro` -- rosbag2's sqlite3 storage
plugin needs write access to the bag's own directory to open it at all,
even just for playback. A bag recorded without WAL/SHM sidecar files
already sitting next to its `.db3` (some of Kei's earlier single-dog
recordings, unlike the two-dog sessions that already carry them) fails
under `:ro` with a misleading "Could not load/open plugin with storage id
'sqlite3'" -- not an obvious permissions error, and easy to mistake for a
corrupt bag.

Watch detections with `docker logs -f go2-online-perception` (or
`perception` if run manually), or point RViz2 at
`/online_perception/markers` (frame `camera_init`, matching FastLIO's own
output frame) if running with a display available.

## What's verified vs still open

**Verified**, against `kei-stuff/ros2-go2/bag/dog1/2026-05-12_16_21_soton_indoor`
(the same session Kei's handover reports "2 real tracks detected" on,
stationary Dog 1, person walking past), rerun after the tracking rewrite
above (`docker compose --profile replay up`, image already built from an
earlier session so this ran off cache -- log excerpt kept here rather than
just the summary, per the project convention of noting what was actually
run):

- FastLIO (Humble) correctly fuses this Foxy-recorded bag's LiDAR/IMU data
  end to end -- `/Odometry` and `/cloud_registered` both publish at ~10 Hz,
  matching the handover's documented "known-good result" exactly.
- This node picks up two tracks (IDs 0 and 1) that persist across frames
  6-14 (9 consecutive frames, matching the previously documented count) at
  0.77-1.68 m/s (walking pace, close to the previously documented
  0.8-1.5 m/s range), alongside several single-frame noise blips at longer
  range (IDs 2-6) -- both the persistent tracks and the noise pattern match
  Kei's own documented finding for this exact session, and the earlier
  version of this node's own previously-recorded result.
- **The coasting improvement is directly visible in this run, not just
  theoretical**: track 0 has no matching detection at frame 10 (only 1
  cluster that frame, matched to track 1) and is reported `[coasting]` at
  its Kalman-predicted position; it re-matches a real detection at frame 11
  and continues as track 0, not a new ID. The previous greedy-matching
  version of this node would have ended track 0 at frame 10 and started a
  new track ID at frame 11 for the same person -- this is the exact failure
  mode the rewrite targeted, reproduced and fixed against real data in the
  same run.
- Track IDs 2-6, single-frame detections at the edge of the scene, coast
  for their full `max_missed_frames` window and then get dropped, rather
  than lingering forever -- confirms the coasting window is doing its job
  in both directions (keeping real tracks alive, not accumulating dead
  ones).

**Not yet done:**

- No live sensor tested -- only replayed bags standing in for one. The node
  itself doesn't know or care which it's talking to, but "the same code
  should work" and "confirmed working against a live sensor" are different
  claims -- don't conflate them in future notes.
- The moving-sensor false-positive rate is substantially reduced by the
  visibility gate (see "Validating the visibility gate against the
  moving-sensor problem" above) but not proven eliminated -- no session
  tested has independently confirmed ground truth, so "the surviving
  tracks look like plausible walking-pace detections" is the strongest
  claim actually supported so far, not "every remaining track is
  confirmed real."
- No parameter re-tuning against a live sensor yet. The detection-side
  parameter defaults are carried over from `track_motion.py`'s offline
  FastLIO-tuned values; the Kalman noise, `min_hits`, `max_missed_seconds`,
  `max_sensor_range`, `range_image_*`, and `reid_*` parameters are new and
  were validated against recorded sessions during this round of testing
  (see above) but not against live sensor timing/density.
- `/Odometry` is used for the plausibility gate and the visibility gate,
  but not yet for motion-compensating detections within the
  `accumulate_scans` window itself -- still worth checking whether
  FastLIO's own per-scan registration already covers everything needed
  here, independent of the FOV question the visibility gate addresses.
- Accumulate-then-cluster with a fixed-size window is still the
  architecture, but the specific "overlapping window might reduce
  latency" question is no longer open -- it was investigated and rejected
  with data, not left untried (see "Overlapping frames" above). What's
  still genuinely open is whether a different windowing scheme entirely
  (not just overlap) would help, which hasn't been explored.
- Re-identification after a real, multi-second occlusion or stop is
  **implemented and validated against recorded sessions** (see
  "Re-identification" above) -- this closes what was previously the
  clearest open gap here. What's still open: it only bridges a genuine
  stop-in-place (gated on the last known *position*), not a detection
  missed while the object kept moving, which a real session in this
  round of testing showed is also a real failure mode (a 7.6 m gap over
  5 s -- the person kept walking, undetected, rather than stopping) and
  which no static-position re-identification scheme can safely bridge.
- The outdoor deployment session (`2026-05-08_12_10_dog_2_outdoor_deployment`)
  turned out to be a data-quality problem, not a clean test -- see
  "Outdoor deployment session" above. The pipeline behaved safely against
  degraded input (no phantom far detections, bounded plausible positions),
  but the two-walkers-and-a-cyclist scene this session was meant to test
  against likely occurred later in the walk than the ~186s of actual LiDAR
  data the recording contains, so this specific claim is still untested.
  A short, clean outdoor recording (LiDAR confirmed publishing throughout)
  would be a more useful test than re-running this one.
