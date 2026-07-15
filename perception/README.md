# perception

Live/online counterpart to `kei-stuff/lidar-perception/scripts/track_motion.py`.
That script batch-processes a folder of pre-exported PCD files; this
subscribes to FastLIO's `/cloud_registered` and `/Odometry` directly and
runs the same accumulate -> diff -> cluster pipeline as each message
arrives, with an improved tracker on top -- see "Tracking" below for what
changed and why.

## Layout

- `online_perception_node.py` -- the ROS 2 node: subscribes to FastLIO's
  topics, accumulates scans into frames, publishes RViz markers. The only
  non-ROS logic left in it is a trivial colour-picker for track markers,
  not worth splitting out for its own tests.
- `pointcloud.py` -- PointCloud2 parsing and voxel downsampling. Plain
  numpy, no rclpy dependency, so it's unit-testable on its own.
- `tracking.py` -- DBSCAN clustering of the moved points, and the
  frame-to-frame centroid tracker (Kalman filter + Hungarian assignment +
  coasting). Plain numpy/scipy/scikit-learn, no rclpy dependency.
- `range_image.py` -- the odometry-referenced visibility gate that
  suppresses "moved" points a moving sensor's own viewpoint change
  produced, rather than something actually moving -- see "Visibility gate"
  below. Plain numpy, no rclpy dependency.
- `tests/` -- unit tests for `pointcloud.py`, `tracking.py`, and
  `range_image.py` (see "Testing" below).

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

## Testing

```bash
pip install numpy scipy scikit-learn pytest   # or: apt install python3-numpy python3-scipy python3-sklearn
python3 -m pytest tests/
```

These cover the pure clustering/assignment/Kalman-filter/plausibility-gate
logic in `tracking.py`, the PointCloud2 parsing/downsampling in
`pointcloud.py`, and the range-image visibility check in `range_image.py`
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
  `max_sensor_range`, and `range_image_*` parameters are new and were
  validated against recorded sessions during this round of testing (see
  above) but not against live sensor timing/density.
- `/Odometry` is used for the plausibility gate and the visibility gate,
  but not yet for motion-compensating detections within the
  `accumulate_scans` window itself -- still worth checking whether
  FastLIO's own per-scan registration already covers everything needed
  here, independent of the FOV question the visibility gate addresses.
- Accumulate-then-cluster (fixed-size non-overlapping frames, mirroring
  `--accumulate 10` in the offline pipeline) is the simplest possible port
  of the existing algorithm, not necessarily the right online architecture
  -- a rolling/overlapping buffer might reduce detection latency and
  jitter, but hasn't been investigated.
- Multi-object re-identification after a real, multi-second occlusion or
  stop (as opposed to a single missed frame) isn't implemented -- see
  "Testing against more recorded sessions" above. `lab_walk_with_stops`'s
  15 surviving tracks after the visibility gate are the clearest evidence
  this still matters: consistent with one person's walk being split across
  many IDs by repeated stops, not 15 separate objects.
