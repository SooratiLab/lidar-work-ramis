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
- `tests/` -- unit tests for `pointcloud.py` and `tracking.py` (see
  "Testing" below).

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

| session | sensor | duration | distinct tracks logged (before confirmation filter) |
|---|---|---|---|
| `soton_indoor` | stationary | ~24s | 7 |
| `fallback_cardbox1` | stationary | ~86s | 38 |
| `lab_walk_no_stop` | walking, continuous | ~38s | 22 |
| `lab_walk_with_stops` | walking, stop-and-go | ~75s | 361 |

`lab_walk_with_stops` stood out sharply: mean "moved" points per frame was
843 (max 1548) versus `lab_walk_no_stop`'s mean of 263 (max 386) -- roughly
3x higher despite both being a walking dog. The most likely explanation is
the abrupt stop-start motion itself: each transition is a jolt that FastLIO's
registration has to re-settle after, and any brief registration jitter
shows up as widespread spurious "moved" points across the whole scene, not
just at the person's location -- this is a plausible mechanism, not
independently confirmed against FastLIO's internal state, since no ground
truth pose is available for this session. Either way, this is on top of the
already-documented issue (Kei's handover,
`kei-stuff/Multi-LiDAR Sensing.pdf`) that a moving sensor's own field of
view changing between frames makes newly-visible static geometry
indistinguishable from a moving object, since the change detector only
asks "was there a point near here in the previous frame," not "could this
location have been outside both frames' shared field of view." **Track
confirmation (`min_hits`) filters the single-frame-noise half of this
problem well** (`lab_walk_with_stops` still dropped from 361 to 207
distinct tracks after the fix), but a large residual remains: 140 of those
tracks, spot-checked, were spatially and temporally consistent enough to
survive 2-3 real detections without being a real object -- tracking-side
fixes (better assignment, filtering, coasting) can't fully solve a problem
that originates in the detection step. See "Open risk for the Jetson port"
below.

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

## Open risk for the Jetson port

The single biggest remaining accuracy risk, based on the testing above, is
the moving-sensor false-positive rate -- specifically that it is a
*detection-side* problem (frame-to-frame nearest-neighbour change
detection can't distinguish "newly visible because the sensor's field of
view changed" from "genuinely moved"), not something addressable by
further tracking-side refinement. `min_hits` narrows it considerably but
doesn't solve it. A field deployment almost certainly involves a walking,
not stationary, dog, so this is worth resolving -- or at least
consciously deciding to accept -- before relying on this pipeline's output
in the field. The most promising direction identified but not yet
prototyped: use `/Odometry` to restrict change detection to the region of
the current frame that was also within the previous frame's field of view,
so newly-visible geometry at the edges stops being compared against
"nothing was there before" by construction. This is a detection-algorithm
change, not a tracking one, and hasn't been scoped in detail yet.

## Testing

```bash
pip install numpy scipy scikit-learn pytest   # or: apt install python3-numpy python3-scipy python3-sklearn
python3 -m pytest tests/
```

These cover the pure clustering/assignment/Kalman-filter/plausibility-gate
logic in `tracking.py` and the PointCloud2 parsing/downsampling in
`pointcloud.py` -- no ROS install, no bag file, no running node needed, so
they're the fast repeatable check to run after touching either module. They
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
    -v <path-to-a-kei-stuff-bag-directory>:/bag:ro \
    go2-lidar-humble:latest bash -c "ros2 bag play /bag"
```

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
- The moving-sensor false-positive rate (see "Open risk for the Jetson
  port" above) is the most important open accuracy question, and it's a
  detection-side problem this round of testing identified and quantified
  but didn't fix.
- No parameter re-tuning against a live sensor yet. The detection-side
  parameter defaults are carried over from `track_motion.py`'s offline
  FastLIO-tuned values; the Kalman noise, `min_hits`, `max_missed_seconds`,
  and `max_sensor_range` parameters are new and were validated against
  recorded sessions during this round of testing (see above) but not
  against live sensor timing/density.
- `/Odometry` is now used for the plausibility gate, but not yet for
  motion-compensating detections or restricting change detection to the
  overlapping field of view between frames -- the latter is the leading
  candidate fix for the moving-sensor false-positive problem above.
- Accumulate-then-cluster (fixed-size non-overlapping frames, mirroring
  `--accumulate 10` in the offline pipeline) is the simplest possible port
  of the existing algorithm, not necessarily the right online architecture
  -- a rolling/overlapping buffer might reduce detection latency and
  jitter, but hasn't been investigated.
- Multi-object re-identification after a real, multi-second occlusion or
  stop (as opposed to a single missed frame) isn't implemented -- see
  "Testing against more recorded sessions" above.
