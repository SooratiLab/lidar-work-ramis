# perception

Live/online counterpart to `kei-stuff/lidar-perception/scripts/track_motion.py`.
That script batch-processes a folder of pre-exported PCD files; this
subscribes to FastLIO's `/cloud_registered` and `/Odometry` directly and
runs the same accumulate -> diff -> cluster pipeline as each message
arrives, with an improved tracker on top -- see "Tracking" below for what
changed and why.

## Layout

- `online_perception_node.py` -- the ROS 2 node: subscribes to FastLIO's
  topics, accumulates scans into frames, publishes RViz markers. Owns
  nothing that isn't ROS-specific.
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
possible yet (no LiDAR/Jetson reachable while writing this -- see
`../TODO.md`), so this runs the same live-subscription code against a
**replayed rosbag standing in for a live sensor** instead. `ros2 bag play`
publishes at real time by default, so from this node's point of view a
replayed bag looks the same as a live sensor feed -- the only difference is
where the messages originate. This is a genuine step towards Phase 2, not a
simulation of one; the same script runs unmodified once a live sensor is
available.

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
  to `max_missed_frames` frames (drawn faded out in RViz, see
  `online_perception_node.py`'s marker code) before being dropped, instead
  of ending on the first missed frame.

## Testing

```bash
pip install numpy scipy scikit-learn pytest   # or: apt install python3-numpy python3-scipy python3-sklearn
python3 -m pytest tests/
```

These cover the pure clustering/assignment/Kalman-filter logic in
`tracking.py` and the PointCloud2 parsing/downsampling in `pointcloud.py` --
no ROS install, no bag file, no running node needed, so they're the fast
repeatable check to run after touching either module. They do not exercise
`online_perception_node.py` itself (the ROS wiring) -- that still needs the
bag-replay check below.

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

- No live sensor tested -- only a replayed bag standing in for one. The
  node itself doesn't know or care which it's talking to, but "the same
  code should work" and "confirmed working against a live sensor" are
  different claims -- don't conflate them in future notes.
- Only this one session re-tested after the tracking rewrite. The fallback
  cardboard-box and two-dog sessions (better tests of occlusion/coasting,
  since they involve objects actually blocking the sensor's view rather
  than just a detector miss) haven't been run against the new tracker yet.
- No re-tuning yet. The detection-side parameter defaults are carried over
  from `track_motion.py`'s offline FastLIO-tuned values (see the module
  docstring) -- they happened to reproduce Kei's result on this session,
  which is a good sign, not proof they're right for other sessions or for
  a live sensor's actual timing/density. The Kalman noise parameters are
  new and have had no tuning pass at all yet, just reasoned defaults.
- `/Odometry` is subscribed but not yet used for anything -- see the module
  docstring. Whether it's needed at all depends on how much of FastLIO's
  own registration already covers what it would be used for.
- Accumulate-then-cluster (fixed-size non-overlapping frames, mirroring
  `--accumulate 10` in the offline pipeline) is the simplest possible port
  of the existing algorithm, not necessarily the right online architecture
  -- see `../TODO.md`'s Phase 2 notes on this.
