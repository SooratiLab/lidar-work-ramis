# perception

Live/online counterpart to `kei-stuff/lidar-perception/scripts/track_motion.py`.
That script batch-processes a folder of pre-exported PCD files; this
subscribes to FastLIO's `/cloud_registered` and `/Odometry` directly and
runs the same accumulate -> diff -> cluster -> track pipeline as each
message arrives.

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

## Running it

Needs the `docker/` image (`docker compose build` there first). Three
pieces run together, each in host networking so they share the DDS domain:

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

Or via `docker-compose.yml`'s `replay` profile (`BAG_PATH` set in `.env`):

```bash
docker compose --profile replay up
```

Watch detections with `docker logs -f perception`, or point RViz2 at
`/online_perception/markers` (frame `camera_init`, matching FastLIO's own
output frame) if running with a display available.

## What's verified vs still open

**Verified**, against `kei-stuff/ros2-go2/bag/dog1/2026-05-12_16_21_soton_indoor`
(the same session Kei's handover reports "2 real tracks detected" on,
stationary Dog 1, person walking past):

- FastLIO (Humble) correctly fuses this Foxy-recorded bag's LiDAR/IMU data
  end to end -- `/Odometry` and `/cloud_registered` both publish at ~10 Hz,
  matching the handover's documented "known-good result" exactly.
- This node picks up two tracks that persist across ~9 consecutive frames
  at 0.8-1.5 m/s (walking pace), alongside several single-frame noise
  blips at longer range -- both the persistent tracks and the noise pattern
  match Kei's own documented finding for this exact session almost exactly.
  This is a real, verified result against real data, not a synthetic test.

**Not yet done:**

- No live sensor tested -- only a replayed bag standing in for one. The
  node itself doesn't know or care which it's talking to, but "the same
  code should work" and "confirmed working against a live sensor" are
  different claims -- don't conflate them in future notes.
- No re-tuning yet. The parameter defaults are carried over from
  `track_motion.py`'s offline FastLIO-tuned values (see the module
  docstring) -- they happened to reproduce Kei's result on this session,
  which is a good sign, not proof they're right for other sessions or for
  a live sensor's actual timing/density.
- `/Odometry` is subscribed but not yet used for anything -- see the module
  docstring. Whether it's needed at all depends on how much of FastLIO's
  own registration already covers what it would be used for.
- Accumulate-then-cluster (fixed-size non-overlapping frames, mirroring
  `--accumulate 10` in the offline pipeline) is the simplest possible port
  of the existing algorithm, not necessarily the right online architecture
  -- see `../TODO.md`'s Phase 2 notes on this.
- No perception-side automated tests -- verification so far is "ran it
  against a real bag and read the log," which is a legitimate first check
  but not a substitute for something repeatable.
