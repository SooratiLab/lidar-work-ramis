# export

Ports `kei-stuff/ros2-go2/scripts/export_fastlio.py` into a `docker-compose`
service alongside this repo's own containerised Humble FastLIO
(`../docker/`), so turning a new bag into `../evaluation/`'s expected
`pcd/frame_*.pcd` + `poses.csv` layout no longer needs Kei's original
three-terminal WSL2/Jazzy workflow on a second machine/ROS distro.
Same subscriptions, same output format, same mm/m unit split as the
original -- what changed is how it's run and how the PointCloud2 parsing
itself works. See `export_fastlio.py`'s own module docstring for the full
detail; this README covers usage and what's actually been verified.

## What changed from Kei's version

- **Runs as a `docker-compose` service** (`../docker/docker-compose.yml`'s
  `export` profile) against this repo's own Humble FastLIO container,
  instead of a manual step in a Jazzy/WSL2 workflow on a separate laptop
  setup. `docker/README.md` documents the one-command version.
- **Vectorised PointCloud2 parsing** (`read_pointcloud2_xyzi` -- a numpy
  structured dtype + `np.frombuffer`, the same approach
  `perception/pointcloud.py` already uses for the same message type)
  instead of Kei's per-point `struct.unpack` loop. This callback runs once
  per incoming scan (10 Hz, thousands of points each), not once per
  recording, so the loop was a real cost worth removing while porting
  this anyway.
- **`--accumulate` now defaults to 10** (Kei's default was 1, i.e. one PCD
  per raw scan) -- 10 scans at ~10 Hz gives ~1-second frames, matching
  `perception/`'s and `evaluation/`'s own `accumulate_scans` default and
  every tuned detection parameter documented throughout this repo, all of
  which assume 1-second frames. Pass `--accumulate 1` explicitly for the
  old default.
- **Handles `docker-compose stop`/`down` (SIGTERM), not just a terminal
  Ctrl+C (SIGINT)** -- both now reach the same flush-and-save-poses code
  path before the process exits. See "Shutdown handling" below for why
  this needed more than routing SIGTERM through the same handler as
  SIGINT.

## Usage

Recommended -- one `docker-compose` command, no second OS/ROS distro:

```bash
cd ../docker
cp .env.example .env   # same LIVOX_LIDAR_IP/LIVOX_HOST_IP as any other profile
BAG_PATH=/path/to/bag EXPORT_OUTPUT_DIR=/path/to/write/into docker compose --profile export up
# watch the bag service's log for playback finishing, then Ctrl+C
```

`EXPORT_OUTPUT_DIR` defaults to `../data/export`; `EXPORT_ACCUMULATE`
defaults to 10. See `../docker/README.md`'s `export` profile section for
the full compose wiring.

Manual -- against any already-running ROS graph (a live sensor once one's
reachable, or FastLIO running outside Docker):

```bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml rviz:=false   # T1
python3 export_fastlio.py ../data/2026-07-16_walk --accumulate 10             # T2
ros2 bag play <bag_path>                                                      # T3, if replaying
```

Either way, output lands at `<output_dir>/pcd/frame_NNNNNN.pcd` (binary
PCD, x/y/z/intensity, coordinates in **millimetres**) and
`<output_dir>/poses.csv` (frame, timestamp, x, y, z, qx, qy, qz, qw, in
**metres**) -- exactly what `../evaluation/pcd_io.py` and
`../evaluation/compare_pipelines.py` already expect, since that's the
format Kei's original script already produced and this is a port, not a
new format.

## Shutdown handling

`docker-compose stop`/`down` sends SIGTERM, not SIGINT, and Python's
default SIGTERM handling is to exit immediately with no cleanup -- which
would silently drop whatever's in the accumulation buffer and skip
`poses.csv` entirely. The first attempt at fixing this
(`signal.signal(signal.SIGTERM, ...)` to raise a `KeyboardInterrupt`, the
same exception a bare Ctrl+C's SIGINT already produces) turned out to
**break shutdown entirely** on testing: `rclpy.init()` installs its own
SIGINT/SIGTERM handling by default, which wakes a blocked `rclpy.spin()`
via an internal guard condition; overriding it with a plain Python
handler removes that wake-up without providing a replacement, so `spin()`
then blocked forever on SIGTERM instead of returning. Confirmed directly:
a minimal `rclpy.spin()` under a custom SIGTERM handler installed after
`rclpy.init()` never woke up, while the same script with no custom
handler at all raised `rclpy.executors.ExternalShutdownException` and
exited within a second of the signal. The fix actually shipped just
catches that exception (alongside `KeyboardInterrupt`, kept for a bare
terminal Ctrl+C) around `spin()`, and uses `rclpy.try_shutdown()` instead
of `rclpy.shutdown()` at the end of `main()` since the context is already
shut down by the time control gets there.

One cosmetic side effect of this: `node.get_logger().info(...)` calls
made after the context is already shut down (i.e. everything logged
during the flush-and-save path) print a `Failed to publish log message to
rosout: publisher's context is invalid` warning to stderr. This is
harmless -- confirmed it happens identically on a plain Ctrl+C (SIGINT) as
well, so it's inherent to how this rclpy version handles any
signal-triggered shutdown, not something this port introduced or could
avoid by handling SIGTERM differently.

## What's verified

Ran the full `docker compose --profile export up` path against
`kei-stuff/ros2-go2/bag/dog1/2026-05-12_16_21_soton_indoor` -- the same
`soton_indoor` bag used throughout `perception/`'s and `evaluation/`'s own
testing, chosen so the result is directly comparable to the "2 tracks
detected" result documented elsewhere in this repo rather than an
unfamiliar session with nothing to check it against:

- FastLIO (this image's `fastlio` service) produced `/cloud_registered`
  at ~7.5k points/frame and `/Odometry`, matching the density and rate
  documented everywhere else in this repo for the same bag.
- The `export` service wrote 25 accumulated frames (`--accumulate 10`,
  the default) from 243 raw scans, and `poses.csv` with 243 pose rows --
  one per raw scan, as designed.
- `docker compose --profile export down`, sent after the `bag` service's
  log showed playback had finished, correctly flushed the partial 25th
  frame's buffered scans and wrote `poses.csv` via the SIGTERM path
  described above, not just on a manual Ctrl+C.
- Loaded the exported frames back with `evaluation/pcd_io.py` directly --
  coordinates land in the expected millimetre range for an indoor scene
  (roughly -12 m to +6.5 m room-scale extent), confirming the mm scaling
  survived the port correctly.
- Fed the exported session straight into
  `evaluation/compare_pipelines.py` (no `--kei-tracks`, just running the
  current pipeline): **2 confirmed tracks**, walking-pace speeds
  (0.78-1.44 m/s) -- the same known-good result this repo already
  documents for `soton_indoor` via every other path (the original
  WSL2/Jazzy export, and the live `perception/online_perception_node.py`
  subscribed directly to a bag replay), now reproduced from data that
  went through this exporter and never touched WSL2 or Jazzy at all.

## Testing

```bash
cd export
python3 -m pytest tests/
```

`test_export_fastlio.py` covers `read_pointcloud2_xyzi` (including the
no-intensity-field fallback) and round-trips `write_pcd_binary`'s output
through `evaluation/pcd_io.py`'s reader directly, rather than
re-implementing a second PCD parser just for the test -- a passing
round-trip there is direct evidence the two stay compatible, which is the
actual thing worth checking. Neither test needs rclpy installed, since
both functions only touch plain data (a PointCloud2-shaped object's
attributes, or a numpy array).

## Honest caveats

- Verified against one session (`soton_indoor`) end to end; not yet
  re-run against every other recorded session the way `perception/`'s
  tracker has been. The result matching the known-good "2 tracks" figure
  on this one session is a good sign the port is correct, not proof every
  session round-trips identically.
- Built and tested on x86_64 as a stand-in for the Jetson's aarch64, same
  as everything else in `docker/` -- not yet run on real Jetson hardware.
- No live Mid-360 tested, only a replayed bag standing in for one -- same
  caveat as every other path in this repo so far.
