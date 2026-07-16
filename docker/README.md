# go2-lidar-humble

Containerised build of the Go2 Mid-360 LiDAR stack -- `livox_ros_driver2` +
`FAST_LIO_ROS2` -- on ROS 2 Humble. Replaces the manual build steps in
`ros2-go2/go2-jetson-setup.md` with a reproducible image: one `docker build`
instead of cloning three repos, patching two of them by hand, and hoping the
Livox SDK didn't end up in the wrong directory.

Built and tested on x86_64 as a stand-in for the Jetson's aarch64. Both the
ROS base image and every upstream repo cloned in the `Dockerfile` support
both architectures, so this should build unmodified on the Jetson -- not yet
verified against real hardware (no Mid-360 or Jetson reachable while writing
this), see the repo's internal notes for what's still open.

## What's in here

```
Dockerfile                        image build: Livox-SDK2 + colcon workspace
entrypoint.sh                     renders per-dog LiDAR config, dispatches launch
patch_livox_timestamp.py          LiDAR/IMU timestamp fix, applied at build time
docker-compose.yml                driver + FastLIO + bag-replay + perception + export services
.env.example                      copy to .env, set LIVOX_LIDAR_IP (and BAG_PATH for replay/export)
config/
  MID360_config.json.template     driver config with ${LIVOX_LIDAR_IP} etc.
  fastrtps_eth0_only.xml           DDS multicast whitelist (loopback + Ethernet only)
```

`docker-compose.yml` has three profiles:

- **`hardware`** -- `driver` + `fastlio`, for a real Mid-360 plugged into
  this Jetson.
- **`replay`** -- `fastlio` + `bag` + `perception`, for testing without a
  real sensor by replaying one of Kei's recorded bags against a live
  FastLIO instance instead. See `../perception/README.md` for what that's
  actually validated so far.
- **`export`** -- `fastlio` + `bag` + `export`, for turning a bag into the
  `pcd/` + `poses.csv` layout `../evaluation/` expects, without a live
  ROS graph to watch or a separate node to run. See `../export/README.md`.

Every service needs a profile flag -- there is no profile-less default
service in this file, so a bare `docker compose build`/`up` matches nothing
and does nothing (Compose prints "No services to build" rather than an
error, which is easy to miss).

## Build

```bash
docker compose --profile hardware build
```

## Run

```bash
cp .env.example .env
# edit .env: set LIVOX_LIDAR_IP to whichever Mid-360 is plugged into this dog
docker compose --profile hardware up
```

This starts both the LiDAR driver and FastLIO. `docker-compose.yml` uses
`network_mode: host` -- the Livox SDK binds directly to the host's
LiDAR-subnet address rather than going through Docker's usual NAT'd bridge
network, so bridge networking fails outright with `bind failed` (confirmed
while testing this image without a real LiDAR-subnet interface present).
Host networking isn't a convenience here, it's required.

To run just one piece:

```bash
docker compose --profile hardware up driver     # LiDAR driver only
docker compose --profile hardware up fastlio    # FastLIO only (needs the driver already running)
docker compose --profile hardware run --rm driver bash   # shell inside the built image
```

No sensor to test against? `.env` also takes a `BAG_PATH` (a directory with
`metadata.yaml` + a `.db3` file, e.g. one of Kei's recordings under
`kei-stuff/ros2-go2/bag/`) for the `replay` profile:

```bash
docker compose --profile replay up
```

This runs FastLIO against a replayed bag instead of a live driver, plus the
online perception node from `../perception/`. See `../perception/README.md`
for what this has actually produced against real recorded data.

The same `BAG_PATH` also works with the `export` profile, which swaps the
live perception node for a one-shot batch export to PCD files + a poses
CSV instead:

```bash
BAG_PATH=/path/to/bag EXPORT_OUTPUT_DIR=/path/to/write/into docker compose --profile export up
```

`EXPORT_OUTPUT_DIR` defaults to `../data/export` if unset; `EXPORT_ACCUMULATE`
(default 10, matching `export_fastlio.py`'s own default) controls how many
scans get merged into each output frame. Stop with Ctrl+C once the `bag`
service's log shows playback finished -- see `../export/README.md` for
what this produces and why SIGTERM/SIGINT both flush cleanly instead of
losing whatever's still in the accumulation buffer.

## What's already fixed vs what's still open

Baked into the image at build time:

- **LiDAR timestamp patch** (`patch_livox_timestamp.py`) -- the Mid-360
  falsely reports a PTP-synced timestamp it doesn't actually have, which
  desyncs LiDAR and IMU timestamps by 20-35 seconds and stops FastLIO from
  fusing any data. Applied via a content-based Python patch (not a
  line-number patch) so it keeps working if upstream shifts line numbers.
- **DDS multicast whitelist** (`config/fastrtps_eth0_only.xml`) -- keeps
  DDS traffic off WiFi, so two dogs sharing a non-enterprise network don't
  cross-talk and drop each other's scan rate.
- **No `ConstSharedPtr` patch for FAST_LIO_ROS2.** That patch was needed on
  ROS 2 Foxy because the upstream fork is written against Humble's service
  callback API. On Humble it isn't needed -- confirmed by building clean
  without it.

Verified so far, by actually running it, not just building it:

- Both packages build clean under `-DDISTRO_ROS=humble`; the rendered
  LiDAR config picks up `LIVOX_LIDAR_IP`/`LIVOX_HOST_IP` correctly; both
  launch files parse and their arguments resolve.
- The driver fails with the expected, known `bind failed` error when no
  real LiDAR-subnet interface is present, matching the documented
  real-hardware failure mode exactly rather than being a container-specific
  bug.
- **FastLIO (from this image) correctly processes a real recorded Mid-360
  bag end to end**: replayed one of Kei's Foxy-recorded sessions against
  this image's `fastlio` service and got `/Odometry` + `/cloud_registered`
  both publishing at ~10 Hz, matching the handover doc's documented
  known-good result exactly. This isn't a live sensor, but it is real
  recorded LiDAR/IMU data being fused correctly by this specific image --
  see `../perception/README.md` for the full record, including a downstream
  perception result built on top of it.
- The full `docker compose --profile replay up` workflow (not just manual
  `docker run` commands) has been run end to end and reproduces the same
  result.

Not yet verified: an actual Mid-360 talking to this container, an actual
Jetson build, and whether CycloneDDS's node-creation segfault (which forced
FastRTPS on the Foxy setup this replaces) still happens on Humble. FastRTPS
is kept as the default RMW here because it's the combination proven to
work, not because CycloneDDS is assumed still broken.
