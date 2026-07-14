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
docker-compose.yml                driver + FastLIO services, host networking
.env.example                      copy to .env, set LIVOX_LIDAR_IP per dog
config/
  MID360_config.json.template     driver config with ${LIVOX_LIDAR_IP} etc.
  fastrtps_eth0_only.xml           DDS multicast whitelist (loopback + Ethernet only)
```

## Build

```bash
docker compose build
```

## Run

```bash
cp .env.example .env
# edit .env: set LIVOX_LIDAR_IP to whichever Mid-360 is plugged into this dog
docker compose up
```

This starts both the LiDAR driver and FastLIO. `docker-compose.yml` uses
`network_mode: host` -- the Livox SDK binds directly to the host's
LiDAR-subnet address rather than going through Docker's usual NAT'd bridge
network, so bridge networking fails outright with `bind failed` (confirmed
while testing this image without a real LiDAR-subnet interface present).
Host networking isn't a convenience here, it's required.

To run just one piece:

```bash
docker compose up driver           # LiDAR driver only
docker compose up fastlio           # FastLIO only (needs the driver already running)
docker compose run --rm driver bash  # shell inside the built image
```

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

Verified so far (see repo notes for the full record): both packages build
clean under `-DDISTRO_ROS=humble`; the rendered LiDAR config picks up
`LIVOX_LIDAR_IP`/`LIVOX_HOST_IP` correctly; both launch files parse and
their arguments resolve; the driver fails with the expected, known
`bind failed` error when no real LiDAR-subnet interface is present, which
matches the documented real-hardware failure mode exactly rather than being
a container-specific bug.

Not yet verified: an actual Mid-360 talking to this container, an actual
Jetson build, and whether CycloneDDS's node-creation segfault (which forced
FastRTPS on the Foxy setup this replaces) still happens on Humble. FastRTPS
is kept as the default RMW here because it's the combination proven to
work, not because CycloneDDS is assumed still broken.
