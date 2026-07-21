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

## First live run on a Go2 Jetson

This is the lowest-friction route from a freshly accessible Jetson to a
useful live test. Do one dog at a time first. The two known Mid-360 addresses
are `192.168.1.137` for Dog 1 and `192.168.1.120` for Dog 2; the address belongs
to the physical LiDAR, so use the other value if the sensors have been swapped.

### 1. Preflight the host and network

Keep the robot supported or standing still for the first test, connect the
Mid-360 to the rear XT30 Ethernet port, and SSH to the Jetson over WiFi or
Tailscale. Run the deployment inside `tmux` so losing SSH does not stop it.

```bash
tmux new -s lidar-live

# Record exactly what platform is under test. JetPack 6 should report Ubuntu
# 22.04; the container remains the same if a Jetson is still on JetPack 5.
uname -m                         # expect aarch64
cat /etc/os-release
cat /etc/nv_tegra_release
docker version
docker compose version
df -h /var/lib/docker .         # leave ample room for the multi-GB build

# eth0 must carry both the Go2 and LiDAR subnets, with no eth0 default route.
ip -br addr show eth0
ip route
ping -c 3 192.168.1.137         # Dog 2: 192.168.1.120
```

Expected on `eth0`: `192.168.123.18/24` and `192.168.1.50/24`. There should
not be a `default via 192.168.123.1` route. If either is wrong, fix the Jetson
netplan using `kei-stuff/ros2-go2/go2-jetson-setup.md` sections 4 and 7 before
debugging Docker. A failed LiDAR ping is likewise a cable/address/network
problem, not a perception problem.

The image does not use CUDA, so `nvidia-container-runtime` is not required for
this test. Docker Engine plus the Compose plugin is sufficient.

### 2. Copy the code and build once on the Jetson

Clone or pull this repository on the Jetson, then configure the physical
sensor. Do not copy another dog's `.env` without checking the address.

```bash
cd ~/go2-stuff/lidar-work-ramis/docker
cp .env.example .env
nano .env                       # set LIVOX_LIDAR_IP; host IP is normally unchanged

# Build can be slow on the Jetson. Keep it in tmux and do not start a field
# session until this has completed successfully.
docker compose --profile hardware build
```

The Dockerfile is intentionally multi-architecture: on the Jetson it pulls
the arm64 ROS image and compiles arm64 dependencies locally. A failure while
cloning or running `rosdep` usually means the Jetson lost its internet route;
check `ip route` and WiFi before changing the Dockerfile.

Before involving hardware, a short recorded-bag replay is the best smoke test
if a bag is already present on the Jetson:

```bash
# Add BAG_PATH=/absolute/path/to/bag to .env, then:
docker compose --profile replay up
```

That separates image/DDS/perception failures from live Livox networking. Stop
it with Ctrl+C after `go2-online-perception` has logged several processed
frames.

### 3. Start the live stack and verify it from the bottom up

```bash
docker compose --profile hardware up -d
docker compose --profile hardware ps
docker compose --profile hardware logs -f
```

In a second SSH/tmux window, check every boundary in order. Each command is
run inside the already-configured FastLIO container, avoiding a second host
ROS installation or mismatched DDS environment.

```bash
# Raw sensor: both must publish. LiDAR should normally be about 10 Hz.
docker exec go2-fastlio timeout 15 ros2 topic hz /livox/lidar
docker exec go2-fastlio timeout 15 ros2 topic hz /livox/imu

# FastLIO: both should settle near 9-10 Hz.
docker exec go2-fastlio timeout 15 ros2 topic hz /cloud_registered
docker exec go2-fastlio timeout 15 ros2 topic hz /Odometry

# Perception output. It needs two accumulated frames before it can compare
# motion, so allow roughly 2-3 seconds with the default 10-scan windows.
docker logs --since 2m go2-online-perception
```

Success means all four topics publish continuously and perception logs lines
of the form `frame ... processed in ...ms`. Confirmed tracks appear as
`track N [new|matched|coasting|reidentified]`. No tracks in an empty, static
scene is healthy; frame processing is the liveness check. Walk a person across
the LiDAR view before moving the dog, then repeat while walking the dog slowly.
This distinguishes basic detection from the harder moving-viewpoint case.

Keep an eye on `/livox/lidar` during a field run, not only at startup. One
existing outdoor bag contains IMU for 29 minutes but LiDAR for only its first
186 seconds; a periodic rate check would have caught that live.

### 4. Run a repeatable benchmark

Leave the hardware profile running, allow 60 seconds for FastLIO and thermal
state to settle, then exercise a representative route during a five-minute
capture:

```bash
python3 benchmark_live.py --duration 300 --output benchmark-dog1-moving
```

The collector uses only the Python standard library. It writes:

- `summary.json`: perception frame count, mean/p50/p95/p99/max processing
  time, real-time budget use, and warning count;
- `topic-hz-*.txt`: raw input and FastLIO output rates;
- `docker-stats.csv`: per-container CPU and memory once per second;
- `tegrastats.txt`: Jetson clocks, temperatures, RAM, and power telemetry
  when the host provides `tegrastats`;
- `perception.log`, platform/version files, and the exact container state.

Use separate directories for at least three runs: stationary empty scene,
stationary with a walker, and dog moving with a walker. Run the same route and
duration when comparing settings. Five minutes is long enough to expose heat
or clock throttling that a quick indoor smoke test can miss.

For reproducible maximum-performance numbers, record the current power mode
with `sudo nvpmodel -q`; optionally select the lab-approved maximum-power mode
and run `sudo jetson_clocks` before all comparison runs. These commands alter
power/thermal behaviour, so do not mix boosted and default-mode results or use
them without considering battery and cooling. The benchmark records the mode
and tegrastats output so the result remains interpretable.

A practical pass criterion is:

- raw LiDAR, `/cloud_registered`, and `/Odometry` remain close to 10 Hz with
  no long gaps;
- the benchmark contains the expected number of perception frames (roughly
  one per second with the default `accumulate_scans=10`);
- p95 processing time stays comfortably below its roughly one-second frame
  budget (under 50% is the node's built-in warning boundary; under 20% leaves
  much healthier margin for load spikes);
- memory does not trend upward throughout the run, and tegrastats shows no
  sustained thermal throttling;
- the moving run produces plausible, persistent tracks rather than a flood of
  one-frame IDs or positions tens of metres from the robot.

### 5. Stop cleanly

```bash
docker compose --profile hardware down
```

`restart: unless-stopped` is useful after a process crash, but also means the
stack can return after a Jetson reboot. Use `down` when the experiment is over
rather than only killing individual processes.

### Live troubleshooting by symptom

| Symptom | Most likely cause | Check/fix |
|---|---|---|
| Driver logs `bind failed` | `192.168.1.50/24` missing from `eth0` | Fix netplan; host networking cannot compensate for a missing host address |
| Driver runs but `/livox/lidar` is silent | Wrong physical LiDAR IP, rear cable/power, or timestamp/config problem | Ping the `.env` address; inspect `docker logs go2-lidar-driver`; confirm the rendered-IP line |
| Raw topics publish but FastLIO topics do not | LiDAR/IMU timestamps or FastLIO startup | Confirm both raw rates, restart the whole profile, and check FastLIO for sync warnings; the image already applies the known timestamp patch |
| FastLIO publishes but perception has no frames | DDS discovery or perception crash | `docker compose ... ps`, then `docker logs go2-online-perception`; all services must use host networking |
| Frames process but no tracks appear | Static scene, insufficient points, or live density differs from replay data | First cross the view at walking pace; inspect moved/cluster counts before tuning parameters |
| Topic rate degrades after minutes | LiDAR link/driver dropout or Jetson throttling | Compare topic-hz output with `tegrastats.txt`; check cable/power and temperatures |
| `benchmark_live.py` reports no frames | Pipeline was not live during the timed window | Read `perception.log` and the four topic-hz files; fix the lowest silent topic first |

Do not tune detection thresholds until the raw and FastLIO rates are stable.
If tuning is needed, change one ROS parameter at a time and keep a benchmark
directory for the baseline and each variant; otherwise hardware, route, and
algorithm changes become impossible to separate.

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

Whether this container can run *live*, right now, on a Jetson that hasn't
been reflashed yet (still JetPack 5.1.x/Ubuntu 20.04/Foxy) rather than
waiting for the JetPack 6 reflash is a separate, related question with its
own reasoning and open items -- see the repo's internal notes' "Can the
Humble container actually run live on a Jetson without reflashing?"
section (short answer: architecturally yes, this is exactly why it's a
container and not a patch script, but three concrete things -- Docker
actually present on the Jetson, an aarch64 build, and the segfault
question re-tested from inside a container -- are still unconfirmed
either way).
