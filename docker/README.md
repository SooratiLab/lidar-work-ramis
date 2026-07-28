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
docker-compose.yml                sensor, perception, response, actuation, record, replay, export, shared services
.env.example                      copy to .env, set hardware, bag, and experiment values
config/
  MID360_config.json.template     driver config with ${LIVOX_LIDAR_IP} etc.
  cyclonedds_eth0.xml              CycloneDDS wired-interface restriction
  fastrtps_eth0_only.xml           Fast DDS fallback multicast whitelist
```

`docker-compose.yml` has five profiles; the optional `shared` profile is
layered onto `hardware`:

- **`hardware`** -- `driver` + `fastlio` + `perception` + `response` +
  `actuation`, for a real Mid-360 plugged into this Jetson. Actuation is a
  dry run unless explicitly enabled in `.env`.
- **`replay`** -- `fastlio` + `bag` plus the same perception/response/dry-run
  actuation path, for testing without a real sensor by replaying one of
  Kei's recorded bags. See `../perception/README.md`.
- **`record`** -- a raw `/livox/lidar` + `/livox/imu` rosbag recorder to run
  alongside `hardware`. Set `RECORD_OUTPUT_DIR` and give each capture a
  unique `RECORD_NAME`; see `../FIELD_TEST_GUIDE.md`.
- **`export`** -- `fastlio` + `bag` + `export`, for turning a bag into the
  `pcd/` + `poses.csv` layout `../evaluation/` expects, without a live
  ROS graph to watch or a separate node to run. See `../export/README.md`.
- **`shared`** -- the opt-in authenticated UDP track bridge and local
  two-source fusion node. Run it alongside `hardware` after configuring
  both dogs; it does not start a sensor or perception by itself. See
  `../merge/README.md`.

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

This starts the LiDAR driver, FastLIO, perception, response, and the
disabled-by-default actuation adapter. `docker-compose.yml` uses
`network_mode: host` -- the Livox SDK binds directly to the host's
LiDAR-subnet address rather than going through Docker's usual NAT'd bridge
network, so bridge networking fails outright with `bind failed` (confirmed
while testing this image without a real LiDAR-subnet interface present).
Host networking isn't a convenience here, it's required.

The actuation adapter is a dry run while `.env` contains
`ACTUATION_ENABLED=false` (the shipped default). It logs
`would_send_stop_move` but cannot publish `/api/sport/request`. Do not change
this setting until the stationary checklist below passes.

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

### Before going to the lab

Bring the laptop and its charger, a charged Go2 battery, the Go2 charger,
Ethernet cable and any USB-to-Ethernet adapter the laptop needs. Keep the
controller beside the operator and know how to stop or damp the robot before
asking anyone to walk through the sensor view. The software below only reads
LiDAR and odometry; it does not send motion commands, but the normal robot
safety procedure still applies.

The Go2 has one Ethernet path serving two different jobs at different times:

1. **Initial access:** laptop Ethernet is connected to the Go2 so the laptop
   can SSH directly to the Jetson at `192.168.123.18`.
2. **Live LiDAR:** that laptop cable is removed and the Mid-360 is connected
   to the Go2 Ethernet port, with power from the **rear** XT30 port. From this
   point, SSH must use WiFi, Tailscale, or the `go2-field` access point.

Do not expect to keep the direct laptop SSH link while the LiDAR occupies the
port. Before swapping the cable, prove that one of the wireless SSH routes
works. Direct Ethernet also does not give the Jetson internet access; GitHub
fetches require working WiFi (or another separately configured internet
route).

### 1. Connect the laptop directly and SSH to the Jetson

Power on one dog and wait for the Jetson to boot. Connect the laptop's
Ethernet adapter to the Go2 Ethernet port. Set **only that laptop adapter** to
the following manual IPv4 settings:

- address: `192.168.123.99`
- prefix/subnet mask: `/24` or `255.255.255.0`
- gateway: blank
- DNS: blank

On Windows this is **Settings -> Network & internet -> Ethernet -> IP
assignment -> Edit -> Manual -> IPv4**. On Ubuntu with NetworkManager, first
find the wired connection name with `nmcli connection show`, then use:

```bash
sudo nmcli connection modify "<wired-connection-name>" \
  ipv4.method manual ipv4.addresses 192.168.123.99/24 \
  ipv4.gateway "" ipv4.dns "" ipv4.never-default yes
sudo nmcli connection up "<wired-connection-name>"
```

Leaving the gateway blank is deliberate: it prevents the laptop from trying
to send internet traffic into the robot network. Verify the link from a
laptop terminal:

```bash
ping -c 3 192.168.123.18                 # Linux/macOS
# Windows PowerShell: ping 192.168.123.18
ssh unitree@192.168.123.18
```

Both dogs use `192.168.123.18`, so connect only one at a time. On the first
SSH connection, check the displayed fingerprint with the lab's existing
record before accepting it; a changed fingerprint after a reflash can be
legitimate, but should not be accepted blindly. Obtain the current password
from the lab owner -- credentials are intentionally not stored in this repo.
If an old reflash key causes `REMOTE HOST IDENTIFICATION HAS CHANGED`, remove
only this host's stale key after confirming the reflash:

```bash
ssh-keygen -R 192.168.123.18
```

If ping fails, do not change ROS or Docker. Check that the dog has finished
booting, the Ethernet adapter shows link, the laptop address really is
`192.168.123.99/24`, VPN software is not capturing the route, and no second
dog with the same address is connected.

### 2. Establish the wireless SSH and internet path

While still connected over direct Ethernet, inspect rather than guess which
wireless connection is available:

```bash
ip -br link
ip -br addr
nmcli device status
ip route
ping -c 3 8.8.8.8
getent hosts github.com
```

The direct cable proves local access; the final two commands prove that the
Jetson can actually fetch GitHub code. If internet works, note a wireless SSH
address. Depending on the lab setup this is the Jetson's Tailscale IP, or
`192.168.50.10` for Dog 1 / `192.168.50.11` for Dog 2 on the `go2-field`
access point. The latter provides field SSH but normally **not** internet.

From a second laptop terminal, test the wireless route while the direct SSH
session remains open:

```bash
ssh unitree@<wireless-or-tailscale-ip>
```

Only continue when that succeeds. If the Jetson was reflashed and WiFi or
Tailscale is not configured, stop here and complete the networking setup in
the legacy
[Go2 Jetson setup runbook](https://github.com/SooratiLab/ros2-go2/blob/main/go2-jetson-setup.md);
otherwise swapping in the LiDAR cable will lock the laptop out.

### 3. Fetch the GitHub code and pin what will be tested

The repository is public, so HTTPS is the simplest read-only deployment path
and does not require putting a GitHub SSH key on the robot. Run these commands
on the **Jetson**, not on the laptop:

```bash
cd ~
git clone https://github.com/SooratiLab/lidar-work-ramis.git
cd ~/lidar-work-ramis
git switch main
git pull --ff-only origin main
git status --short
git rev-parse HEAD
```

For an existing checkout, skip `clone` and run the final five commands. A
successful preflight has no output from `git status --short`; if it lists
files, preserve and review those local changes instead of overwriting them.
`--ff-only` refuses an ambiguous merge, which is safer on a deployment copy.
Save the printed commit hash in the experiment notes. Fetching `main` makes
the run repeatable only when that hash is recorded.

If the repository is private by test day, use a lab-approved deploy key or
personal access method; do not paste a token into a clone URL or shell
history. If `git clone` cannot resolve or reach GitHub, fix the Jetson's WiFi,
DNS, and default route before touching the code.

### 4. Preflight the host and robot network

Keep the robot supported or standing still for the first test, connect the
Mid-360 data lead to the Go2 Ethernet port, power it from the rear XT30 port,
and reconnect to the Jetson using the wireless SSH route already tested.
Run the deployment inside `tmux` so losing SSH does not stop it.

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
netplan using sections 4 and 7 of the
[Go2 Jetson setup runbook](https://github.com/SooratiLab/ros2-go2/blob/main/go2-jetson-setup.md)
before debugging Docker. A failed LiDAR ping is likewise a
cable/address/network problem, not a perception problem.

Also confirm the system clock is credible with `date --iso-8601=seconds`;
incorrect clocks make logs and comparisons hard to reconcile. The image does
not use CUDA, so `nvidia-container-runtime` is not required for this test.
Docker Engine plus the Compose plugin is sufficient.

### 5. Configure and build once on the Jetson

Configure the physical sensor. Do not copy another dog's `.env` without
checking the address.

```bash
cd ~/lidar-work-ramis/docker
cp .env.example .env
nano .env                       # set LIVOX_LIDAR_IP; host IP is normally unchanged

# Build can be slow on the Jetson. Keep it in tmux and do not start a field
# session until this has completed successfully.
docker compose --profile hardware build
```

Before starting containers, make Compose show the resolved configuration and
check that the selected LiDAR address is exactly the intended physical unit:

```bash
docker compose --profile hardware config
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

For the default-off per-scan occlusion A/B test, keep
`ACTUATION_ENABLED=false` and change only these `.env` values:

```text
USE_OCCLUSION_ACCUMULATION=true
PERCEPTION_ACCUMULATE_SCANS=1
PERCEPTION_ACCUMULATE_STRIDE=1
PERCEPTION_CLUSTER_MIN_POINTS=5
```

The normal values are `false`, `10`, `0`, and `10`. Restore them before the
baseline run. The node rejects occlusion accumulation with a multi-scan
window, because offline testing confirmed that merging several sensor poses
violates the detector's assumptions. Record both runs and compare tracks;
do not enable actuation while screening an experimental detector.

### 6. Start the live stack and verify it from the bottom up

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
docker logs --since 2m go2-cluster-response
docker logs --since 2m go2-stop-actuation
```

Success means the sensor/FastLIO topics publish continuously and perception
logs lines
of the form `frame ... processed in ...ms`. Confirmed tracks appear as
`track N [new|matched|coasting|reidentified]`. The response log should move
through `clear`, `pending_stop`, and `stop` as a person crosses the 2 m
boundary, and the actuation log should say `would_send_stop_move`, never
`sent_stop_move`, during this first check. No tracks in an empty, static
scene is healthy; frame processing is the liveness check. Walk a person across
the LiDAR view before moving the dog, then repeat while walking the dog slowly.
This distinguishes basic detection from the harder moving-viewpoint case.

Keep an eye on `/livox/lidar` during a field run, not only at startup. One
existing outdoor bag contains IMU for 29 minutes but LiDAR for only its first
186 seconds; a periodic rate check would have caught that live.

### 7. Run a repeatable benchmark

Leave the hardware profile running, allow 60 seconds for FastLIO and thermal
state to settle, then exercise a representative route during a five-minute
capture:

```bash
python3 benchmark_live.py --profile hardware --duration 300 \
  --output benchmark-dog1-moving
```

The collector uses only the Python standard library. It writes:

- `summary.json`: perception frame count, mean/p50/p95/p99/max processing
  time, real-time budget use, moved-point/cluster/confirmed-track counts,
  and warning count;
- `topic-hz-*.txt`: raw input and FastLIO output rates;
- `docker-stats.csv`: per-container CPU and memory once per second;
- `tegrastats.txt`: Jetson clocks, temperatures, RAM, and power telemetry
  when the host provides `tegrastats`;
- `perception.log`, `response.log`, and `actuation.log`;
- response transition counts, stale-state entries, and dry-run/sent actuation
  counts in `summary.json`;
- platform/version files and the exact container state.

The same collector can compare detector configurations against an identical
bag. Restart the replay profile from the beginning for each configuration,
then use `--profile replay`; the field guide contains the recommended matrix.

Use separate directories for at least three runs: stationary empty scene,
stationary with a walker, and dog moving with a walker. Run the same route and
duration when comparing settings. Five minutes is long enough to expose heat
or clock throttling that a quick indoor smoke test can miss.

Use names that cannot be confused later, for example
`benchmark-20260721-dog1-stationary-empty`,
`benchmark-20260721-dog1-stationary-walker`, and
`benchmark-20260721-dog1-moving-walker`. The output directory must not already
exist. During the first two runs leave the dog stationary; for the moving run,
use the controller normally and have a second person walk the agreed route.

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
- the stationary-walker run produces the expected stop/clear transition and
  dry-run StopMove action, with no stale-input entries during the healthy
  portion of the run.

### Enabling StopMove after the stationary dry run

Only after the topic rates, response threshold, and dry-run action have been
observed together on a stationary dog:

```bash
# Stop the profile before editing the safety boundary.
docker compose --profile hardware down
sed -i 's/^ACTUATION_ENABLED=false$/ACTUATION_ENABLED=true/' .env
docker compose --profile hardware up
```

On startup, verify `go2-stop-actuation` logs `enabled=True`. A stop request
publishes Unitree API ID 1003 (`StopMove`) and repeats it at most once per
second while the request remains active. Clearing the request deliberately
does not resume walking; recovery is manual. Keep the controller in hand and
test StopMove with the dog already standing before attempting commanded
motion. Return `ACTUATION_ENABLED=false` immediately after the test.

### 8. Stop cleanly and copy the evidence off the dog

```bash
docker compose --profile hardware down
```

Confirm `docker compose --profile hardware ps` shows no remaining project
containers. Copy benchmark directories to the laptop before leaving the lab;
run this on the laptop while a working SSH route is available:

```bash
scp -r unitree@<jetson-ip>:~/lidar-work-ramis/docker/benchmark-20260721-dog1-* .
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
| Response stays in `waiting_*` or `stale_*` | Tracks or odometry are absent/older than 2.5 s | Check both topic rates and response logs; stale input requests a stop by default |
| Dry-run adapter logs nothing | Stop threshold did not persist for 1 s, or response DDS path is broken | Echo `/online_perception/stop_requested`; keep actuation disabled while diagnosing |
| Enabled adapter exits immediately | Image predates the pinned `unitree_api` package | Rebuild the current Dockerfile and confirm `ros2 interface show unitree_api/msg/Request` |
| Topic rate degrades after minutes | LiDAR link/driver dropout or Jetson throttling | Compare topic-hz output with `tegrastats.txt`; check cable/power and temperatures |
| `benchmark_live.py` reports no frames | Pipeline was not live during the timed window | Read `perception.log` and the four topic-hz files; fix the lowest silent topic first |

Do not tune detection thresholds until the raw and FastLIO rates are stable.
If tuning is needed, change one ROS parameter at a time and keep a benchmark
directory for the baseline and each variant; otherwise hardware, route, and
algorithm changes become impossible to separate.

### First-session go/no-go checklist

Do not begin the moving benchmark until every earlier gate is true:

- [ ] Only one dog is on the direct `192.168.123.18` Ethernet path.
- [ ] Direct SSH worked, then wireless/Tailscale SSH worked before the cable swap.
- [ ] The Jetson can resolve and reach GitHub, and the tested commit hash is recorded.
- [ ] `eth0` has both expected addresses and no default route.
- [ ] The physical LiDAR IP matches `.env` and responds to ping.
- [ ] Compose built successfully and all five hardware containers are running.
- [ ] `.env` still has `ACTUATION_ENABLED=false`.
- [ ] A threshold crossing produces `would_send_stop_move` in dry-run mode.
- [ ] Stopping track or odometry input produces a fail-safe stop request.
- [ ] The operator has verified manual recovery and has the controller ready
      before the separately-approved enabled StopMove test.
- [ ] Raw LiDAR and IMU publish; FastLIO cloud and odometry publish continuously.
- [ ] Perception logs processed frames in a stationary test.
- [ ] The controller, operator, clear test area, battery, disk space, and cooling are ready.
- [ ] A short stationary-with-walker run produces plausible output before the dog moves.

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
- CycloneDDS has been verified locally with that replay profile against
  `dog1/2026-05-12_16_21_soton_indoor`, using the development computer's
  active interface override. Cross-container rates were ~10 Hz for
  `/livox/lidar`, `/cloud_registered`, and `/Odometry`; the perception node
  processed all 24 accumulated frames, reproduced the two persistent tracks,
  and averaged 10.5 ms per processed frame. This verifies the Humble replay
  path, not the Go2 Jetson or native Unitree DDS topics.

Not yet verified: an actual Mid-360 talking to this container, an actual
Jetson build, and whether CycloneDDS's node-creation segfault (which forced
Fast DDS on the Foxy setup this replaces) still happens on Humble. CycloneDDS
is now the default live-test path; Fast DDS remains an environment-variable
fallback because it is already known to work.

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
CycloneDDS is the default. Rebuild after pulling this change so the image
contains Humble's CycloneDDS RMW implementation:

```bash
docker compose build
docker compose --profile hardware up -d
```

Confirm every service received the same RMW selection:

```bash
docker compose --profile hardware exec driver printenv RMW_IMPLEMENTATION
docker compose --profile hardware exec fastlio printenv RMW_IMPLEMENTATION
docker compose --profile hardware exec perception printenv RMW_IMPLEMENTATION
```

All three should print `rmw_cyclonedds_cpp`. Then perform the topic-rate and
container-log checks below. A Foxy-era node-creation failure would appear
immediately in `docker compose --profile hardware logs`.

To fall back without editing any files, recreate the stack with Fast DDS:

```bash
docker compose --profile hardware down
RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  docker compose --profile hardware up -d --force-recreate
```

The existing Fast DDS Ethernet whitelist remains in the image and is selected
automatically by `FASTRTPS_DEFAULT_PROFILES_FILE`. Use the same environment
prefix on later `docker compose` commands for that session, or put
`RMW_IMPLEMENTATION=rmw_fastrtps_cpp` in `.env` until the CycloneDDS issue is
understood.

The CycloneDDS file intentionally names the Go2 Jetson's `eth0`. For replay on
a development computer whose active interface has another name, override the
URI for the whole Compose invocation:

```bash
CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="YOUR_INTERFACE" priority="default" multicast="default"/></Interfaces></General></Domain></CycloneDDS>' \
  BAG_PATH=/absolute/path/to/bag \
  docker compose --profile replay up
```

Use `ip -br addr` to find `YOUR_INTERFACE`. This override is only for local
replay; leave it unset on the Go2 so DDS remains restricted to `eth0`.
