# lidar-work-ramis

LiDAR work by Ramis Bhatty (rb3g23), continuing from Kei Kheng Tan's
[lidar-perception](https://github.com/SooratiLab/lidar-perception.git) and
[ros2-go2](https://github.com/SooratiLab/ros2-go2.git) repos. Part of a 10
week Summer Internship at the University of Southampton, supervised by
Dr. Mohammad Soorati.

## Background

Kei's pipeline records LiDAR data in the field with two Unitree Go2 EDU
robots (each with a Livox Mid-360), then processes it entirely offline
afterwards: ROS 2 Foxy on the Jetsons for recording, then a laptop replays
the bag through FastLIO and runs a Python/Open3D pipeline for detection,
tracking, and rendering. Full detail on that pipeline lives in the two
repos above.

This repo continues that work in two directions:

1. **Moving the ROS stack to ROS 2 Humble** with a reproducible Docker
   build, replacing the manual multi-repo build-and-patch process in
   `ros2-go2`'s setup runbook.
2. **Moving detection/tracking from an offline batch script to a live
   ROS 2 node**, so it can eventually run on the Jetson in the field
   instead of after a bag gets shipped to a laptop.

No Go2, Jetson, or Mid-360 has been reachable yet during this work --
everything below has been verified either on x86_64 as a stand-in for the
Jetson's aarch64, or against Kei's recorded bags standing in for a live
sensor. Both Jetsons are currently being reflashed to JetPack 6 (Ubuntu
22.04), which will put them on native ROS 2 Humble rather than the
Foxy+container fallback this repo's Docker work was hedging against -- see
each subdirectory's README for exactly what has and hasn't been tested.

## What's here

- **`docker/`** -- ROS 2 Humble container for `livox_ros_driver2` +
  FastLIO. Builds and runs clean, with fewer patches than the inherited
  Foxy setup needed. Verified end to end against one of Kei's recorded
  bags: FastLIO correctly fuses real recorded LiDAR/IMU data and produces
  ~10 Hz odometry, matching Kei's own documented result on that session.
  See `docker/README.md` for setup and the full list of what's confirmed
  vs still open.
- **`perception/`** -- a live ROS 2 node that ports Kei's offline
  `track_motion.py` pipeline (frame-to-frame background subtraction ->
  DBSCAN clustering -> centroid tracking) to a direct subscriber on
  FastLIO's output, instead of a batch script over exported PCD files.
  Tracking itself has since moved well beyond a direct port: a
  constant-velocity Kalman filter per track, globally-optimal (Hungarian)
  frame-to-frame assignment, coasting through brief gaps, a minimum-hits
  confirmation step to filter single-frame noise, an odometry-based
  plausibility gate, an odometry-referenced visibility gate that fixes the
  moving-sensor false-positive problem a stationary-only pipeline doesn't
  have to deal with, and re-identification that bridges genuine stops or
  occlusions beyond what coasting alone can trust. Validated against
  replayed bags standing in for a live sensor across stationary, walking,
  stop-and-go, degraded pre-fix, and (with real caveats found along the
  way) outdoor sessions -- reproduces Kei's own documented "2 tracks
  detected" result on `soton_indoor`, and the visibility gate cuts false
  tracks by 45-93% depending on session while both real tracks in that
  baseline still survive. An overlapping-frame architecture was
  investigated as a latency improvement and rejected with data, and a
  stage-by-stage performance profile found no library swap currently
  justified -- this is the current priority: a solid single-LiDAR
  implementation, tested and documented against recorded bags, before
  live hardware testing or multi-dog merging. See `perception/README.md`
  for the full writeup, including what's confirmed vs still open -- most
  notably, this is all validated against recorded bags standing in for a
  live sensor, not an actual moving dog yet.
- **`evaluation/`** -- compares the current tracking pipeline against
  Kei's offline `track_motion.py` output on the same recorded sessions,
  producing trajectory/speed/track-count plots and a frame-by-frame
  point-cloud gif per session. Reproduces the "2 tracks on `soton_indoor`"
  and false-positive-reduction numbers above as concrete artifacts instead
  of log excerpts. See `evaluation/README.md`.
- **`export/`** -- ports Kei's `export_fastlio.py` (originally a manual
  step in a three-terminal WSL2/Jazzy workflow, see
  `kei-stuff/ros2-go2/laptop-wsl-setup.md`) into a `docker-compose`
  service alongside this repo's own containerised Humble FastLIO, so
  turning a new bag into `evaluation/`'s expected `pcd/` + `poses.csv`
  layout no longer needs a second OS/ROS distro on a laptop. Verified end
  to end: replayed the same `soton_indoor` bag used throughout
  `perception/`'s testing through `docker compose --profile export up`,
  fed the result straight into `evaluation/compare_pipelines.py`, and got
  the same "2 confirmed tracks" result documented elsewhere in this
  README from data that had never touched WSL2 or Jazzy. See
  `export/README.md`.
- **`merge/`** -- fuses two dogs' independently-tracked objects into one
  shared-frame track log, instead of merging raw point clouds (which
  Kei's handover already found unreliable on real two-dog field data --
  see `merge/README.md` for why). Calibrates a supplied relative-pose
  prior against whichever tracks the two dogs actually agree on (a
  track-level analogue of ICP-with-prior), searches for a frame-index
  lag between the two dogs' independently-exported sessions, and fuses
  corresponded tracks while keeping single-dog-only tracks rather than
  dropping them. Validated against the one two-dog session currently
  exported (`2026-05-12_fallback_cardbox1`, the cardboard-box
  complementary-coverage demo from Kei's handover): recovers a plausible
  calibration from an identity prior and produces one continuous fused
  trajectory for the object both dogs actually saw, alongside several
  correctly-uncorresponded single-dog tracks. See `merge/README.md` for
  the full result and honest caveats -- one validated session is a real
  result, not a systematic one.

## Usage

Two different workflows, depending on whether there's a sensor/bag
involved at all:

### Offline: evaluating the tracking pipeline against recorded data

The fastest way to run or tune the tracking algorithm itself -- no Docker,
no ROS, no FastLIO build. `perception/tracking.py`, `pointcloud.py`, and
`range_image.py` have no `rclpy` dependency (see `perception/README.md`),
so `evaluation/` runs them directly against a session's already-exported
PCD frames + poses CSV.

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r evaluation/requirements.txt

# a session already lives under data/ -- see evaluation/README.md for
# which ones and where they came from
python3 evaluation/compare_pipelines.py data/2026-05-12_soton_indoor_dog1 \
    --kei-tracks ../kei-stuff/lidar-perception/output/2026-05-12_indoor/soton_indoor_dog1_tracks.csv
```

Writes a tracks CSV, a frame-by-frame gif, and (with `--kei-tracks`)
trajectory/speed/track-count comparison plots to `output/<session>/`. See
`evaluation/README.md` for the full option list and what each output
actually shows.

This only works against a session that's already been exported to PCD
frames + a poses CSV (`data/<session>/pcd/frame_*.pcd` + `poses.csv`) --
a handful of Kei's sessions already are, copied in from
`kei-stuff/lidar-perception/data/`. Turning a *new* raw bag into that
format no longer needs Kei's original WSL2/Jazzy detour -- see `export/`
above and its `README.md`:

```bash
cd docker
cp .env.example .env   # same LIVOX_LIDAR_IP/LIVOX_HOST_IP as any other profile
BAG_PATH=/path/to/bag EXPORT_OUTPUT_DIR=../data/<session> docker compose --profile export up
# Ctrl+C once the bag service's log shows playback finished
```

### Live (or bag-replay-standing-in-for-live): running the actual ROS node

`docker/` builds the one Docker image everything else runs in --
`livox_ros_driver2` + FastLIO on ROS 2 Humble -- and `docker-compose.yml`
wires it up two ways:

```bash
cd docker
cp .env.example .env   # set LIVOX_LIDAR_IP for whichever dog this is

# a real Mid-360 plugged into this machine
docker compose --profile hardware up

# no sensor available -- replay one of Kei's recorded bags through FastLIO
# instead (set BAG_PATH in .env to a directory with metadata.yaml + a .db3)
docker compose --profile replay up
```

Both profiles start `perception/online_perception_node.py` subscribed to
FastLIO's live output, publishing a `MarkerArray` to
`/online_perception/markers` (view in RViz2) and logging each frame's
detections. See `docker/README.md` for the full profile/service breakdown
and `perception/README.md` for what's actually been validated running
this way.

**Known issue, previously blocking, now confirmed resolved on at least
this machine**: this depends on ROS 2 DDS discovery actually working
between the `fastlio`/`bag`/`perception` containers (`network_mode: host`
should be enough, and this has worked on other machines) -- earlier
testing found it completely broken on this dev machine (a bare `rclpy`
publisher/subscriber pair failing to discover each other even within a
single container). Re-checked and actually re-ran `docker compose
--profile replay up` against `soton_indoor` end to end: the `perception`
container now logs live detections frame by frame, matching the same two
tracks and speeds `evaluation/`'s offline replay of the same session
reports (see `evaluation/README.md`). Neither this nor the earlier
"broken" finding controlled for what changed in between (machine state,
Docker version, network config are all plausible, none confirmed), so
this is a live re-test, not a root-cause explanation -- but the `replay`
profile with `perception` subscribed live is confirmed working *now*, on
this machine, not just "worth retrying." If `docker compose --profile
replay up` produces FastLIO output (`ros2 topic hz /cloud_registered`
from the host) but the `perception` container logs nothing on some other
machine, that's a sign this is machine/environment-dependent, not
necessarily a code issue -- see `evaluation/README.md`'s "Why this runs
offline, not over a live bag replay" section for the full history. No
LiDAR or Jetson has been reachable yet either way, so this whole path is
itself only validated against replayed bags -- see `docker/README.md`
and `perception/README.md` for exactly what's confirmed vs still open.
