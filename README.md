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
- **Multi-dog map/track merging** -- not started yet, still at the
  planning stage.
