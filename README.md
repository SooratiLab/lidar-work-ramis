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
sensor. -- see each subdirectory's README for exactly what has and
hasn't been tested.

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
  Tracking itself has since moved beyond a direct port: a constant-velocity
  Kalman filter per track, globally-optimal (Hungarian) frame-to-frame
  assignment, coasting through brief gaps, a minimum-hits confirmation step
  to filter single-frame noise, and an odometry-based plausibility gate.
  Validated against a replayed bag standing in for a live sensor --
  reproduces Kei's own documented "2 tracks detected" result on the
  `soton_indoor` session, and was reviewed against several more recorded
  sessions (stationary, walking, walking-with-stops, degraded pre-fix data)
  to build confidence before a Jetson port. That review found processing
  time has a large margin on this hardware, but also found and quantified
  a real open risk: a moving sensor produces substantially more false
  positives than a stationary one, for reasons tracking-side fixes alone
  could narrow but not fully solve. An odometry-referenced visibility gate
  (`range_image.py`) now addresses this directly -- rerunning the same
  recorded sessions with it enabled cuts false tracks by 45-93% depending
  on session (207 -> 15 on the worst one) while both real tracks in the
  `soton_indoor` baseline still survive. See `perception/README.md` for the
  full writeup, including what's confirmed vs still open -- most notably,
  this is validated against recorded bags standing in for a live sensor,
  not an actual moving dog yet.
- **Multi-dog map/track merging** -- not started yet, still at the
  planning stage.
