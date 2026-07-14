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
sensor. That's a real constraint on what "verified" means here, not a
minor caveat -- see each subdirectory's README for exactly what has and
hasn't been tested.

## What's here

- **`docker/`** -- ROS 2 Humble container for `livox_ros_driver2` +
  FastLIO. Builds and runs clean, with fewer patches than the inherited
  Foxy setup needed. Verified end to end against one of Kei's recorded
  bags: FastLIO correctly fuses real recorded LiDAR/IMU data and produces
  ~10 Hz odometry, matching Kei's own documented result on that session.
  See `docker/README.md` for setup and the full list of what's confirmed
  vs still open.
- **`perception/`** -- a live ROS 2 node that reproduces Kei's offline
  `track_motion.py` pipeline (frame-to-frame background subtraction ->
  DBSCAN clustering -> centroid tracking) as a direct subscriber to
  FastLIO's output, instead of a batch script over exported PCD files.
  Validated against a replayed bag standing in for a live sensor --
  reproduces Kei's own documented "2 tracks detected" result on the
  `soton_indoor` session using the same tuned parameters, unchanged. See
  `perception/README.md`.
- **Multi-dog map/track merging** -- not started yet, still at the
  planning stage.

## Working notes

Day-to-day working notes, decisions, and the task list live locally as
`DOCS.md`/`TODO.md`/`AGENTS.md` (gitignored, not shown on GitHub) -- ask
Ramis for the current working copy if picking this up without it.
