#!/usr/bin/env python3
"""
export_fastlio.py -- export FastLIO's registered clouds + odometry to PCD
files and a poses CSV.

Ported from kei-stuff/ros2-go2/scripts/export_fastlio.py. rclpy's Node/
subscription API hasn't changed between Foxy, Humble, and Jazzy, so the
port itself needed no ROS-version-specific changes -- what changed is how
it's run: Kei's version was a manual step in a three-terminal WSL2/Jazzy
workflow (see kei-stuff/ros2-go2/laptop-wsl-setup.md); this one runs as a
docker-compose service (../docker/docker-compose.yml's `export` service)
alongside the same containerised Humble FastLIO this repo's `docker/`
already builds, so exporting a new bag no longer needs a second OS/ROS
distro on a laptop. See ../docker/README.md for the one-command version and
"Usage" below for running it directly against any ROS graph.

Also rewrites the PointCloud2 parse as a vectorised numpy operation
(structured dtype + np.frombuffer) instead of Kei's per-point
struct.unpack loop, matching the approach perception/pointcloud.py already
uses for the same message type -- this callback runs once per incoming
scan (10 Hz, thousands of points each), not once per recording, so the
per-point Python loop was a real cost worth removing while porting this
anyway, not just a style preference.

Subscribes to:
    /cloud_registered   (sensor_msgs/PointCloud2)  -- registered point cloud per scan
    /Odometry           (nav_msgs/Odometry)         -- 6-DOF pose per scan

Outputs:
    <output_dir>/pcd/frame_NNNNNN.pcd   -- one PCD per accumulated frame (coords in mm)
    <output_dir>/poses.csv              -- frame, timestamp, x, y, z, qx, qy, qz, qw (metres)

Usage (recommended -- one docker-compose command, no WSL2/second ROS distro):

    cd ../docker
    cp .env.example .env   # set BAG_PATH to the bag to export, EXPORT_OUTPUT_DIR to where
    docker compose --profile export up
    # watch the `bag` service's log for playback finishing, then Ctrl+C --
    # SIGTERM triggers the same flush-and-save-poses path as Ctrl+C would
    # (see main() below), so poses.csv gets written either way.

Usage (manual -- against any already-running ROS graph, e.g. a live sensor
once one is reachable, or FastLIO running outside Docker):

    ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml rviz:=false   # T1
    python3 export_fastlio.py data/2026-07-16_walk --accumulate 10                # T2
    ros2 bag play <bag_path>                                                      # T3 (if replaying)
    # Ctrl+C the exporter once the bag/session is done.

Notes:
    - ROS uses metres; the lidar-perception pipeline expects millimetres
      (see kei-stuff/lidar-perception/README.md's "Important: coordinate
      units"). Point clouds are scaled x1000 before writing. Poses stay in
      metres -- they encode the robot trajectory, not point coordinates,
      and evaluation/ and perception/ both expect FastLIO's own metre
      convention there. This mm/m split is the exact gotcha AGENTS.md
      flags as having caused real confusion once already -- don't
      "simplify" it to one unit without updating every downstream reader
      of these files' units at the same time.
    - Each /cloud_registered message becomes one accumulated frame once
      --accumulate scans have arrived. At ~10 Hz, --accumulate 10 gives
      ~1-second frames, matching evaluation/'s and perception/'s own
      accumulate_scans default and the tuned detection parameters
      documented throughout this repo (they were tuned against 1-second
      frames, not against whatever a different --accumulate would produce).
"""

import argparse
import csv
import os

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry


# sensor_msgs/msg/PointField datatype constants -> numpy dtype characters.
# Same mapping as perception/pointcloud.py's _POINTFIELD_TO_NUMPY -- not
# imported from there since this script runs standalone (its own
# container/venv, no perception/ on its path) and it's a five-line dict,
# not worth a cross-directory import for. See evaluation/pcd_io.py's
# docstring for the same reasoning applied to a different pair of modules.
_POINTFIELD_TO_NUMPY = {
    1: "i1", 2: "u1",
    3: "i2", 4: "u2",
    5: "i4", 6: "u4",
    7: "f4", 8: "f8",
}


def read_pointcloud2_xyzi(msg: PointCloud2) -> np.ndarray:
    """
    Parse a PointCloud2 into an (N, 4) float32 array [x, y, z, intensity].

    Vectorised (structured dtype + np.frombuffer) rather than a per-point
    loop -- see the module docstring for why that matters here. Builds the
    dtype from the message's own field layout rather than assuming x/y/z/
    intensity are packed at fixed offsets, so it still works if a source
    reorders fields or omits intensity (falls back to 0.0 in that case,
    matching Kei's original behaviour).
    """
    names, formats, offsets = [], [], []
    for field in msg.fields:
        if field.name not in ("x", "y", "z", "intensity"):
            continue
        names.append(field.name)
        formats.append(_POINTFIELD_TO_NUMPY[field.datatype])
        offsets.append(field.offset)

    dtype = np.dtype({
        "names": names, "formats": formats,
        "offsets": offsets, "itemsize": msg.point_step,
    })

    count = msg.width * msg.height
    structured = np.frombuffer(msg.data, dtype=dtype, count=count)

    points = np.empty((count, 4), dtype=np.float32)
    points[:, 0] = structured["x"]
    points[:, 1] = structured["y"]
    points[:, 2] = structured["z"]
    points[:, 3] = structured["intensity"] if "intensity" in names else 0.0
    return points


def write_pcd_binary(path: str, points: np.ndarray):
    """Write an (N, 4) float32 array as a binary PCD (x y z intensity in mm)."""
    n = len(points)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    )
    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(points.astype(np.float32).tobytes())


class FastLIOExporter(Node):
    def __init__(self, output_dir: str, accumulate: int):
        super().__init__('fastlio_exporter')
        self.output_dir = output_dir
        self.pcd_dir = os.path.join(output_dir, 'pcd')
        os.makedirs(self.pcd_dir, exist_ok=True)

        self.accumulate = accumulate
        self.accum_buffer = []
        self.scan_count = 0       # raw scans received
        self.frame_count = 0      # accumulated frames written
        self.poses = []
        self.latest_odom = None

        self.cloud_sub = self.create_subscription(
            PointCloud2, '/cloud_registered', self.cloud_cb, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/Odometry', self.odom_cb, 10)

        self.get_logger().info(
            f'Exporter ready -- saving to {output_dir}  '
            f'(accumulate={accumulate} scans per frame)')

    def odom_cb(self, msg: Odometry):
        self.latest_odom = msg

    def cloud_cb(self, msg: PointCloud2):
        points = read_pointcloud2_xyzi(msg)
        if len(points) == 0:
            return

        self.accum_buffer.append(points)
        self.scan_count += 1

        # Record the pose at each scan (before accumulation decision) --
        # one pose per raw scan, not one per accumulated frame, so
        # downstream consumers can see the trajectory at full resolution
        # even though the point cloud itself is coarser.
        if self.latest_odom:
            p = self.latest_odom.pose.pose.position
            q = self.latest_odom.pose.pose.orientation
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.poses.append([
                self.frame_count, t,
                p.x, p.y, p.z,
                q.x, q.y, q.z, q.w
            ])

        if len(self.accum_buffer) >= self.accumulate:
            merged = np.concatenate(self.accum_buffer, axis=0)
            self.accum_buffer = []

            # Convert metres -> millimetres (xyz only, not intensity).
            merged[:, :3] *= 1000.0

            filename = os.path.join(
                self.pcd_dir, f'frame_{self.frame_count:06d}.pcd')
            write_pcd_binary(filename, merged)
            self.frame_count += 1

            if self.frame_count % 10 == 0:
                self.get_logger().info(
                    f'Wrote {self.frame_count} frames '
                    f'({self.scan_count} scans, '
                    f'~{merged.shape[0]} pts last frame)')

    def save_poses(self):
        path = os.path.join(self.output_dir, 'poses.csv')
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow([
                'frame', 'timestamp',
                'x', 'y', 'z',
                'qx', 'qy', 'qz', 'qw'
            ])
            w.writerows(self.poses)
        self.get_logger().info(f'Poses CSV written to {path}')

    def flush(self):
        """Write any remaining scans in the accumulation buffer."""
        if self.accum_buffer:
            merged = np.concatenate(self.accum_buffer, axis=0)
            self.accum_buffer = []
            merged[:, :3] *= 1000.0
            filename = os.path.join(
                self.pcd_dir, f'frame_{self.frame_count:06d}.pcd')
            write_pcd_binary(filename, merged)
            self.frame_count += 1


def main():
    parser = argparse.ArgumentParser(
        description='Export FastLIO /cloud_registered to PCD files + poses CSV')
    parser.add_argument(
        'output_dir', type=str,
        help='Directory to write pcd/ subfolder and poses.csv into')
    parser.add_argument(
        '--accumulate', type=int, default=10,
        help='Number of scans to merge into each output frame '
             '(default 10 = ~1-second frames at 10 Hz, matching the tuned '
             'detection parameters used throughout this repo; use 1 for '
             'one PCD per raw scan instead)')
    args = parser.parse_args()

    rclpy.init()

    # rclpy installs its own SIGINT/SIGTERM handling as part of
    # rclpy.init()'s default signal_handler_options=ALL -- confirmed by
    # testing that overriding it with a plain Python signal.signal() call
    # (to route SIGTERM through the same path as Ctrl+C's SIGINT) instead
    # silently breaks shutdown entirely: rclpy's own handler is what wakes
    # a blocked spin() via an internal guard condition, and replacing it
    # with a Python-level handler removes that wake-up without providing
    # another one, so spin() then blocks forever on SIGTERM instead of
    # returning. rclpy's actual shutdown path also isn't KeyboardInterrupt
    # -- a signal-triggered shutdown makes spin() raise
    # ExternalShutdownException instead, which is what's actually caught
    # below (KeyboardInterrupt is kept too, for a bare Ctrl+C when this is
    # run directly against a terminal ROS graph rather than under
    # docker-compose). Either way, docker-compose's SIGTERM on `stop`/
    # `down` and a local Ctrl+C's SIGINT now both reach here and take the
    # same flush-and-save-poses path.
    node = FastLIOExporter(args.output_dir, args.accumulate)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.flush()
    node.save_poses()
    node.get_logger().info(
        f'Done -- {node.frame_count} frames from {node.scan_count} scans')
    node.destroy_node()
    # try_shutdown, not shutdown -- a signal-triggered
    # ExternalShutdownException means the context is already shut down by
    # the time control gets here, and plain shutdown() raises on a
    # context that's already down instead of treating it as a no-op.
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
