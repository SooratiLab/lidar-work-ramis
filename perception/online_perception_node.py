#!/usr/bin/env python3
"""
online_perception_node.py -- live frame-to-frame motion detection and
centroid tracking on FastLIO's /cloud_registered + /Odometry, without going
through PCD files on disk first.

This is the streaming counterpart to
lidar-perception/scripts/track_motion.py in kei-stuff. The detection side
of that pipeline is unchanged here -- accumulate scans into fixed-duration
frames, diff consecutive frames, DBSCAN the moved points. The tracking side
has moved on from that script's greedy nearest-centroid matching: this node
uses a constant-velocity Kalman filter per track with globally-optimal
(Hungarian) frame-to-frame assignment and a few frames of coasting through
missed detections, instead of ending a track the instant one frame's
detection is missing. See tracking.py for the full reasoning and pointcloud.py
for the point cloud parsing/downsampling this feeds off.

Coordinate units: metres throughout, matching ROS conventions and FastLIO's
own /cloud_registered and /Odometry output directly. This deliberately does
NOT convert to millimetres the way the offline pipeline's PCD files do (see
export_fastlio.py in kei-stuff) -- there's no PCD file in this path to match
units with, so there's no reason to introduce that convention here. Every
distance parameter below (voxel size, thresholds, etc.) is in metres, every
Kalman noise parameter documents its own units.

Subscribes:
    /cloud_registered   (sensor_msgs/PointCloud2)
    /Odometry            (nav_msgs/Odometry)  -- currently just logged
                          alongside each processed frame for reference;
                          not yet used to compensate motion beyond what
                          FastLIO already bakes into /cloud_registered.

Publishes:
    /online_perception/markers   (visualization_msgs/MarkerArray) -- one
                                   sphere + one text label per active track,
                                   viewable in RViz2 against the same
                                   "camera_init" frame FastLIO publishes in.
                                   Coasting tracks (predicted position, no
                                   detection this frame) are drawn at
                                   reduced opacity so it's visually obvious
                                   when a track is being carried through a
                                   gap rather than freshly confirmed.

Parameters (all overridable via --ros-args -p <name>:=<value>):
    accumulate_scans     (int,   default 10)    scans merged per frame --
                                                 see lidar-perception/
                                                 README.md's --accumulate
                                                 for the offline equivalent
    voxel_size            (float, default 0.05)  downsample voxel size (m)
    change_threshold      (float, default 0.15)  moved-point distance (m)
    cluster_eps            (float, default 0.5)   DBSCAN radius (m)
    cluster_min_points    (int,   default 10)    DBSCAN min_samples
    max_match_distance    (float, default 1.5)   track-matching gate (m) --
                                                 max distance between a
                                                 track's predicted position
                                                 and a detection centroid
                                                 for them to be matched
    max_missed_frames    (int,   default 3)     frames a track keeps
                                                 coasting (predicted
                                                 position, no detection)
                                                 before it's dropped
    kalman_position_std   (float, default 0.1)   assumed centroid
                                                 measurement noise (m) --
                                                 how much a single DBSCAN
                                                 cluster's mean is trusted
                                                 as a position estimate
    kalman_velocity_std   (float, default 2.0)   initial velocity
                                                 uncertainty (m/s) for a
                                                 newly created track, before
                                                 it has any motion history
    kalman_process_std    (float, default 1.0)   assumed acceleration noise
                                                 (m/s^2) -- how much
                                                 unmodelled speeding up/
                                                 slowing down/turning the
                                                 constant-velocity model
                                                 should expect between
                                                 frames
    z_max                 (float, default 2.5)   ceiling crop height (m)

Defaults for the detection-side parameters match the FastLIO-tuned values in
lidar-perception/README.md's track_motion.py parameter table (converted
mm -> m), since that's the best available starting point for FastLIO's
sparse, moving-sensor point density -- not re-derived from scratch here.
They are a starting point, not a guarantee; re-tune against whatever this
node's actual frame density turns out to be once running against a live
sensor. The Kalman noise parameters are new and have no offline equivalent
to carry over -- their defaults are reasoned from walking-pace human motion
(see the module docstring above each one) and, like the detection
parameters, should be revisited once more sessions have been run.
"""
import colorsys
import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

# Local, non-ROS modules living alongside this script -- see pointcloud.py
# and tracking.py for what moved out of this file and why. Explicit
# sys.path insert (rather than relying on the script's directory already
# being first on sys.path) so this still works if the node is ever launched
# via `python3 -m` or an entry point instead of run as a plain script,
# matching the same defensive pattern kei-stuff/lidar-perception/scripts
# already uses for its own local imports.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from pointcloud import pointcloud2_to_xyz, voxel_downsample
from tracking import CentroidTracker, cluster_moved_points


def _track_colour(track_id: int) -> tuple:
    """
    Deterministic, evenly-spread RGB colour for a track ID.

    Walking the hue wheel by the golden ratio (rather than an 8-entry
    lookup table indexed with %) means colours don't repeat until floating
    point precision runs out, not after the 9th track -- long sessions
    with many track IDs no longer alias two unrelated tracks onto the same
    colour.
    """
    hue = (track_id * 0.6180339887) % 1.0
    return colorsys.hsv_to_rgb(hue, 0.85, 1.0)


class OnlinePerceptionNode(Node):
    def __init__(self):
        super().__init__("online_perception_node")

        self.declare_parameter("accumulate_scans", 10)
        self.declare_parameter("voxel_size", 0.05)
        self.declare_parameter("change_threshold", 0.15)
        self.declare_parameter("cluster_eps", 0.5)
        self.declare_parameter("cluster_min_points", 10)
        self.declare_parameter("max_match_distance", 1.5)
        self.declare_parameter("max_missed_frames", 3)
        self.declare_parameter("kalman_position_std", 0.1)
        self.declare_parameter("kalman_velocity_std", 2.0)
        self.declare_parameter("kalman_process_std", 1.0)
        self.declare_parameter("z_max", 2.5)
        self.declare_parameter("frame_id", "camera_init")

        self._accumulate_scans = self.get_parameter("accumulate_scans").value
        self._voxel_size = self.get_parameter("voxel_size").value
        self._change_threshold = self.get_parameter("change_threshold").value
        self._cluster_eps = self.get_parameter("cluster_eps").value
        self._cluster_min_points = self.get_parameter("cluster_min_points").value
        self._z_max = self.get_parameter("z_max").value
        self._frame_id = self.get_parameter("frame_id").value

        self._max_missed_frames = self.get_parameter("max_missed_frames").value
        self._tracker = CentroidTracker(
            max_match_distance=self.get_parameter("max_match_distance").value,
            max_missed_frames=self._max_missed_frames,
            position_variance=self.get_parameter("kalman_position_std").value ** 2,
            velocity_variance=self.get_parameter("kalman_velocity_std").value ** 2,
            process_variance=self.get_parameter("kalman_process_std").value ** 2,
        )

        self._scan_buffer = []
        self._prev_frame = None            # downsampled points, last completed frame
        self._prev_frame_stamp = None
        self._frame_count = 0
        self._scan_count = 0
        self._latest_odom = None

        self._cloud_sub = self.create_subscription(
            PointCloud2, "/cloud_registered", self._cloud_cb, 10)
        self._odom_sub = self.create_subscription(
            Odometry, "/Odometry", self._odom_cb, 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, "/online_perception/markers", 10)

        self.get_logger().info(
            f"online_perception_node ready -- accumulating "
            f"{self._accumulate_scans} scans per frame "
            f"(eps={self._cluster_eps} min_points={self._cluster_min_points} "
            f"threshold={self._change_threshold} z_max={self._z_max}, "
            f"max_missed_frames={self._max_missed_frames})")

    def _odom_cb(self, msg: Odometry) -> None:
        self._latest_odom = msg

    def _cloud_cb(self, msg: PointCloud2) -> None:
        points = pointcloud2_to_xyz(msg)
        self._scan_count += 1
        if len(points) == 0:
            return

        self._scan_buffer.append(points)
        if len(self._scan_buffer) < self._accumulate_scans:
            return

        frame = np.concatenate(self._scan_buffer, axis=0)
        self._scan_buffer = []
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._process_frame(frame, stamp)

    def _process_frame(self, frame: np.ndarray, stamp: float) -> None:
        frame = voxel_downsample(frame, self._voxel_size)
        if self._z_max is not None:
            frame = frame[frame[:, 2] <= self._z_max]

        self._frame_count += 1

        if self._prev_frame is None or len(self._prev_frame) == 0:
            self._prev_frame = frame
            self._prev_frame_stamp = stamp
            self.get_logger().info(
                f"frame {self._frame_count}: {len(frame)} pts "
                f"({self._scan_count} scans so far) -- first frame, "
                f"nothing to compare against yet")
            return

        # Points in this frame far from anything in the previous frame are
        # considered "moved".
        tree = cKDTree(self._prev_frame)
        distances, _ = tree.query(frame, k=1)
        moved = frame[distances > self._change_threshold]

        clusters = cluster_moved_points(moved, self._cluster_eps, self._cluster_min_points)

        dt = stamp - self._prev_frame_stamp if self._prev_frame_stamp else 0.0
        self.get_logger().info(
            f"frame {self._frame_count}: {len(frame)} pts, "
            f"{len(moved)} moved, {len(clusters)} clusters (dt={dt:.2f}s)")
        self._update_tracks(clusters, dt)

        self._prev_frame = frame
        self._prev_frame_stamp = stamp

    def _update_tracks(self, clusters, dt) -> None:
        active_tracks = self._tracker.step(clusters, dt)

        markers = MarkerArray()
        for track_id, info in active_tracks.items():
            track = info["track"]
            speed = float(np.linalg.norm(track.velocity))

            status = "new" if info["is_new"] else ("coasting" if info["is_coasting"] else "matched")
            self.get_logger().info(
                f"  track {track_id} [{status}]: "
                f"({track.position[0]:.2f}, {track.position[1]:.2f}, "
                f"{track.position[2]:.2f}) m, {track.n_points} pts, "
                f"{speed:.2f} m/s")
            markers.markers.extend(self._build_markers(track_id, track, speed, info["is_coasting"]))

        if markers.markers:
            self._marker_pub.publish(markers)

    def _build_markers(self, track_id, track, speed, is_coasting):
        colour = _track_colour(track_id)
        centroid = track.position
        now = self.get_clock().now().to_msg()
        # Coasting tracks are drawn faded out -- a predicted position with
        # no detection to back it up this frame, not a confirmed sighting.
        alpha = 0.35 if is_coasting else 0.8

        sphere = Marker()
        sphere.header.frame_id = self._frame_id
        sphere.header.stamp = now
        sphere.ns = "online_perception"
        sphere.id = track_id * 2
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        (sphere.pose.position.x,
         sphere.pose.position.y,
         sphere.pose.position.z) = (float(centroid[0]), float(centroid[1]), float(centroid[2]))
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.3
        sphere.color.r, sphere.color.g, sphere.color.b = colour
        sphere.color.a = alpha
        sphere.lifetime.sec = 1

        label = Marker()
        label.header.frame_id = self._frame_id
        label.header.stamp = now
        label.ns = "online_perception_labels"
        label.id = track_id * 2 + 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = float(centroid[0])
        label.pose.position.y = float(centroid[1])
        label.pose.position.z = float(centroid[2]) + 0.4
        label.pose.orientation.w = 1.0
        label.scale.z = 0.25
        label.color.r, label.color.g, label.color.b = colour
        label.color.a = 1.0 if not is_coasting else 0.6
        suffix = " (coasting)" if is_coasting else ""
        label.text = f"track {track_id}: {speed:.2f} m/s{suffix}"
        label.lifetime.sec = 1

        return [sphere, label]


def main():
    rclpy.init()
    node = OnlinePerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
