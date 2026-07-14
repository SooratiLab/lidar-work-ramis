#!/usr/bin/env python3
"""
online_perception_node.py -- live frame-to-frame motion detection on
FastLIO's /cloud_registered + /Odometry, without going through PCD files
on disk first.

This is the streaming counterpart to
lidar-perception/scripts/track_motion.py in kei-stuff. That script's
pipeline is unchanged here -- accumulate scans into fixed-duration frames,
diff consecutive frames, DBSCAN the moved points, match clusters across
frames by nearest centroid, report velocity. What's different is that it
runs against a live topic subscription instead of a folder of pre-exported
PCD files, so it can run alongside a live sensor -- or, until one is
reachable, a replayed bag standing in for one -- instead of after the fact.

Coordinate units: metres throughout, matching ROS conventions and FastLIO's
own /cloud_registered and /Odometry output directly. This deliberately does
NOT convert to millimetres the way the offline pipeline's PCD files do (see
export_fastlio.py in kei-stuff) -- there's no PCD file in this path to match
units with, so there's no reason to introduce that convention here. Every
distance parameter below (voxel size, thresholds, etc.) is in metres.

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

Parameters (all overridable via --ros-args -p <name>:=<value>):
    accumulate_scans     (int,   default 10)    scans merged per frame --
                                                 see lidar-perception/
                                                 README.md's --accumulate
                                                 for the offline equivalent
    voxel_size            (float, default 0.05)  downsample voxel size (m)
    change_threshold      (float, default 0.15)  moved-point distance (m)
    cluster_eps           (float, default 0.5)   DBSCAN radius (m)
    cluster_min_points    (int,   default 10)    DBSCAN min_samples
    max_match_distance    (float, default 1.5)   track-matching radius (m)
    z_max                 (float, default 2.5)   ceiling crop height (m)

Defaults match the FastLIO-tuned values in lidar-perception/README.md's
track_motion.py parameter table (converted mm -> m), since that's the best
available starting point for FastLIO's sparse, moving-sensor point density
-- not re-derived from scratch here. They are a starting point, not a
guarantee; re-tune against whatever this node's actual frame density turns
out to be once running against a live sensor (see DOCS.md).
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from sklearn.cluster import DBSCAN
from scipy.spatial import cKDTree


# sensor_msgs/msg/PointField datatype constants -> numpy dtype characters.
_POINTFIELD_TO_NUMPY = {
    1: "i1", 2: "u1",
    3: "i2", 4: "u2",
    5: "i4", 6: "u4",
    7: "f4", 8: "f8",
}


def pointcloud2_to_xyz(msg: PointCloud2) -> np.ndarray:
    """
    Parse a PointCloud2 into an (N, 3) float64 array of x, y, z (metres).

    Builds a numpy structured dtype directly from the message's own field
    layout (name/offset/datatype) rather than assuming FLOAT32 XYZ packed
    at offsets 0/4/8 -- works whether or not an intensity field is present,
    and regardless of field order. Uses np.frombuffer to reinterpret the
    message bytes directly instead of a per-point Python loop (the offline
    pipeline's export_fastlio.py can afford that loop since it runs once
    per recording; this runs every scan, live).
    """
    names, formats, offsets = [], [], []
    for field in msg.fields:
        if field.name not in ("x", "y", "z"):
            continue
        names.append(field.name)
        formats.append(_POINTFIELD_TO_NUMPY[field.datatype])
        offsets.append(field.offset)

    dtype = np.dtype({
        "names": names,
        "formats": formats,
        "offsets": offsets,
        "itemsize": msg.point_step,
    })

    count = msg.width * msg.height
    structured = np.frombuffer(msg.data, dtype=dtype, count=count)
    return np.column_stack(
        [structured["x"], structured["y"], structured["z"]]
    ).astype(np.float64)


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """
    Cheap voxel-grid downsample: keep one representative point per occupied
    voxel cell. Not a centroid average (unlike Open3D's voxel_down_sample)
    -- picks an arbitrary point per cell, which is fine for the
    distance-threshold change detection this feeds into, and avoids an
    Open3D dependency that's awkward to install on the Jetson (see DOCS.md
    -- point cloud density and dependency weight are already flagged as
    concerns for the eventual on-Jetson deployment).
    """
    if len(points) == 0:
        return points
    keys = np.floor(points / voxel_size).astype(np.int64)
    _, unique_idx = np.unique(keys, axis=0, return_index=True)
    return points[unique_idx]


TRACK_COLOURS = [
    (1.0, 0.0, 0.0), (0.0, 0.5, 1.0), (1.0, 0.5, 0.0), (0.0, 1.0, 1.0),
    (1.0, 0.0, 1.0), (1.0, 1.0, 0.0), (0.5, 0.0, 1.0), (0.0, 1.0, 0.5),
]


class OnlinePerceptionNode(Node):
    def __init__(self):
        super().__init__("online_perception_node")

        self.declare_parameter("accumulate_scans", 10)
        self.declare_parameter("voxel_size", 0.05)
        self.declare_parameter("change_threshold", 0.15)
        self.declare_parameter("cluster_eps", 0.5)
        self.declare_parameter("cluster_min_points", 10)
        self.declare_parameter("max_match_distance", 1.5)
        self.declare_parameter("z_max", 2.5)
        self.declare_parameter("frame_id", "camera_init")

        self._accumulate_scans = self.get_parameter("accumulate_scans").value
        self._voxel_size = self.get_parameter("voxel_size").value
        self._change_threshold = self.get_parameter("change_threshold").value
        self._cluster_eps = self.get_parameter("cluster_eps").value
        self._cluster_min_points = self.get_parameter("cluster_min_points").value
        self._max_match_distance = self.get_parameter("max_match_distance").value
        self._z_max = self.get_parameter("z_max").value
        self._frame_id = self.get_parameter("frame_id").value

        self._scan_buffer = []
        self._prev_frame = None            # downsampled points, last completed frame
        self._prev_frame_stamp = None
        self._prev_clusters = []           # clusters from the last frame transition
        self._prev_cluster_to_track = {}   # cluster index -> track id, last transition
        self._tracks = {}                  # track id -> (last_centroid, last_stamp)
        self._next_track_id = 0
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
            f"threshold={self._change_threshold} z_max={self._z_max})")

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

        clusters = []
        if len(moved) >= self._cluster_min_points:
            labels = DBSCAN(
                eps=self._cluster_eps, min_samples=self._cluster_min_points
            ).fit_predict(moved)
            for label in set(labels):
                if label == -1:
                    continue
                cluster_points = moved[labels == label]
                clusters.append({
                    "centroid": cluster_points.mean(axis=0),
                    "n_points": int(len(cluster_points)),
                })

        dt = stamp - self._prev_frame_stamp if self._prev_frame_stamp else 0.0
        self.get_logger().info(
            f"frame {self._frame_count}: {len(frame)} pts, "
            f"{len(moved)} moved, {len(clusters)} clusters (dt={dt:.2f}s)")
        self._update_tracks(clusters, stamp, dt)

        self._prev_frame = frame
        self._prev_frame_stamp = stamp
        self._prev_clusters = clusters

    def _update_tracks(self, clusters, stamp, dt) -> None:
        curr_cluster_to_track = {}

        if self._prev_clusters and clusters:
            prev_centroids = np.array([c["centroid"] for c in self._prev_clusters])
            used_prev = set()
            for ci, cluster in enumerate(clusters):
                dists = np.linalg.norm(prev_centroids - cluster["centroid"], axis=1)
                for pi in np.argsort(dists):
                    if dists[pi] > self._max_match_distance:
                        break
                    if pi in used_prev:
                        continue
                    used_prev.add(pi)
                    if pi in self._prev_cluster_to_track:
                        curr_cluster_to_track[ci] = self._prev_cluster_to_track[pi]
                    break

        markers = MarkerArray()
        for ci, cluster in enumerate(clusters):
            if ci in curr_cluster_to_track:
                track_id = curr_cluster_to_track[ci]
                prev_centroid, _ = self._tracks[track_id]
                displacement = np.linalg.norm(cluster["centroid"] - prev_centroid)
                speed = displacement / dt if dt > 0 else 0.0
            else:
                track_id = self._next_track_id
                self._next_track_id += 1
                speed = 0.0
                curr_cluster_to_track[ci] = track_id

            self._tracks[track_id] = (cluster["centroid"], stamp)
            self.get_logger().info(
                f"  track {track_id}: "
                f"({cluster['centroid'][0]:.2f}, {cluster['centroid'][1]:.2f}, "
                f"{cluster['centroid'][2]:.2f}) m, {cluster['n_points']} pts, "
                f"{speed:.2f} m/s")
            markers.markers.extend(self._build_markers(track_id, cluster, speed))

        self._prev_cluster_to_track = curr_cluster_to_track
        if markers.markers:
            self._marker_pub.publish(markers)

    def _build_markers(self, track_id, cluster, speed):
        colour = TRACK_COLOURS[track_id % len(TRACK_COLOURS)]
        centroid = cluster["centroid"]
        now = self.get_clock().now().to_msg()

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
        sphere.color.a = 0.8
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
        label.color.a = 1.0
        label.text = f"track {track_id}: {speed:.2f} m/s"
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
