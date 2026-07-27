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
    /Odometry            (nav_msgs/Odometry)  -- the robot's current
                          position gates implausible detections (see
                          max_sensor_range below) and anchors each frame's
                          range image for the visibility gate (see
                          use_visibility_gate below); not used to
                          motion-compensate beyond what FastLIO already
                          bakes into /cloud_registered.

Publishes:
    /online_perception/markers   (visualization_msgs/MarkerArray) -- one
                                   sphere + one text label per confirmed
                                   track, viewable in RViz2 against
                                   frame_id (default "camera_init", the
                                   frame FastLIO itself publishes in).
                                   Coasting tracks (predicted position, no
                                   detection this frame) are drawn at
                                   reduced opacity so it's visually obvious
                                   when a track is being carried through a
                                   gap rather than freshly confirmed.
                                   Tentative tracks (fewer than min_hits
                                   real detections so far) are not
                                   published at all -- see min_hits below.
    /online_perception/tracks     (geometry_msgs/PoseArray) -- confirmed
                                   detections backed by a measurement in
                                   the current frame, including an empty
                                   array when no track is observed. Unlike
                                   the RViz markers, coasted predictions
                                   are excluded so control consumers do
                                   not react to stale evidence.
    /online_perception/track_observations (std_msgs/String, JSON) -- the
                                   same current confirmed tracks with local
                                   track ID, velocity, cluster size, point
                                   count, source timestamp, sensor position,
                                   and dog_id preserved for the inter-dog
                                   exchange prototype. This is informational;
                                   the stop policy continues to consume the
                                   simpler PoseArray boundary above.

Parameters (all overridable via --ros-args -p <name>:=<value>):
    accumulate_scans     (int,   default 10)    scans merged per frame --
                                                 see lidar-perception/
                                                 README.md's --accumulate
                                                 for the offline equivalent
    accumulate_stride     (int,   default 0)      scans between processed
                                                 frames -- 0 means "same as
                                                 accumulate_scans" (the
                                                 original behaviour: each
                                                 frame is a fresh,
                                                 non-overlapping window).
                                                 Set lower than
                                                 accumulate_scans to
                                                 overlap consecutive
                                                 frames and report more
                                                 often -- e.g.
                                                 accumulate_scans=10 and
                                                 accumulate_stride=5 doubles
                                                 the processing frequency
                                                 and roughly doubles the
                                                 clustering work. A bag A/B
                                                 test found no detection-
                                                 latency improvement because
                                                 each comparison also had
                                                 roughly half the motion
                                                 signal; see perception/
                                                 README.md's "Overlapping
                                                 frames" section. Overlap
                                                 therefore remains opt-in,
                                                 not the default. It does
                                                 not change what counts as
                                                 "moved" between two
                                                 frames -- it only changes
                                                 how often a new frame
                                                 boundary is drawn through
                                                 the same underlying scan
                                                 stream.
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
    max_missed_seconds   (float, default 3.0)   real seconds a track keeps
                                                 coasting before it's
                                                 dropped, whichever of this
                                                 and max_missed_frames comes
                                                 first -- catches the case a
                                                 frame count alone can't:
                                                 testing against a session
                                                 with a degraded, bursty
                                                 scan rate found a track
                                                 that survived a 42-second
                                                 real gap because only 3
                                                 *frames* happened to occur
                                                 in that stretch, coasting
                                                 its prediction to a
                                                 position 15+ metres from
                                                 anywhere real
    min_hits              (int,   default 2)     real detections a track
                                                 needs before it's
                                                 "confirmed" and reported --
                                                 filters single-frame noise
                                                 clusters, which testing
                                                 against recorded sessions
                                                 showed never get a second
                                                 real detection (see
                                                 tracking.py's
                                                 CentroidTracker docstring)
    reid_max_distance     (float, default 2.0)   metres a new detection
                                                 must fall within of a
                                                 recently-dropped track's
                                                 last known position to
                                                 revive that track's ID
                                                 instead of starting a new
                                                 one -- bridges a genuine
                                                 stop or occlusion longer
                                                 than max_missed_frames/
                                                 max_missed_seconds without
                                                 extending the coasting
                                                 window itself (which would
                                                 extrapolate an increasingly
                                                 untrustworthy velocity
                                                 across the whole gap
                                                 instead). Deliberately
                                                 gates on the last known
                                                 *position*, not an
                                                 extrapolated one -- see
                                                 tracking.py's
                                                 CentroidTracker docstring
                                                 for why this only helps a
                                                 real stop-in-place, not a
                                                 missed detection during
                                                 continued walking.
    reid_window_seconds    (float, default 15.0)  how long a dropped
                                                 track's last known
                                                 position stays eligible
                                                 for re-identification
                                                 before being forgotten for
                                                 good -- long enough to
                                                 bridge a real pause, short
                                                 enough that a much later,
                                                 unrelated detection at the
                                                 same spot doesn't
                                                 incorrectly inherit an old
                                                 track's identity and hit
                                                 count.
    max_sensor_range      (float, default 40.0)  detections farther than
                                                 this from the robot's
                                                 current /Odometry position
                                                 are dropped as implausible
                                                 -- guards against reporting
                                                 phantom "tracks" if
                                                 FastLIO's registration ever
                                                 drifts (seen on degraded
                                                 pre-DDS-fix recordings
                                                 during testing, where scan-
                                                 matching failures put
                                                 /cloud_registered points
                                                 hundreds of metres from
                                                 anywhere real). A Mid-360's
                                                 rated range for typical
                                                 (~10% reflectivity) targets
                                                 is well under this; a
                                                 "detection" beyond it is
                                                 far more likely to be
                                                 drifted odometry than a
                                                 real long-range return.
    use_visibility_gate   (bool,  default True)  suppress "moved" points
                                                 the previous frame's own
                                                 sensor position couldn't
                                                 have seen -- see
                                                 range_image.py. This is the
                                                 fix for the moving-sensor
                                                 false-positive problem
                                                 documented in
                                                 perception/README.md's
                                                 "Open risk for the Jetson
                                                 port": on a moving sensor,
                                                 newly-visible static
                                                 geometry (rounding a
                                                 corner, a wall the sensor
                                                 just got closer to) has no
                                                 nearby point in the
                                                 previous frame either, and
                                                 the plain change_threshold
                                                 test alone can't tell that
                                                 apart from something
                                                 genuinely moving. Left as a
                                                 parameter (rather than
                                                 unconditional) so a
                                                 before/after comparison
                                                 against the same recorded
                                                 session is one flag, not a
                                                 code change.
    range_image_azimuth_bins    (int, default 72)   horizontal resolution
                                                 (5 degrees/bin) of the
                                                 visibility gate's range
                                                 image. Coarser than a
                                                 first attempt at 2 degrees/
                                                 bin (180 bins) -- testing
                                                 against soton_indoor showed
                                                 that resolution was finer
                                                 than the ~5k points/frame
                                                 FastLIO actually produces
                                                 can fill: most bins in a
                                                 person's own solid angle
                                                 came up empty in the
                                                 previous frame purely from
                                                 sampling sparsity, not
                                                 genuine unvisited
                                                 directions, and the gate
                                                 wrongly dropped most of a
                                                 real, walking-pace track
                                                 along with the false
                                                 positives it was meant to
                                                 catch.
    range_image_elevation_bins  (int, default 36)   vertical resolution
                                                 (5 degrees/bin), spanning
                                                 the full +/-90 degrees --
                                                 deliberately not narrowed
                                                 to the Mid-360's actual
                                                 vertical FOV, since a
                                                 direction the sensor can't
                                                 physically reach already
                                                 shows up as an empty bin
                                                 without needing to encode
                                                 that separately (see
                                                 range_image.py). Same
                                                 sparsity reasoning as
                                                 range_image_azimuth_bins
                                                 above.
    range_image_tolerance        (float, default 0.3)  metres a candidate
                                                 point's range along a
                                                 previously-seen direction
                                                 must undercut the previous
                                                 frame's range by before
                                                 it's kept as "moved" --
                                                 looser than
                                                 change_threshold since this
                                                 compares a single ray
                                                 across two different
                                                 viewpoints and a
                                                 discretised bin, not
                                                 nearest-neighbour distance
                                                 within one point cloud.
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
    frame_id               (str,   default "camera_init")  frame markers
                                                 are published in -- match
                                                 whatever frame FastLIO is
                                                 actually publishing
                                                 /cloud_registered in if
                                                 that ever changes
    dog_id                 (str,   default "dog") source identifier in rich
                                                 inter-dog track packets;
                                                 set uniquely per robot

Defaults for the detection-side parameters match the FastLIO-tuned values in
lidar-perception/README.md's track_motion.py parameter table (converted
mm -> m), since that's the best available starting point for FastLIO's
sparse, moving-sensor point density -- not re-derived from scratch here.
They are a starting point, not a guarantee; re-tune against whatever this
node's actual frame density turns out to be once running against a live
sensor. The Kalman noise, min_hits, max_sensor_range, and range_image_*
parameters are new and have no offline equivalent to carry over -- their
defaults are reasoned (see each one's docstring above) and validated
against recorded sessions during testing, not tuned against a live sensor
yet.
"""
import colorsys
import json
import os
import sys
import time
import uuid
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

# Local, non-ROS modules living alongside this script -- see pointcloud.py,
# tracking.py, and range_image.py for what moved out of this file and why.
# Explicit sys.path insert (rather than relying on the script's directory
# already being first on sys.path) so this still works if the node is ever
# launched via `python3 -m` or an entry point instead of run as a plain
# script, matching the same defensive pattern kei-stuff/lidar-perception/
# scripts already uses for its own local imports.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from pointcloud import pointcloud2_to_xyz, voxel_downsample
from tracking import CentroidTracker, cluster_moved_points, filter_plausible_detections
from range_image import build_range_image, previously_visible_mask


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
        self.declare_parameter("accumulate_stride", 0)
        self.declare_parameter("voxel_size", 0.05)
        self.declare_parameter("change_threshold", 0.15)
        self.declare_parameter("cluster_eps", 0.5)
        self.declare_parameter("cluster_min_points", 10)
        self.declare_parameter("max_match_distance", 1.5)
        self.declare_parameter("max_missed_frames", 3)
        self.declare_parameter("max_missed_seconds", 3.0)
        self.declare_parameter("min_hits", 2)
        self.declare_parameter("reid_max_distance", 2.0)
        self.declare_parameter("reid_window_seconds", 15.0)
        self.declare_parameter("max_sensor_range", 40.0)
        self.declare_parameter("use_visibility_gate", True)
        self.declare_parameter("range_image_azimuth_bins", 72)
        self.declare_parameter("range_image_elevation_bins", 36)
        self.declare_parameter("range_image_tolerance", 0.3)
        self.declare_parameter("kalman_position_std", 0.1)
        self.declare_parameter("kalman_velocity_std", 2.0)
        self.declare_parameter("kalman_process_std", 1.0)
        self.declare_parameter("z_max", 2.5)
        self.declare_parameter("frame_id", "camera_init")
        self.declare_parameter("dog_id", os.getenv("DOG_ID", "dog"))

        self._accumulate_scans = self.get_parameter("accumulate_scans").value
        # 0 is a sentinel for "same as accumulate_scans" (the original,
        # non-overlapping behaviour) -- lets accumulate_scans stay the
        # single source of truth for the default instead of duplicating
        # its value into a second parameter's default.
        self._accumulate_stride = self.get_parameter("accumulate_stride").value or self._accumulate_scans
        self._voxel_size = self.get_parameter("voxel_size").value
        self._change_threshold = self.get_parameter("change_threshold").value
        self._cluster_eps = self.get_parameter("cluster_eps").value
        self._cluster_min_points = self.get_parameter("cluster_min_points").value
        self._max_sensor_range = self.get_parameter("max_sensor_range").value
        self._use_visibility_gate = self.get_parameter("use_visibility_gate").value
        self._range_image_azimuth_bins = self.get_parameter("range_image_azimuth_bins").value
        self._range_image_elevation_bins = self.get_parameter("range_image_elevation_bins").value
        self._range_image_tolerance = self.get_parameter("range_image_tolerance").value
        self._z_max = self.get_parameter("z_max").value
        self._frame_id = self.get_parameter("frame_id").value
        self._dog_id = self.get_parameter("dog_id").value
        if not self._dog_id:
            raise ValueError("dog_id must be a non-empty string")

        self._max_missed_frames = self.get_parameter("max_missed_frames").value
        self._max_missed_seconds = self.get_parameter("max_missed_seconds").value
        self._reid_max_distance = self.get_parameter("reid_max_distance").value
        self._reid_window_seconds = self.get_parameter("reid_window_seconds").value
        self._tracker = CentroidTracker(
            max_match_distance=self.get_parameter("max_match_distance").value,
            max_missed_frames=self._max_missed_frames,
            max_missed_seconds=self._max_missed_seconds,
            min_hits=self.get_parameter("min_hits").value,
            position_variance=self.get_parameter("kalman_position_std").value ** 2,
            velocity_variance=self.get_parameter("kalman_velocity_std").value ** 2,
            process_variance=self.get_parameter("kalman_process_std").value ** 2,
            reid_max_distance=self._reid_max_distance,
            reid_window_seconds=self._reid_window_seconds,
        )

        self._scan_buffer = deque(maxlen=self._accumulate_scans)
        self._scans_since_last_frame = 0
        self._prev_frame = None            # downsampled points, last completed frame
        self._prev_frame_stamp = None
        self._prev_frame_position = None   # sensor position (m) at that frame, for the visibility gate
        self._frame_count = 0
        # Local track IDs and frame sequence numbers restart with this node.
        # A boot/session identifier lets a peer distinguish that legitimate
        # reset from a delayed UDP datagram from the previous process.
        self._session_id = str(uuid.uuid4())
        self._scan_count = 0
        self._latest_odom = None
        self._logged_missing_odom_warning = False
        self._logged_missing_odom_for_visibility_warning = False
        self._timing_samples = []  # last 50 _process_frame wall-clock durations (s)

        self._cloud_sub = self.create_subscription(
            PointCloud2, "/cloud_registered", self._cloud_cb, 10)
        self._odom_sub = self.create_subscription(
            Odometry, "/Odometry", self._odom_cb, 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, "/online_perception/markers", 10)
        self._tracks_pub = self.create_publisher(
            PoseArray, "/online_perception/tracks", 10)
        self._track_observations_pub = self.create_publisher(
            String, "/online_perception/track_observations", 10)

        self.get_logger().info(
            f"online_perception_node ready -- accumulating "
            f"{self._accumulate_scans} scans per frame "
            f"(eps={self._cluster_eps} min_points={self._cluster_min_points} "
            f"threshold={self._change_threshold} z_max={self._z_max}, "
            f"max_missed_frames={self._max_missed_frames}, "
            f"max_missed_seconds={self._max_missed_seconds}, "
            f"reid_max_distance={self._reid_max_distance}, "
            f"reid_window_seconds={self._reid_window_seconds}, "
            f"max_sensor_range={self._max_sensor_range}, "
            f"use_visibility_gate={self._use_visibility_gate})")

    def _odom_cb(self, msg: Odometry) -> None:
        self._latest_odom = msg

    def _odom_position(self):
        """Current /Odometry position (m) as a (3,) array, or None if no
        /Odometry message has arrived yet."""
        if self._latest_odom is None:
            return None
        p = self._latest_odom.pose.pose.position
        return np.array([p.x, p.y, p.z])

    def _cloud_cb(self, msg: PointCloud2) -> None:
        points = pointcloud2_to_xyz(msg)
        self._scan_count += 1
        if len(points) == 0:
            return

        # A fixed-size sliding window (deque, maxlen=accumulate_scans)
        # rather than a list that gets cleared every accumulate_scans
        # scans -- this is what makes accumulate_stride < accumulate_scans
        # (overlapping frames) possible without restructuring anything
        # else: the window always holds the most recent accumulate_scans
        # scans, and a new frame is processed every accumulate_stride
        # scans rather than only once the window has been completely
        # replaced. With the default accumulate_stride == accumulate_scans
        # this behaves exactly like the original clear-and-refill buffer.
        self._scan_buffer.append(points)
        self._scans_since_last_frame += 1
        if len(self._scan_buffer) < self._accumulate_scans:
            return
        if self._scans_since_last_frame < self._accumulate_stride:
            return

        frame = np.concatenate(self._scan_buffer, axis=0)
        self._scans_since_last_frame = 0
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._process_frame(frame, stamp)

    def _process_frame(self, frame: np.ndarray, stamp: float) -> None:
        # Wall-clock processing time for this frame, measured against the
        # real-world time it represents (dt below) -- the real-time budget
        # question that matters for the eventual Jetson port isn't "is this
        # fast" in the abstract, it's "does processing a frame take less
        # wall-clock time than the frame itself spans." A laptop's numbers
        # here are optimistic for a Jetson's weaker CPU, but a pipeline
        # that's already using most of its budget on x86 is a clear warning
        # sign; one comfortably inside budget is a necessary (not
        # sufficient) condition for the port to have a chance.
        processing_start = time.perf_counter()

        # Sensor position "as of" this frame -- captured now, at the point
        # this frame is complete, so it becomes the *previous* frame's
        # reference position by the time the next frame runs the
        # visibility gate below. Same snapshot-at-completion convention
        # already used for the max_sensor_range plausibility gate.
        frame_position = self._odom_position()

        frame = voxel_downsample(frame, self._voxel_size)
        if self._z_max is not None:
            frame = frame[frame[:, 2] <= self._z_max]

        self._frame_count += 1

        if self._prev_frame is None or len(self._prev_frame) == 0:
            self._prev_frame = frame
            self._prev_frame_stamp = stamp
            self._prev_frame_position = frame_position
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

        moved, n_unseen = self._apply_visibility_gate(moved)

        clusters = cluster_moved_points(moved, self._cluster_eps, self._cluster_min_points)
        clusters, n_implausible = self._drop_implausible_clusters(clusters)

        dt = stamp - self._prev_frame_stamp if self._prev_frame_stamp is not None else 0.0
        self._update_tracks(clusters, dt, stamp)

        processing_s = time.perf_counter() - processing_start
        self._log_frame_timing(processing_s, dt)
        self.get_logger().info(
            f"frame {self._frame_count}: {len(frame)} pts, "
            f"{len(moved)} moved ({n_unseen} dropped as previously "
            f"unseen/background), {len(clusters)} clusters "
            f"({n_implausible} dropped as implausible) (dt={dt:.2f}s, "
            f"processed in {processing_s * 1000:.1f}ms)")

        self._prev_frame = frame
        self._prev_frame_stamp = stamp
        self._prev_frame_position = frame_position

    def _apply_visibility_gate(self, moved_points: np.ndarray):
        """
        Suppress "moved" points the previous frame's own sensor position
        couldn't have told us anything about -- see range_image.py and
        use_visibility_gate in the module docstring for the full
        reasoning. Returns (surviving_points, n_dropped).

        Fails open (no filtering) if disabled, if there are no candidate
        points to check, or if no /Odometry has arrived yet to anchor the
        previous frame's range image -- same rationale as
        _drop_implausible_clusters below: refusing to detect anything
        until odometry shows up is a worse failure mode than letting a
        few frames through unfiltered while it's still arriving.
        """
        if not self._use_visibility_gate or len(moved_points) == 0:
            return moved_points, 0

        if self._prev_frame_position is None:
            if not self._logged_missing_odom_for_visibility_warning:
                self.get_logger().warning(
                    "no /Odometry received yet -- visibility gate "
                    "disabled until it arrives")
                self._logged_missing_odom_for_visibility_warning = True
            return moved_points, 0

        prev_range_image = build_range_image(
            self._prev_frame, self._prev_frame_position,
            self._range_image_azimuth_bins, self._range_image_elevation_bins)
        keep = previously_visible_mask(
            moved_points, self._prev_frame_position, prev_range_image,
            self._range_image_azimuth_bins, self._range_image_elevation_bins,
            self._range_image_tolerance)
        return moved_points[keep], int((~keep).sum())

    def _drop_implausible_clusters(self, clusters):
        """
        Apply the odometry-based plausibility gate (see max_sensor_range in
        the module docstring). Returns (surviving_clusters, n_dropped).

        Fails open (no filtering) until the first /Odometry message
        arrives -- FastLIO publishes odometry and cloud_registered
        together per scan, so in practice this only matters for the very
        first frame or two, and refusing to detect anything just because
        odometry hasn't arrived yet would be a worse failure mode than
        occasionally letting an implausible detection through before the
        gate is live.
        """
        if not clusters or self._latest_odom is None:
            if self._latest_odom is None and not self._logged_missing_odom_warning:
                self.get_logger().warning(
                    "no /Odometry received yet -- plausibility filtering "
                    "disabled until it arrives")
                self._logged_missing_odom_warning = True
            return clusters, 0

        sensor_position = self._odom_position()
        filtered = filter_plausible_detections(clusters, sensor_position, self._max_sensor_range)
        return filtered, len(clusters) - len(filtered)

    def _log_frame_timing(self, processing_s: float, dt: float) -> None:
        """
        Warn once processing a frame starts eating into the real-time
        budget dt represents -- the earliest, cheapest signal that either
        the accumulate/voxel/cluster parameters need to shrink or the
        underlying hardware needs to be faster, well before this actually
        falls behind on a live sensor and starts silently dropping scans.
        """
        self._timing_samples.append(processing_s)
        if len(self._timing_samples) > 50:
            self._timing_samples.pop(0)

        if dt > 0 and processing_s > 0.5 * dt:
            self.get_logger().warning(
                f"frame {self._frame_count} took {processing_s * 1000:.1f}ms "
                f"to process a {dt:.2f}s frame ({processing_s / dt * 100:.0f}% "
                f"of budget) -- getting close to falling behind real time")

        if self._frame_count % 20 == 0:
            samples = np.array(self._timing_samples)
            self.get_logger().info(
                f"processing time over last {len(samples)} frames: "
                f"mean {samples.mean() * 1000:.1f}ms, "
                f"max {samples.max() * 1000:.1f}ms")

    def _update_tracks(self, clusters, dt, stamp) -> None:
        active_tracks = self._tracker.step(clusters, dt)

        markers = MarkerArray()
        tracks = PoseArray()
        tracks.header.frame_id = self._frame_id
        tracks.header.stamp = self.get_clock().now().to_msg()
        rich_tracks = []
        for track_id, info in active_tracks.items():
            # Tentative tracks (fewer than min_hits real detections) are
            # tracked internally so they have a chance to become confirmed,
            # but not logged or published -- testing showed most of these
            # are single-frame noise that never gets a second detection at
            # all, and reporting them identically to a real track made the
            # output far noisier than the underlying detection rate
            # justified. See tracking.py's CentroidTracker docstring.
            if not info["is_confirmed"]:
                continue

            track = info["track"]
            speed = float(np.linalg.norm(track.velocity))

            status = ("new" if info["is_new"]
                       else "reidentified" if info["is_reidentified"]
                       else "coasting" if info["is_coasting"]
                       else "matched")
            self.get_logger().info(
                f"  track {track_id} [{status}]: "
                f"({track.position[0]:.2f}, {track.position[1]:.2f}, "
                f"{track.position[2]:.2f}) m, {track.n_points} pts, "
                f"{speed:.2f} m/s")
            markers.markers.extend(
                self._build_markers(track_id, track, speed, info["is_coasting"], info["is_reidentified"]))
            # PoseArray is intentionally limited to detections backed by a
            # measurement in this frame. A coasted Kalman prediction is
            # useful for visual continuity, but should not trigger a robot
            # response without a current observation.
            if not info["is_coasting"]:
                pose = Pose()
                pose.position.x = float(track.position[0])
                pose.position.y = float(track.position[1])
                pose.position.z = float(track.position[2])
                pose.orientation.w = 1.0
                tracks.poses.append(pose)
                rich_tracks.append({
                    "local_id": track_id,
                    "position_m": track.position.tolist(),
                    "velocity_m_s": track.velocity.tolist(),
                    "size_m": track.size.tolist(),
                    "n_points": track.n_points,
                })

        if markers.markers:
            self._marker_pub.publish(markers)
        # Publish empty arrays too: consumers need an explicit "clear"
        # observation rather than having to guess whether silence means no
        # tracks or a dead perception node.
        self._tracks_pub.publish(tracks)
        sensor_position = self._odom_position()
        packet = {
            "schema_version": 1,
            "source_id": self._dog_id,
            "session_id": self._session_id,
            "sequence": self._frame_count,
            "stamp_s": stamp,
            "frame_id": self._frame_id,
            "sensor_position_m": (
                sensor_position.tolist()
                if sensor_position is not None else [0.0, 0.0, 0.0]
            ),
            "tracks": rich_tracks,
        }
        self._track_observations_pub.publish(
            String(data=json.dumps(packet, separators=(",", ":"))))

    def _build_markers(self, track_id, track, speed, is_coasting, is_reidentified):
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
        # Re-identification (see reid_max_distance/reid_window_seconds in
        # the module docstring) is visually distinguished from an ordinary
        # coast-then-match the same way coasting itself is faded out --
        # this is the one frame where it's useful to see that a track's ID
        # just survived a gap much longer than a normal coast, not just
        # that it matched.
        suffix = " (coasting)" if is_coasting else " (re-identified)" if is_reidentified else ""
        label.text = f"track {track_id}: {speed:.2f} m/s{suffix}"
        label.lifetime.sec = 1

        return [sphere, label]


def main():
    rclpy.init()
    node = OnlinePerceptionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
