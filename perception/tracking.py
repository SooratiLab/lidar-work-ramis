"""
tracking.py -- clustering and frame-to-frame centroid tracking for the
online perception node.

This replaces the greedy nearest-centroid matching the offline pipeline
(kei-stuff/lidar-perception/scripts/track_motion.py) used, and that this
node originally ported unchanged, with:

  - globally-optimal assignment (Hungarian algorithm) between predicted
    track positions and this frame's detections, instead of matching
    whichever cluster happens to come first in array order. Greedy
    matching can give away a track's correct match to a different track
    that gets processed earlier, when two people are close together --
    the Hungarian algorithm considers all pairings at once and minimises
    total assignment distance.
  - a constant-velocity Kalman filter per track, instead of a raw
    finite-difference velocity from the last two centroids. This smooths
    out per-frame centroid jitter (DBSCAN's cluster mean moves around by
    a few centimetres between frames even for a standing person, purely
    from which points happened to be flagged as "moved") and gives a
    principled predicted position to match new detections against and to
    publish during a coast (see below).
  - track coasting: a track that goes unmatched for a frame (occlusion,
    the detector not firing, a gap in the point cloud) keeps predicting
    forward via the Kalman filter for a few frames before being dropped,
    instead of ending immediately and forcing a new track ID on
    reappearance.

Split out of online_perception_node.py so all of this is testable as plain
numpy/scipy/sklearn code, without needing rclpy or a running ROS graph.

Units: metres and seconds throughout, matching FastLIO's own topics.
"""
import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial.distance import cdist
from sklearn.cluster import DBSCAN


def cluster_moved_points(points: np.ndarray, eps: float, min_points: int) -> list:
    """
    DBSCAN-cluster points already flagged as "moved" by change detection.

    Returns a list of dicts, one per cluster (DBSCAN noise, label -1, is
    dropped): {'centroid': (3,) array (m), 'size': (3,) array, the
    axis-aligned bounding box extent (m), 'n_points': int}.
    """
    if len(points) < min_points:
        return []

    labels = DBSCAN(eps=eps, min_samples=min_points).fit_predict(points)

    clusters = []
    for label in set(labels):
        if label == -1:
            continue
        cluster_points = points[labels == label]
        clusters.append({
            "centroid": cluster_points.mean(axis=0),
            "size": cluster_points.max(axis=0) - cluster_points.min(axis=0),
            "n_points": int(len(cluster_points)),
        })
    return clusters


def filter_plausible_detections(clusters: list, sensor_position: np.ndarray,
                                 max_range: float) -> list:
    """
    Drop clusters implausibly far from the sensor's current position.

    Motivated by degraded/pre-fix recordings where FastLIO's registration
    diverged (scan-matching failing on a fast or jerky motion, or on
    already-corrupted LiDAR/IMU timestamps): /cloud_registered can end up
    containing points thousands of metres from anything real, which
    cluster and track exactly like a genuine detection with nothing in the
    pipeline to say otherwise. A Livox Mid-360 has a rated detection range
    on the order of tens of metres for typical (~10% reflectivity)
    targets like clothing -- max_range should be set a little above that,
    not at the sensor's absolute maximum spec range, since a "person"
    detection far beyond typical range is far more likely to be drifted
    odometry than a real long-range return.

    sensor_position: (3,) array, the sensor's current position in the same
    frame as the cluster centroids (i.e. FastLIO's /Odometry position,
    not the world origin -- distance from the origin isn't meaningful once
    the sensor has moved away from where it started).
    """
    return [c for c in clusters
            if np.linalg.norm(c["centroid"] - sensor_position) <= max_range]


def assign_detections(track_positions: np.ndarray, detection_positions: np.ndarray,
                       max_distance: float):
    """
    Globally-optimal assignment between predicted track positions and this
    frame's detection centroids, gated by max_distance (m).

    track_positions: (M, 3) array of predicted track positions.
    detection_positions: (N, 3) array of this frame's cluster centroids.

    Returns (matches, unmatched_tracks, unmatched_detections):
      matches: list of (track_index, detection_index) pairs, each within
               max_distance of each other.
      unmatched_tracks: list of track indices with no acceptable match.
      unmatched_detections: list of detection indices with no acceptable
               match (these start new tracks).

    linear_sum_assignment finds the assignment that minimises total
    distance across *all* pairs at once (unlike greedily matching each
    detection to its own nearest track in array order, which can lock in a
    suboptimal pairing when two tracks are close together). It doesn't
    accept a "no match" option directly, so every track is assigned to
    some detection by the solver if the matrix is square -- max_distance
    gating is applied afterwards, rejecting any assignment further apart
    than that and returning the track/detection to the unmatched lists.
    """
    n_tracks = len(track_positions)
    n_detections = len(detection_positions)
    if n_tracks == 0 or n_detections == 0:
        return [], list(range(n_tracks)), list(range(n_detections))

    cost = cdist(track_positions, detection_positions)
    track_idx, det_idx = linear_sum_assignment(cost)

    matches = []
    matched_tracks, matched_detections = set(), set()
    for t, d in zip(track_idx, det_idx):
        if cost[t, d] <= max_distance:
            matches.append((int(t), int(d)))
            matched_tracks.add(t)
            matched_detections.add(d)

    unmatched_tracks = [t for t in range(n_tracks) if t not in matched_tracks]
    unmatched_detections = [d for d in range(n_detections) if d not in matched_detections]
    return matches, unmatched_tracks, unmatched_detections


class KalmanTrack:
    """
    Constant-velocity Kalman filter for one tracked centroid.

    State x = [x, y, z, vx, vy, vz] (m, m/s). Only position is measured
    each frame (a DBSCAN cluster centroid) -- velocity is inferred purely
    from how the filter's own position estimate moves over time, which is
    what lets a track predict a sensible position while coasting through a
    missed detection instead of freezing in place.
    """

    def __init__(self, track_id: int, initial_position: np.ndarray,
                 position_variance: float, velocity_variance: float,
                 process_variance: float):
        self.track_id = track_id
        self.x = np.zeros(6)
        self.x[:3] = initial_position
        self.P = np.diag([position_variance] * 3 + [velocity_variance] * 3)

        # Measurement noise (m^2): how much we trust a single cluster
        # centroid as a position estimate.
        self._position_variance = position_variance
        # Process noise density (m/s^2)^2: how much unmodelled acceleration
        # (speeding up, slowing down, turning) we expect between frames.
        # Higher values make the filter trust new detections more than its
        # own constant-velocity prediction; lower values smooth harder but
        # react more slowly to genuine direction changes.
        self._process_variance = process_variance

        self.n_points = 0
        self.size = np.zeros(3)
        self.hits = 1            # frames this track has ever received a real detection
        self.missed = 0          # consecutive frames since the last real detection
        self.missed_seconds = 0.0  # real time elapsed since the last real detection

    @property
    def position(self) -> np.ndarray:
        return self.x[:3]

    @property
    def velocity(self) -> np.ndarray:
        return self.x[3:]

    def predict(self, dt: float) -> None:
        """Advance the state by dt seconds (dt >= 0) with no new measurement."""
        dt = max(dt, 0.0)
        F = np.eye(6)
        F[0, 3] = F[1, 4] = F[2, 5] = dt
        self.x = F @ self.x

        # Standard discretised constant-velocity process noise (see e.g.
        # Bar-Shalom, "Estimation with Applications to Tracking and
        # Navigation", ch. 6): each axis gets its own 2x2 block, off-diagonal
        # terms couple position and velocity noise from the same underlying
        # acceleration.
        dt2, dt3, dt4 = dt * dt, dt ** 3, dt ** 4
        q = self._process_variance
        block = np.array([[dt4 / 4, dt3 / 2], [dt3 / 2, dt2]]) * q
        Q = np.zeros((6, 6))
        for axis in range(3):
            idx = [axis, axis + 3]
            Q[np.ix_(idx, idx)] = block

        self.P = F @ self.P @ F.T + Q

    def update(self, measured_position: np.ndarray) -> None:
        """Incorporate a new detection centroid (m) into the state estimate."""
        H = np.zeros((3, 6))
        H[0, 0] = H[1, 1] = H[2, 2] = 1.0
        R = np.eye(3) * self._position_variance

        innovation = measured_position - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ innovation
        self.P = (np.eye(6) - K @ H) @ self.P

        self.missed = 0
        self.missed_seconds = 0.0
        self.hits += 1


class CentroidTracker:
    """
    Frame-to-frame centroid tracker.

    Each step: predicts every existing track forward with its Kalman
    filter, assigns this frame's cluster detections to tracks via
    globally-optimal matching gated by max_match_distance, updates matched
    tracks with their new measurement, starts new tracks for unmatched
    detections, and coasts unmatched tracks (keeps predicting, no
    measurement update) for up to max_missed_frames before dropping them --
    or up to max_missed_seconds of real elapsed time, whichever comes
    first. The frame-count limit alone isn't reliable on irregular data:
    testing against a session with a degraded, bursty scan rate (pre-DDS-
    fix recording conditions) found a track that survived a 42-second real
    gap because only 3 *frames* happened to occur in that stretch, then
    coasted its Kalman prediction across the whole gap and reported a
    position 15+ metres from anything real. A constant-velocity
    extrapolation is only trustworthy over the second or two coasting is
    meant to bridge, not over tens of seconds -- max_missed_seconds catches
    that case even when max_missed_frames hasn't been reached yet.

    Every track also carries an is_confirmed flag (hits >= min_hits).
    Testing against several recorded sessions showed that a large fraction
    of all tracks ever created are pure single-frame noise -- a DBSCAN
    cluster that clears eps/min_points by chance on one frame (typically
    from long-range LiDAR noise or, on a moving sensor, a static object
    newly entering the field of view looking like motion) and is never
    seen again. Every one of these got exactly one real detection, then
    coasted for max_missed_frames frames and disappeared -- never a second
    real detection. Requiring min_hits real detections before a track
    counts as "confirmed" filters this whole category out of what gets
    logged/published, at the cost of one extra frame of latency (with the
    default min_hits=2) before a genuinely new object gets reported.
    """

    def __init__(self, max_match_distance: float, max_missed_frames: int = 3,
                 max_missed_seconds: float = 3.0, min_hits: int = 2,
                 position_variance: float = 0.01, velocity_variance: float = 4.0,
                 process_variance: float = 1.0):
        self._max_match_distance = max_match_distance
        self._max_missed_frames = max_missed_frames
        self._max_missed_seconds = max_missed_seconds
        self._min_hits = min_hits
        self._position_variance = position_variance
        self._velocity_variance = velocity_variance
        self._process_variance = process_variance

        self.tracks = {}  # track_id -> KalmanTrack
        self._next_id = 0

    def step(self, detections: list, dt: float) -> dict:
        """
        Advance the tracker by one frame.

        detections: list of cluster dicts as returned by
                    cluster_moved_points (need at least 'centroid';
                    'n_points'/'size' are copied onto the track if present).
        dt: seconds since the previous frame's measurement, for the motion
            model's predict step.

        Returns {track_id: {'track': KalmanTrack, 'is_new': bool,
        'is_coasting': bool, 'is_confirmed': bool}} for every track active
        this frame -- matched, newly created, or still within its coasting
        window. is_new/is_coasting describe what happened to the track
        *this frame*; is_confirmed describes the track's status overall
        (hits >= min_hits) and is what callers should gate
        logging/publishing on to avoid reporting single-frame noise as if
        it were a real detection.
        """
        for track in self.tracks.values():
            track.predict(dt)

        track_ids = list(self.tracks.keys())
        track_positions = (np.array([self.tracks[tid].position for tid in track_ids])
                            if track_ids else np.empty((0, 3)))
        detection_positions = (np.array([d["centroid"] for d in detections])
                                if detections else np.empty((0, 3)))

        matches, unmatched_tracks, unmatched_detections = assign_detections(
            track_positions, detection_positions, self._max_match_distance)

        active = {}

        for t_idx, d_idx in matches:
            tid = track_ids[t_idx]
            track = self.tracks[tid]
            detection = detections[d_idx]
            track.update(detection["centroid"])
            track.n_points = detection.get("n_points", track.n_points)
            track.size = detection.get("size", track.size)
            active[tid] = {"track": track, "is_new": False, "is_coasting": False,
                            "is_confirmed": track.hits >= self._min_hits}

        for t_idx in unmatched_tracks:
            tid = track_ids[t_idx]
            track = self.tracks[tid]
            track.missed += 1
            track.missed_seconds += dt
            if (track.missed > self._max_missed_frames
                    or track.missed_seconds > self._max_missed_seconds):
                del self.tracks[tid]
                continue
            active[tid] = {"track": track, "is_new": False, "is_coasting": True,
                            "is_confirmed": track.hits >= self._min_hits}

        for d_idx in unmatched_detections:
            detection = detections[d_idx]
            tid = self._next_id
            self._next_id += 1
            track = KalmanTrack(tid, detection["centroid"], self._position_variance,
                                 self._velocity_variance, self._process_variance)
            track.n_points = detection.get("n_points", 0)
            track.size = detection.get("size", np.zeros(3))
            self.tracks[tid] = track
            active[tid] = {"track": track, "is_new": True, "is_coasting": False,
                            "is_confirmed": track.hits >= self._min_hits}

        return active
