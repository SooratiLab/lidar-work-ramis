"""Stateful near-cluster stop policy, independent of ROS.

Positions are metres in one shared frame. The policy deliberately produces a
stop *request*, not a Unitree command: robot-specific actuation remains a
separate, explicitly enabled boundary.
"""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class ResponseDecision:
    stop_requested: bool
    nearest_distance: float | None
    changed: bool
    state: str


class NearClusterStopPolicy:
    """Request a stop after a nearby observation persists for long enough.

    Durations are measured from message timestamps rather than frame counts.
    Recorded sessions have shown that accumulated-frame timing can become
    irregular when the LiDAR stream is degraded; two frames therefore do not
    reliably represent one fixed amount of real time.

    Distance is planar by default. A person's centroid height should not make
    an otherwise nearby obstacle appear farther away to a ground robot.
    """

    def __init__(
        self,
        stop_distance: float = 2.0,
        clear_distance: float = 2.5,
        trigger_duration: float = 1.0,
        clear_duration: float = 1.0,
        planar_distance: bool = True,
    ):
        if stop_distance <= 0:
            raise ValueError("stop_distance must be positive")
        if clear_distance < stop_distance:
            raise ValueError("clear_distance must be >= stop_distance")
        if trigger_duration < 0 or clear_duration < 0:
            raise ValueError("durations must be non-negative")
        self.stop_distance = stop_distance
        self.clear_distance = clear_distance
        self.trigger_duration = trigger_duration
        self.clear_duration = clear_duration
        self.planar_distance = planar_distance
        self.stop_requested = False
        self._near_since = None
        self._clear_since = None
        self._last_timestamp = None

    def reset(self) -> None:
        """Return to the initial clear state after a clock discontinuity."""
        self.stop_requested = False
        self._near_since = None
        self._clear_since = None
        self._last_timestamp = None

    def update(
        self, cluster_positions, sensor_position, timestamp: float
    ) -> ResponseDecision:
        if not math.isfinite(timestamp):
            raise ValueError("timestamp must be finite")
        clusters = np.asarray(cluster_positions, dtype=float).reshape(-1, 3)
        sensor = np.asarray(sensor_position, dtype=float).reshape(3)
        if not np.all(np.isfinite(clusters)) or not np.all(np.isfinite(sensor)):
            raise ValueError("positions must be finite")

        # Bag loops and /clock resets can move time backwards. Treat that as
        # a fresh observation sequence rather than accidentally satisfying a
        # dwell period with timestamps from two different playback epochs.
        if self._last_timestamp is not None and timestamp < self._last_timestamp:
            self.reset()
        self._last_timestamp = timestamp

        deltas = clusters - sensor
        if self.planar_distance:
            deltas = deltas[:, :2]
        nearest = (
            float(np.linalg.norm(deltas, axis=1).min()) if len(clusters) else None
        )
        previous = self.stop_requested

        if not self.stop_requested:
            if nearest is not None and nearest <= self.stop_distance:
                if self._near_since is None:
                    self._near_since = timestamp
            else:
                self._near_since = None
            if (
                self._near_since is not None
                and timestamp - self._near_since >= self.trigger_duration
            ):
                self.stop_requested = True
                self._clear_since = None
        else:
            is_clear = nearest is None or nearest >= self.clear_distance
            if is_clear:
                if self._clear_since is None:
                    self._clear_since = timestamp
            else:
                self._clear_since = None
            if (
                self._clear_since is not None
                and timestamp - self._clear_since >= self.clear_duration
            ):
                self.stop_requested = False
                self._near_since = None
                self._clear_since = None

        if self.stop_requested:
            state = "stop"
        elif self._near_since is not None:
            state = "pending_stop"
        else:
            state = "clear"

        return ResponseDecision(
            self.stop_requested,
            nearest,
            self.stop_requested != previous,
            state,
        )
