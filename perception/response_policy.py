"""Stateful near-cluster stop policy, independent of ROS.

Positions are metres in one shared frame. The policy deliberately produces a
stop *request*, not a Unitree command: robot-specific actuation remains a
separate, explicitly enabled boundary.
"""
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ResponseDecision:
    stop_requested: bool
    nearest_distance: float | None
    changed: bool


class NearClusterStopPolicy:
    """Request a stop after repeated nearby observations, with hysteresis."""

    def __init__(self, stop_distance: float = 2.0, clear_distance: float = 2.5,
                 trigger_frames: int = 2, clear_frames: int = 2):
        if stop_distance <= 0:
            raise ValueError("stop_distance must be positive")
        if clear_distance < stop_distance:
            raise ValueError("clear_distance must be >= stop_distance")
        if trigger_frames < 1 or clear_frames < 1:
            raise ValueError("frame counts must be at least 1")
        self.stop_distance = stop_distance
        self.clear_distance = clear_distance
        self.trigger_frames = trigger_frames
        self.clear_frames = clear_frames
        self.stop_requested = False
        self._near_frames = 0
        self._clear_frames = 0

    def update(self, cluster_positions, sensor_position) -> ResponseDecision:
        clusters = np.asarray(cluster_positions, dtype=float).reshape(-1, 3)
        sensor = np.asarray(sensor_position, dtype=float).reshape(3)
        nearest = (float(np.linalg.norm(clusters - sensor, axis=1).min())
                   if len(clusters) else None)
        previous = self.stop_requested

        if not self.stop_requested:
            self._near_frames = self._near_frames + 1 if (
                nearest is not None and nearest <= self.stop_distance) else 0
            if self._near_frames >= self.trigger_frames:
                self.stop_requested = True
                self._clear_frames = 0
        else:
            is_clear = nearest is None or nearest >= self.clear_distance
            self._clear_frames = self._clear_frames + 1 if is_clear else 0
            if self._clear_frames >= self.clear_frames:
                self.stop_requested = False
                self._near_frames = 0

        return ResponseDecision(
            self.stop_requested, nearest, self.stop_requested != previous)
