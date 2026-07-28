"""Conservative free-space intrusion detection for registered LiDAR scans.

Dynablox detects motion when a return occupies space that the map previously
established as free. Its full implementation extends a TSDF in Voxblox. This
module keeps only the part needed to screen that cue in this project's Python
pipeline: rays from pose-aligned scans build a sparse voxel map, repeated free
observations make a voxel trustworthy, and later returns in or next to those
voxels are emitted as intrusion candidates. The pipeline intersects these with
its established nearest-neighbor and visibility tests; the sparse map is an
additional temporal gate, not a replacement for a TSDF.

This is an adapted experiment, not a reimplementation of Dynablox. In
particular, it does not construct a TSDF, and repeated direct ray traversals
stand in for Dynablox's fused distance and occupancy estimates. The conservative
parts of the published method are retained:

* unknown space never counts as free;
* free-space evidence needs several distinct frames;
* spatial support from neighboring voxels can be required;
* short gaps in observations are tolerated for irregular/sparse LiDAR scans;
* persistent occupancy removes stale free-space labels to tolerate drift and
  genuine changes in the static scene; and
* detection uses the map *before* integrating the current scan.

Points and sensor positions are world-frame metres. One update must represent
one scan from one sensor pose; aggregating scans acquired at several poses
would cast physically incorrect rays from a single origin.
"""
from dataclasses import dataclass

import numpy as np


_AXIS_NEIGHBORS = np.array([
    (-1, 0, 0),
    (1, 0, 0),
    (0, -1, 0),
    (0, 1, 0),
    (0, 0, -1),
    (0, 0, 1),
], dtype=np.int64)

_FULL_NEIGHBORS = np.array([
    (x, y, z)
    for x in (-1, 0, 1)
    for y in (-1, 0, 1)
    for z in (-1, 0, 1)
    if (x, y, z) != (0, 0, 0)
], dtype=np.int64)


@dataclass
class FreeSpaceResult:
    """One detector update and its sparse-map diagnostics."""

    moved_points: np.ndarray
    n_free_ray_voxels: int
    n_ever_free_voxels: int
    n_reset_voxels: int


class EverFreeDetector:
    """Detect returns entering conservatively established free-space voxels."""

    def __init__(
        self,
        voxel_size: float = 0.20,
        min_range: float = 0.5,
        max_range: float = 20.0,
        burn_in_observations: int = 5,
        temporal_buffer_frames: int = 2,
        reset_after_occupied_frames: int = 150,
        neighbor_connectivity: int = 6,
        ray_step_ratio: float = 0.75,
        surface_margin_voxels: float = 1.0,
    ):
        if voxel_size <= 0:
            raise ValueError("voxel_size must be positive")
        if min_range < 0 or max_range <= min_range:
            raise ValueError("free-space range bounds are invalid")
        if burn_in_observations < 1:
            raise ValueError("burn_in_observations must be positive")
        if temporal_buffer_frames < 0:
            raise ValueError("temporal_buffer_frames must be non-negative")
        if reset_after_occupied_frames < 1:
            raise ValueError("reset_after_occupied_frames must be positive")
        if neighbor_connectivity not in (0, 6, 26):
            raise ValueError("neighbor_connectivity must be 0, 6, or 26")
        if ray_step_ratio <= 0 or ray_step_ratio > 1:
            raise ValueError("ray_step_ratio must be in (0, 1]")
        if surface_margin_voxels < 0:
            raise ValueError("surface_margin_voxels must be non-negative")

        self.voxel_size = float(voxel_size)
        self.min_range = float(min_range)
        self.max_range = float(max_range)
        self.burn_in_observations = int(burn_in_observations)
        self.temporal_buffer_frames = int(temporal_buffer_frames)
        self.reset_after_occupied_frames = int(
            reset_after_occupied_frames)
        self.neighbor_connectivity = int(neighbor_connectivity)
        self.ray_step = float(ray_step_ratio) * self.voxel_size
        self.surface_margin = float(surface_margin_voxels) * self.voxel_size

        if neighbor_connectivity == 6:
            self._neighbor_offsets = _AXIS_NEIGHBORS
        elif neighbor_connectivity == 26:
            self._neighbor_offsets = _FULL_NEIGHBORS
        else:
            self._neighbor_offsets = np.empty((0, 3), dtype=np.int64)
        self._neighbor_offset_keys = [
            tuple(offset) for offset in self._neighbor_offsets
        ]

        self._frame_index = -1
        self._free_observations = {}
        self._last_free_frame = {}
        self._ever_free = set()
        self._occupied_streak = {}
        self._last_occupied_frame = {}

    def reset(self):
        """Forget the map, for example after a timestamp or pose reset."""
        self._frame_index = -1
        self._free_observations.clear()
        self._last_free_frame.clear()
        self._ever_free.clear()
        self._occupied_streak.clear()
        self._last_occupied_frame.clear()

    def _voxel_indices(self, points: np.ndarray) -> np.ndarray:
        return np.floor(points / self.voxel_size).astype(np.int64)

    def _ray_voxels(
        self,
        points: np.ndarray,
        sensor_position: np.ndarray,
    ) -> np.ndarray:
        relative = points - sensor_position
        ranges = np.linalg.norm(relative, axis=1)
        valid = (
            np.isfinite(ranges)
            & (ranges >= self.min_range)
            & (ranges <= self.max_range)
            & (ranges > self.surface_margin)
        )
        if not np.any(valid):
            return np.empty((0, 3), dtype=np.int64)

        relative = relative[valid]
        ranges = ranges[valid]
        directions = relative / ranges[:, None]
        free_lengths = ranges - self.surface_margin
        step_counts = np.floor(free_lengths / self.ray_step).astype(np.int64)
        keep = step_counts > 0
        if not np.any(keep):
            return np.empty((0, 3), dtype=np.int64)

        directions = directions[keep]
        step_counts = step_counts[keep]
        total_steps = int(step_counts.sum())
        ray_starts = np.repeat(
            np.cumsum(step_counts) - step_counts, step_counts)
        within_ray_step = (
            np.arange(total_steps, dtype=np.int64) - ray_starts + 1)
        distances = within_ray_step * self.ray_step
        samples = (
            sensor_position
            + np.repeat(directions, step_counts, axis=0)
            * distances[:, None]
        )
        return np.unique(self._voxel_indices(samples), axis=0)

    def _has_spatial_support(self, key: tuple) -> bool:
        if self.neighbor_connectivity == 0:
            return True
        for offset in self._neighbor_offset_keys:
            neighbor = tuple(a + b for a, b in zip(key, offset))
            if self._free_observations.get(neighbor, 0) < (
                self.burn_in_observations
            ):
                return False
        return True

    def _is_dynamic_voxel(self, key: tuple) -> bool:
        if key in self._ever_free:
            return True
        return any(
            tuple(a + b for a, b in zip(key, offset)) in self._ever_free
            for offset in self._neighbor_offset_keys
        )

    def _update_persistent_occupancy(self, occupied_keys) -> int:
        reset_keys = set()
        for key in occupied_keys:
            previous = self._last_occupied_frame.get(key)
            if (
                previous is not None
                and previous >= (
                    self._frame_index - self.temporal_buffer_frames - 1)
            ):
                streak = self._occupied_streak.get(key, 0) + 1
            else:
                streak = 1
            self._occupied_streak[key] = streak
            self._last_occupied_frame[key] = self._frame_index

            if (
                streak < self.reset_after_occupied_frames
                or key not in self._ever_free
            ):
                continue
            reset_keys.add(key)
            reset_keys.update(
                tuple(a + b for a, b in zip(key, offset))
                for offset in self._neighbor_offset_keys
            )

        for key in reset_keys:
            self._ever_free.discard(key)
            self._free_observations.pop(key, None)
            self._last_free_frame.pop(key, None)
        return len(reset_keys)

    def _integrate_free_voxels(self, free_voxels: np.ndarray):
        newly_mature = []
        for voxel in free_voxels:
            key = tuple(voxel)
            if key in self._ever_free:
                continue
            if self._last_free_frame.get(key) == self._frame_index:
                continue
            count = self._free_observations.get(key, 0) + 1
            self._free_observations[key] = count
            self._last_free_frame[key] = self._frame_index
            if count == self.burn_in_observations:
                newly_mature.append(key)

        # A voxel can gain its final missing neighbor after it first crosses
        # the count threshold. Reconsider that voxel and the nearby voxels
        # for which it may have been the last missing support, rather than
        # rescanning the entire mature map on every frame.
        candidates = set(newly_mature)
        for key in newly_mature:
            candidates.update(
                tuple(a - b for a, b in zip(key, offset))
                for offset in self._neighbor_offset_keys
            )
        for key in candidates:
            if (
                self._free_observations.get(key, 0)
                >= self.burn_in_observations
                and self._has_spatial_support(key)
            ):
                self._ever_free.add(key)

    def update(
        self,
        points: np.ndarray,
        sensor_position: np.ndarray,
    ) -> FreeSpaceResult:
        """Process one registered scan and return free-space intrusions.

        The current returns are classified against prior state first. Their
        occupied and free-space evidence is integrated only afterwards, so a
        scan cannot create the free label that triggers its own detection.
        """
        points = np.asarray(points, dtype=float)
        sensor_position = np.asarray(sensor_position, dtype=float).reshape(3)
        if points.ndim != 2 or points.shape[1:] != (3,):
            raise ValueError("points must have shape (N, 3)")
        if (
            not np.all(np.isfinite(points))
            or not np.all(np.isfinite(sensor_position))
        ):
            raise ValueError("points and sensor position must be finite")

        self._frame_index += 1
        if len(points):
            point_voxels = self._voxel_indices(points)
            moved_mask = np.fromiter(
                (
                    self._is_dynamic_voxel(tuple(voxel))
                    for voxel in point_voxels
                ),
                dtype=bool,
                count=len(point_voxels),
            )
            occupied_keys = set(map(tuple, point_voxels))
        else:
            moved_mask = np.zeros(0, dtype=bool)
            occupied_keys = set()

        n_reset = self._update_persistent_occupancy(occupied_keys)
        free_voxels = self._ray_voxels(points, sensor_position)
        self._integrate_free_voxels(free_voxels)

        return FreeSpaceResult(
            moved_points=points[moved_mask],
            n_free_ray_voxels=len(free_voxels),
            n_ever_free_voxels=len(self._ever_free),
            n_reset_voxels=n_reset,
        )
