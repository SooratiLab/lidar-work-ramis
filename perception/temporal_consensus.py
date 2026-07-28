"""Multi-frame nearest-neighbor consensus for moving-point candidates.

The established detector compares the current registered cloud with only the
immediately previous frame. Solid-state LiDAR sampling can make a static
surface disappear for one scan and reappear in the next, so a single missing
match is weak evidence of motion.

DOF-LIO addresses this in range-image space by voting over a sliding window of
pose-aligned historical scans. PGP-DOR similarly compares each point with
several temporal neighbors before its grid-level inference. This module adapts
their shared temporal idea to the representation already used here: world-frame
nearest-neighbor change tests. The pipeline first applies its established
previous-frame test, then uses this vote only to reject weak candidates. Older
history can never introduce a point that the established detector accepted as
static.

This is not a reproduction of either paper. It retains this project's
Euclidean change threshold and separate visibility gate, avoiding their
road-terrain and LiDAR-organization assumptions.
"""
from dataclasses import dataclass
from math import ceil

import numpy as np
from scipy.spatial import cKDTree


@dataclass
class TemporalConsensusResult:
    """Candidate mask and the vote threshold used for one frame."""

    changed_mask: np.ndarray
    histories_used: int
    required_changed_histories: int


def changed_in_history_mask(
    points: np.ndarray,
    history_frames,
    distance_threshold: float,
    min_changed_ratio: float = 0.5,
) -> TemporalConsensusResult:
    """Return points changed relative to enough historical frames.

    ``points`` and every history frame contain world-frame XYZ metres.
    ``min_changed_ratio`` is in ``(0, 1]``; the required number of changed
    votes is ``ceil(ratio * available histories)``. Empty history frames are
    ignored because they contain no evidence about visibility or occupancy.
    """
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1:] != (3,):
        raise ValueError("points must have shape (N, 3)")
    if distance_threshold < 0:
        raise ValueError("distance_threshold must be non-negative")
    if not 0 < min_changed_ratio <= 1:
        raise ValueError("min_changed_ratio must be in (0, 1]")
    if not np.all(np.isfinite(points)):
        raise ValueError("points must be finite")

    usable_history = []
    for frame in history_frames:
        frame = np.asarray(frame, dtype=float)
        if frame.ndim != 2 or frame.shape[1:] != (3,):
            raise ValueError("history frames must have shape (N, 3)")
        if not np.all(np.isfinite(frame)):
            raise ValueError("history frames must be finite")
        if len(frame):
            usable_history.append(frame)

    histories_used = len(usable_history)
    if histories_used == 0 or len(points) == 0:
        return TemporalConsensusResult(
            changed_mask=np.zeros(len(points), dtype=bool),
            histories_used=histories_used,
            required_changed_histories=0,
        )

    changed_votes = np.zeros(len(points), dtype=np.int32)
    for frame in usable_history:
        distances, _ = cKDTree(frame).query(points, k=1)
        changed_votes += distances > distance_threshold

    required = int(ceil(min_changed_ratio * histories_used))
    return TemporalConsensusResult(
        changed_mask=changed_votes >= required,
        histories_used=histories_used,
        required_changed_histories=required,
    )
