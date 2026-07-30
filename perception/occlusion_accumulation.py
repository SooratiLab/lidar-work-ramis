"""Experimental temporal moving-point detector in the range-image domain.

This module adapts the occlusion-accumulation idea described by Kim et al.,
"Real-Time Moving Object Detection for 3-D LiDAR Using Occlusion
Accumulation in Range Image" (IEEE Transactions on Instrumentation and
Measurement, 2025). It is a project-specific adaptation, not a reproduction
of the paper's full pipeline.

Inputs are FastLIO-registered world-frame points and the corresponding sensor
position, all in metres. The previous cloud is projected from the current
sensor origin, which is the world-frame equivalent of warping a sensor-frame
range image with the relative pose. Accumulated evidence is carried by its
nearest observed world point and reprojected in the same way.

This remains an experimental alternative to
``range_image.previously_visible_mask``. It is available offline and behind a
default-off live-node flag for controlled tests. The paper's
range-proportional thresholds were tuned on automotive spinning-LiDAR
datasets; the defaults here retain its alpha/beta ratios as starting values
while adding explicit metre floors for this project's sparse
Mid-360/FastLIO recordings.
"""
from dataclasses import dataclass

import numpy as np

from range_image import _bin_indices, points_to_spherical


def _validate_image(image):
    image = np.asarray(image, dtype=float)
    if image.ndim != 2:
        raise ValueError("range image must be a 2-D array")
    return image


def _complete_axis(source, axis, max_gap_bins, max_range_difference, wrap):
    """Fill bounded runs of empty pixels along one image axis.

    A short gap is linearly interpolated when its two boundary ranges agree.
    If they disagree, the farther boundary is used: inventing a nearer
    surface would manufacture positive occlusion evidence. The function reads
    only from ``source`` while filling, so one newly-filled pixel cannot
    cascade across an arbitrarily large empty region.
    """
    result = source.copy()
    rows, columns = source.shape
    outer_size = rows if axis == 1 else columns
    inner_size = columns if axis == 1 else rows

    def value(outer, inner):
        return source[outer, inner] if axis == 1 else source[inner, outer]

    def assign(outer, inner, new_value):
        if axis == 1:
            result[outer, inner] = new_value
        else:
            result[inner, outer] = new_value

    for outer in range(outer_size):
        for inner in range(inner_size):
            if np.isfinite(value(outer, inner)):
                continue

            left = right = None
            for distance in range(1, max_gap_bins + 2):
                candidate = inner - distance
                if wrap:
                    candidate %= inner_size
                elif candidate < 0:
                    break
                if np.isfinite(value(outer, candidate)):
                    left = (candidate, distance, value(outer, candidate))
                    break
            for distance in range(1, max_gap_bins + 2):
                candidate = inner + distance
                if wrap:
                    candidate %= inner_size
                elif candidate >= inner_size:
                    break
                if np.isfinite(value(outer, candidate)):
                    right = (candidate, distance, value(outer, candidate))
                    break

            if left is None or right is None:
                continue
            gap_length = left[1] + right[1] - 1
            if gap_length > max_gap_bins:
                continue

            if abs(left[2] - right[2]) <= max_range_difference:
                # Inverse-distance interpolation reduces to this weighted
                # average between the two boundaries.
                new_value = (
                    left[2] * right[1] + right[2] * left[1]
                ) / (left[1] + right[1])
            else:
                new_value = max(left[2], right[2])
            assign(outer, inner, new_value)
    return result


def complete_small_gaps(
    image: np.ndarray,
    max_gap_bins: int = 2,
    max_range_difference: float = 0.3,
) -> np.ndarray:
    """Conservatively fill one- or two-pixel holes in a range image.

    Horizontal completion runs first and wraps at the 360-degree azimuth
    seam. Vertical completion does not wrap and allows twice the range
    difference, following the paper's treatment of adjacent scan rows.
    ``np.inf`` denotes empty bins and remains for gaps that are too large or
    do not have boundaries on both sides.
    """
    image = _validate_image(image)
    if max_gap_bins < 0:
        raise ValueError("max_gap_bins must be non-negative")
    if max_range_difference < 0:
        raise ValueError("max_range_difference must be non-negative")
    if max_gap_bins == 0 or image.size == 0:
        return image.copy()

    horizontal = _complete_axis(
        image,
        axis=1,
        max_gap_bins=max_gap_bins,
        max_range_difference=max_range_difference,
        wrap=True,
    )
    return _complete_axis(
        horizontal,
        axis=0,
        max_gap_bins=max_gap_bins,
        max_range_difference=2.0 * max_range_difference,
        wrap=False,
    )


def _range_image_with_nearest_points(points, origin, azimuth_bins, elevation_bins):
    """Return minimum range, nearest point, and per-point bin indices."""
    image = np.full((elevation_bins, azimuth_bins), np.inf)
    nearest_points = np.full((elevation_bins, azimuth_bins, 3), np.nan)
    if len(points) == 0:
        return (
            image,
            nearest_points,
            np.empty(0, dtype=np.int64),
            np.empty(0, dtype=np.int64),
            np.empty(0),
        )

    ranges, azimuth, elevation = points_to_spherical(points, origin)
    azimuth_indices, elevation_indices = _bin_indices(
        azimuth, elevation, azimuth_bins, elevation_bins)
    flat_indices = elevation_indices * azimuth_bins + azimuth_indices

    # Sort near-to-far, then keep the first point for each flat pixel index.
    order = np.argsort(ranges)
    sorted_pixels = flat_indices[order]
    _, first = np.unique(sorted_pixels, return_index=True)
    nearest_point_indices = order[first]
    nearest_azimuth = azimuth_indices[nearest_point_indices]
    nearest_elevation = elevation_indices[nearest_point_indices]
    image[nearest_elevation, nearest_azimuth] = ranges[nearest_point_indices]
    nearest_points[nearest_elevation, nearest_azimuth] = points[
        nearest_point_indices]
    return image, nearest_points, azimuth_indices, elevation_indices, ranges


@dataclass
class OcclusionResult:
    """One accumulator update, with points and diagnostics in metres."""

    moved_points: np.ndarray
    accumulated_occlusion_m: np.ndarray
    range_difference_m: np.ndarray
    n_positive_bins: int
    n_reappeared_bins: int
    n_active_bins: int


class OcclusionAccumulator:
    """Accumulate pose-compensated positive occlusion across point clouds."""

    def __init__(
        self,
        azimuth_bins: int = 72,
        elevation_bins: int = 36,
        max_gap_bins: int = 1,
        completion_range_difference: float = 0.3,
        reappearance_floor: float = 0.10,
        reappearance_range_ratio: float = 0.10,
        activation_threshold: float = 0.30,
        activation_range_ratio: float = 0.30,
        point_depth_tolerance: float = 0.50,
        max_accumulation: float = 5.0,
    ):
        if azimuth_bins < 1 or elevation_bins < 1:
            raise ValueError("range-image dimensions must be positive")
        nonnegative = {
            "max_gap_bins": max_gap_bins,
            "completion_range_difference": completion_range_difference,
            "reappearance_floor": reappearance_floor,
            "reappearance_range_ratio": reappearance_range_ratio,
            "activation_threshold": activation_threshold,
            "activation_range_ratio": activation_range_ratio,
            "point_depth_tolerance": point_depth_tolerance,
            "max_accumulation": max_accumulation,
        }
        if any(value < 0 for value in nonnegative.values()):
            raise ValueError("occlusion accumulator parameters must be non-negative")

        self.azimuth_bins = azimuth_bins
        self.elevation_bins = elevation_bins
        self.max_gap_bins = int(max_gap_bins)
        self.completion_range_difference = completion_range_difference
        self.reappearance_floor = reappearance_floor
        self.reappearance_range_ratio = reappearance_range_ratio
        self.activation_threshold = activation_threshold
        self.activation_range_ratio = activation_range_ratio
        self.point_depth_tolerance = point_depth_tolerance
        self.max_accumulation = max_accumulation

        shape = (elevation_bins, azimuth_bins)
        self._accumulation = np.zeros(shape)
        self._support_points = np.full((*shape, 3), np.nan)
        self._previous_points = None

    def reset(self):
        """Forget all temporal evidence, e.g. after a clock/pose reset."""
        self._accumulation.fill(0.0)
        self._support_points.fill(np.nan)
        self._previous_points = None

    def _warp_accumulation(self, current_origin):
        warped = np.zeros_like(self._accumulation)
        valid = (
            (self._accumulation > 0)
            & np.all(np.isfinite(self._support_points), axis=2)
        )
        if not np.any(valid):
            return warped

        support = self._support_points[valid]
        values = self._accumulation[valid]
        _, azimuth, elevation = points_to_spherical(support, current_origin)
        azimuth_indices, elevation_indices = _bin_indices(
            azimuth,
            elevation,
            self.azimuth_bins,
            self.elevation_bins,
        )
        # Several old pixels can project into one new pixel. Keeping the
        # strongest evidence avoids adding the same old observation twice.
        np.maximum.at(warped, (elevation_indices, azimuth_indices), values)
        return warped

    def update(self, points: np.ndarray, sensor_position: np.ndarray) -> OcclusionResult:
        """Process one registered cloud and return currently active points.

        ``points`` are ``(N, 3)`` world-frame metres. ``sensor_position`` is
        a world-frame ``(3,)`` vector in metres. The first update initializes
        history and intentionally returns no motion.
        """
        points = np.asarray(points, dtype=float)
        sensor_position = np.asarray(sensor_position, dtype=float).reshape(3)
        if points.ndim != 2 or points.shape[1:] != (3,):
            raise ValueError("points must have shape (N, 3)")
        if not np.all(np.isfinite(points)) or not np.all(np.isfinite(sensor_position)):
            raise ValueError("points and sensor position must be finite")

        (
            current_raw,
            current_nearest_points,
            point_azimuth,
            point_elevation,
            point_ranges,
        ) = _range_image_with_nearest_points(
            points,
            sensor_position,
            self.azimuth_bins,
            self.elevation_bins,
        )

        if self._previous_points is None:
            self._previous_points = points.copy()
            self._support_points = current_nearest_points
            empty = np.zeros_like(self._accumulation)
            return OcclusionResult(
                moved_points=np.empty((0, 3)),
                accumulated_occlusion_m=empty.copy(),
                range_difference_m=empty,
                n_positive_bins=0,
                n_reappeared_bins=0,
                n_active_bins=0,
            )

        previous_raw, _, _, _, _ = _range_image_with_nearest_points(
            self._previous_points,
            sensor_position,
            self.azimuth_bins,
            self.elevation_bins,
        )
        current = complete_small_gaps(
            current_raw,
            self.max_gap_bins,
            self.completion_range_difference,
        )
        previous = complete_small_gaps(
            previous_raw,
            self.max_gap_bins,
            self.completion_range_difference,
        )

        common = np.isfinite(current) & np.isfinite(previous)
        range_difference = np.zeros_like(self._accumulation)
        range_difference[common] = previous[common] - current[common]

        reappearance_threshold = np.maximum(
            self.reappearance_floor,
            self.reappearance_range_ratio * np.where(
                np.isfinite(current), current, 0.0),
        )
        current_observed = np.isfinite(current_raw)
        positive = common & current_observed & (range_difference > 0.0)
        reappeared = common & (range_difference < -reappearance_threshold)

        accumulation = self._warp_accumulation(sensor_position)
        measured = common & current_observed
        accumulation[measured] += range_difference[measured]
        # Evidence without a current measured support point cannot be safely
        # carried into another frame. Reappearance is stronger negative
        # evidence and explicitly clears the old occlusion.
        accumulation[~current_observed | ~common | reappeared] = 0.0

        # Equation (8) in Kim et al. truncates weak accumulated evidence on
        # every sequence. This ordering matters: retaining sub-threshold
        # changes lets small pose and sampling errors build indefinitely
        # until they eventually look like motion, which is not the published
        # method. Evidence persists only after a single update has crossed
        # the range-proportional threshold; subsequent signed differences
        # can then strengthen or weaken it.
        activation_threshold = np.maximum(
            self.activation_threshold,
            self.activation_range_ratio * np.where(
                np.isfinite(current_raw), current_raw, 0.0),
        )
        accumulation[accumulation <= activation_threshold] = 0.0
        np.clip(accumulation, 0.0, self.max_accumulation, out=accumulation)
        active_bins = accumulation > 0.0
        if len(points):
            nearest_ranges = current_raw[point_elevation, point_azimuth]
            on_nearest_surface = (
                point_ranges <= nearest_ranges + self.point_depth_tolerance)
            moved_mask = active_bins[point_elevation, point_azimuth] & on_nearest_surface
            moved_points = points[moved_mask]
        else:
            moved_points = np.empty((0, 3))

        self._accumulation = accumulation
        self._support_points = current_nearest_points
        self._previous_points = points.copy()
        return OcclusionResult(
            moved_points=moved_points,
            accumulated_occlusion_m=accumulation.copy(),
            range_difference_m=range_difference,
            n_positive_bins=int(positive.sum()),
            n_reappeared_bins=int(reappeared.sum()),
            n_active_bins=int(active_bins.sum()),
        )
