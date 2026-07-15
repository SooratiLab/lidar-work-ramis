"""
Unit tests for range_image.py -- the odometry-referenced visibility check
that suppresses "moved" false positives caused by a moving sensor bringing
new geometry into view, rather than something actually moving.

Plain numpy inputs throughout, no rclpy/ROS dependency needed.
"""
import numpy as np

from range_image import build_range_image, points_to_spherical, previously_visible_mask


# --- points_to_spherical ----------------------------------------------------

def test_points_to_spherical_straight_ahead():
    points = np.array([[5.0, 0.0, 0.0]])
    origin = np.array([0.0, 0.0, 0.0])

    ranges, azimuth, elevation = points_to_spherical(points, origin)

    assert ranges[0] == 5.0
    assert azimuth[0] == 0.0
    assert elevation[0] == 0.0


def test_points_to_spherical_is_relative_to_origin_not_world_origin():
    # A point 5 m from a sensor that has itself moved 100 m along x should
    # report range 5, not ~105 -- this is the whole reason the sensor's
    # own position has to be threaded through here rather than binning
    # against the world origin.
    points = np.array([[105.0, 0.0, 0.0]])
    origin = np.array([100.0, 0.0, 0.0])

    ranges, azimuth, elevation = points_to_spherical(points, origin)

    assert ranges[0] == 5.0


# --- build_range_image -------------------------------------------------------

def test_build_range_image_records_minimum_range_per_direction():
    origin = np.array([0.0, 0.0, 0.0])
    # Two points in the same direction (straight ahead), different ranges.
    points = np.array([[5.0, 0.0, 0.0], [3.0, 0.0, 0.0]])

    image = build_range_image(points, origin, azimuth_bins=180, elevation_bins=90)

    ranges, azimuth, elevation = points_to_spherical(points, origin)
    from range_image import _bin_indices
    az_idx, el_idx = _bin_indices(azimuth, elevation, 180, 90)
    assert image[el_idx[0], az_idx[0]] == 3.0  # the nearer of the two


def test_build_range_image_empty_input_is_all_inf():
    image = build_range_image(np.empty((0, 3)), np.zeros(3),
                               azimuth_bins=180, elevation_bins=90)
    assert np.all(np.isinf(image))


def test_build_range_image_unvisited_directions_stay_inf():
    origin = np.array([0.0, 0.0, 0.0])
    points = np.array([[5.0, 0.0, 0.0]])  # only ever looks "ahead"

    image = build_range_image(points, origin, azimuth_bins=180, elevation_bins=90)

    # Directly behind the sensor was never returned by this point set.
    ranges, azimuth, elevation = points_to_spherical(np.array([[-5.0, 0.0, 0.0]]), origin)
    from range_image import _bin_indices
    az_idx, el_idx = _bin_indices(azimuth, elevation, 180, 90)
    assert np.isinf(image[el_idx[0], az_idx[0]])


# --- previously_visible_mask -------------------------------------------------

def test_previously_visible_mask_flags_point_closer_than_previous_surface():
    # Previous scan saw a wall 5 m ahead; this frame has a point 2 m ahead
    # in the same direction -- something is now blocking that line of
    # sight, the actual signature of motion this gate is meant to keep.
    origin = np.array([0.0, 0.0, 0.0])
    prev_points = np.array([[5.0, 0.0, 0.0]])
    image = build_range_image(prev_points, origin, azimuth_bins=180, elevation_bins=90)

    candidates = np.array([[2.0, 0.0, 0.0]])
    mask = previously_visible_mask(candidates, origin, image, 180, 90, tolerance=0.3)

    assert bool(mask[0]) is True


def test_previously_visible_mask_drops_point_in_previously_unseen_direction():
    # The previous scan never returned anything behind the sensor -- a
    # point suddenly appearing there is a viewpoint change (or a fresh
    # blind-spot return), not evidence of motion, and must not be flagged.
    origin = np.array([0.0, 0.0, 0.0])
    prev_points = np.array([[5.0, 0.0, 0.0]])  # only "ahead"
    image = build_range_image(prev_points, origin, azimuth_bins=180, elevation_bins=90)

    candidates = np.array([[-3.0, 0.0, 0.0]])  # "behind"
    mask = previously_visible_mask(candidates, origin, image, 180, 90, tolerance=0.3)

    assert bool(mask[0]) is False


def test_previously_visible_mask_drops_point_at_same_range_as_before():
    # Same direction, same range as the previous scan's surface, within
    # tolerance -- this is the same static background, not something new.
    origin = np.array([0.0, 0.0, 0.0])
    prev_points = np.array([[5.0, 0.0, 0.0]])
    image = build_range_image(prev_points, origin, azimuth_bins=180, elevation_bins=90)

    candidates = np.array([[4.95, 0.0, 0.0]])
    mask = previously_visible_mask(candidates, origin, image, 180, 90, tolerance=0.3)

    assert bool(mask[0]) is False


def test_previously_visible_mask_drops_point_farther_than_before():
    # Same direction, farther than the previous surface -- background
    # revealed at greater range (e.g. an occluder left, or the sensor
    # rounded a corner), deliberately not treated as motion. This is the
    # exact false-positive pattern a moving sensor produces.
    origin = np.array([0.0, 0.0, 0.0])
    prev_points = np.array([[5.0, 0.0, 0.0]])
    image = build_range_image(prev_points, origin, azimuth_bins=180, elevation_bins=90)

    candidates = np.array([[8.0, 0.0, 0.0]])
    mask = previously_visible_mask(candidates, origin, image, 180, 90, tolerance=0.3)

    assert bool(mask[0]) is False


def test_previously_visible_mask_respects_tolerance_boundary():
    origin = np.array([0.0, 0.0, 0.0])
    prev_points = np.array([[5.0, 0.0, 0.0]])
    image = build_range_image(prev_points, origin, azimuth_bins=180, elevation_bins=90)

    # Just inside tolerance -- not flagged.
    just_inside = np.array([[4.75, 0.0, 0.0]])
    assert bool(previously_visible_mask(just_inside, origin, image, 180, 90, tolerance=0.3)[0]) is False

    # Clearly beyond tolerance -- flagged.
    clearly_closer = np.array([[4.0, 0.0, 0.0]])
    assert bool(previously_visible_mask(clearly_closer, origin, image, 180, 90, tolerance=0.3)[0]) is True


def test_previously_visible_mask_empty_input():
    image = build_range_image(np.empty((0, 3)), np.zeros(3), 180, 90)
    mask = previously_visible_mask(np.empty((0, 3)), np.zeros(3), image, 180, 90, tolerance=0.3)
    assert len(mask) == 0
