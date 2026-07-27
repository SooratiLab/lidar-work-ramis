import numpy as np

from occlusion_accumulation import (
    OcclusionAccumulator,
    complete_small_gaps,
)


def test_complete_small_gaps_interpolates_short_consistent_gap():
    image = np.array([[5.0, np.inf, np.inf, 5.2]])

    completed = complete_small_gaps(
        image, max_gap_bins=2, max_range_difference=0.3)

    np.testing.assert_allclose(
        completed, [[5.0, 5.0666667, 5.1333333, 5.2]], atol=1e-6)


def test_complete_small_gaps_uses_farther_surface_across_discontinuity():
    image = np.array([[2.0, np.inf, 8.0]])

    completed = complete_small_gaps(
        image, max_gap_bins=1, max_range_difference=0.3)

    assert completed[0, 1] == 8.0


def test_complete_small_gaps_does_not_cascade_across_large_gap():
    image = np.array([[5.0, np.inf, np.inf, np.inf, 5.0]])

    completed = complete_small_gaps(
        image, max_gap_bins=2, max_range_difference=0.3)

    assert np.all(np.isinf(completed[0, 1:4]))


def test_accumulator_requires_temporal_evidence_before_activation():
    accumulator = OcclusionAccumulator(
        azimuth_bins=72,
        elevation_bins=36,
        max_gap_bins=0,
        contribution_floor=0.01,
        contribution_range_ratio=0.0,
        activation_threshold=0.5,
        activation_range_ratio=0.0,
    )
    origin = np.zeros(3)

    first = accumulator.update(np.array([[5.0, 0.0, 0.0]]), origin)
    second = accumulator.update(np.array([[4.7, 0.0, 0.0]]), origin)
    third = accumulator.update(np.array([[4.4, 0.0, 0.0]]), origin)

    assert len(first.moved_points) == 0
    assert len(second.moved_points) == 0
    assert len(third.moved_points) == 1
    assert third.n_active_bins == 1


def test_accumulator_does_not_flag_static_surface():
    accumulator = OcclusionAccumulator(
        azimuth_bins=72, elevation_bins=36, max_gap_bins=0)
    points = np.array([[5.0, 0.0, 0.0], [5.0, 1.0, 0.0]])

    accumulator.update(points, np.zeros(3))
    result = accumulator.update(points, np.zeros(3))

    assert len(result.moved_points) == 0
    assert result.n_positive_bins == 0


def test_accumulator_reprojects_static_world_when_sensor_moves():
    accumulator = OcclusionAccumulator(
        azimuth_bins=360,
        elevation_bins=90,
        max_gap_bins=0,
        contribution_floor=0.01,
        contribution_range_ratio=0.0,
    )
    static_world = np.array([[5.0, 0.0, 0.0], [5.0, 1.0, 0.0]])

    accumulator.update(static_world, np.zeros(3))
    result = accumulator.update(
        static_world, np.array([0.5, 0.0, 0.0]))

    assert len(result.moved_points) == 0
    assert result.n_positive_bins == 0


def test_background_reappearance_clears_accumulated_occlusion():
    accumulator = OcclusionAccumulator(
        azimuth_bins=72,
        elevation_bins=36,
        max_gap_bins=0,
        contribution_floor=0.01,
        contribution_range_ratio=0.0,
        activation_threshold=0.2,
        activation_range_ratio=0.0,
        reappearance_floor=0.01,
        reappearance_range_ratio=0.0,
    )
    origin = np.zeros(3)

    accumulator.update(np.array([[5.0, 0.0, 0.0]]), origin)
    active = accumulator.update(np.array([[4.0, 0.0, 0.0]]), origin)
    cleared = accumulator.update(np.array([[5.0, 0.0, 0.0]]), origin)

    assert len(active.moved_points) == 1
    assert cleared.n_reappeared_bins == 1
    assert len(cleared.moved_points) == 0
    assert cleared.n_active_bins == 0


def test_reset_forgets_previous_frame_and_evidence():
    accumulator = OcclusionAccumulator(
        azimuth_bins=72,
        elevation_bins=36,
        max_gap_bins=0,
        contribution_floor=0.01,
        contribution_range_ratio=0.0,
        activation_threshold=0.2,
        activation_range_ratio=0.0,
    )
    origin = np.zeros(3)
    accumulator.update(np.array([[5.0, 0.0, 0.0]]), origin)
    accumulator.update(np.array([[4.0, 0.0, 0.0]]), origin)

    accumulator.reset()
    after_reset = accumulator.update(np.array([[3.0, 0.0, 0.0]]), origin)

    assert len(after_reset.moved_points) == 0
    assert after_reset.n_active_bins == 0
