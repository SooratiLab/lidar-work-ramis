import numpy as np
import pytest

from free_space import EverFreeDetector


def _wall(x, y_values=(-0.4, 0.0, 0.4), z_values=(-0.4, 0.0, 0.4)):
    return np.array([
        (x, y, z)
        for y in y_values
        for z in z_values
    ], dtype=float)


def test_unknown_space_is_not_motion():
    detector = EverFreeDetector(
        voxel_size=0.25,
        burn_in_observations=2,
        neighbor_connectivity=0,
    )
    origin = np.zeros(3)
    background = _wall(3.0)
    detector.update(background, origin)
    detector.update(background, origin)

    # The established rays point around +x. A return along +y lies in space
    # the map has never observed and therefore must not be called moving.
    result = detector.update(np.array([[0.0, 2.0, 0.0]]), origin)

    assert len(result.moved_points) == 0


def test_return_entering_established_free_space_is_motion():
    detector = EverFreeDetector(
        voxel_size=0.25,
        burn_in_observations=2,
        neighbor_connectivity=0,
    )
    origin = np.zeros(3)
    background = _wall(3.0)

    assert len(detector.update(background, origin).moved_points) == 0
    assert len(detector.update(background, origin).moved_points) == 0
    result = detector.update(_wall(2.0), origin)

    assert len(result.moved_points) == 9
    assert result.n_ever_free_voxels > 0


def test_free_label_requires_repeated_observations():
    detector = EverFreeDetector(
        voxel_size=0.25,
        burn_in_observations=3,
        neighbor_connectivity=0,
    )
    origin = np.zeros(3)
    background = _wall(3.0)

    detector.update(background, origin)
    detector.update(background, origin)
    result = detector.update(_wall(2.0), origin)

    assert len(result.moved_points) == 0


def test_persistent_occupancy_removes_stale_free_label():
    detector = EverFreeDetector(
        voxel_size=0.25,
        burn_in_observations=1,
        reset_after_occupied_frames=2,
        neighbor_connectivity=0,
    )
    origin = np.zeros(3)
    detector.update(_wall(3.0), origin)

    first = detector.update(_wall(2.0), origin)
    second = detector.update(_wall(2.0), origin)
    third = detector.update(_wall(2.0), origin)

    assert len(first.moved_points) == 9
    assert len(second.moved_points) == 9
    assert len(third.moved_points) == 0
    assert second.n_reset_voxels > 0


def test_reset_forgets_free_space():
    detector = EverFreeDetector(
        burn_in_observations=1,
        neighbor_connectivity=0,
    )
    origin = np.zeros(3)
    detector.update(_wall(3.0), origin)
    detector.reset()

    result = detector.update(_wall(2.0), origin)

    assert len(result.moved_points) == 0


@pytest.mark.parametrize("connectivity", [-1, 1, 18])
def test_rejects_unsupported_neighbor_connectivity(connectivity):
    with pytest.raises(ValueError, match="neighbor_connectivity"):
        EverFreeDetector(neighbor_connectivity=connectivity)
