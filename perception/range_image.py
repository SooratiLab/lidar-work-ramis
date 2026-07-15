"""
range_image.py -- odometry-referenced visibility check for change detection
on a moving sensor.

online_perception_node.py flags a point as "moved" if nothing in the
previous frame's point cloud was near it (see its change_threshold
parameter). That test is only meaningful if the previous frame could have
seen that location at all. On a stationary sensor it always could have --
the whole scene stayed in view -- so "nothing was there before" reliably
means something moved. On a moving sensor it often couldn't have: the
sensor's own position changing between frames constantly brings new
geometry into view (a wall past a corner, a shelf beyond where the sensor
used to stand) that the previous frame never had a chance to observe,
and frame-to-frame nearest-neighbour distance can't tell that apart from
an object that actually moved there. This is the false-positive mechanism
Kei's handover and this project's own testing (see perception/README.md)
both identify as the main open problem for a walking, not stationary, dog.

The fix here doesn't model the Mid-360's mounting geometry or a fixed
field-of-view cone -- it reasons from what the previous frame actually
returned instead. Bin the previous frame's points into a spherical grid
(azimuth/elevation) around the previous frame's own sensor position and
keep the minimum range seen in each direction -- this is a range image,
the same representation used for LiDAR occlusion/dynamic-object reasoning
in the wider literature (e.g. Removert, ERASOR). A direction with no entry
is empirically a direction the previous scan didn't reach, whether because
it was outside the sensor's physical FOV, occluded, or just missed by
sparse sampling -- either way, a new point along that direction is not
evidence of motion, just of a change in viewpoint. A direction *with* a
previous entry only counts as evidence of motion if the new point is
genuinely closer than what was already there -- something now blocking a
line of sight the previous scan had clear to a farther surface. Farther-
or similar-range returns in an already-seen direction are treated as the
same static background (revealed at a new range because whatever used to
be in front of it, if anything, is no longer relevant), not as movement --
this is deliberately conservative, trading a small chance of missing an
object that recedes to about where the background already was for
suppressing the much more common "newly visible edge geometry" false
positive.

Only position is used, not orientation. Two point sets already expressed
in the same world frame (as /cloud_registered is, via FastLIO) only need a
common origin to compare directions from -- the direction from that origin
to a world-frame point is a purely geometric quantity, independent of
which way the sensor body happened to be facing when it captured either
point set. Modelling the sensor's actual body-relative blind zones would
need orientation too, but isn't necessary here: a direction the sensor
physically couldn't see from a given position shows up automatically as an
empty bin in the range image, without needing to know why it's empty.

Units: metres for all positions/ranges, radians for all angles, seconds
nowhere (this module has no time dependency of its own).
"""
import numpy as np


def points_to_spherical(points: np.ndarray, origin: np.ndarray):
    """
    Convert points (N, 3, metres) to range/azimuth/elevation relative to
    origin (3,, metres), using world axes -- not the sensor's own
    orientation, see the module docstring for why that's fine here.

    azimuth: atan2(y, x), (-pi, pi].
    elevation: atan2(z, hypot(x, y)), [-pi/2, pi/2].
    """
    relative = points - origin
    ranges = np.linalg.norm(relative, axis=1)
    azimuth = np.arctan2(relative[:, 1], relative[:, 0])
    elevation = np.arctan2(relative[:, 2], np.hypot(relative[:, 0], relative[:, 1]))
    return ranges, azimuth, elevation


def _bin_indices(azimuth: np.ndarray, elevation: np.ndarray,
                  azimuth_bins: int, elevation_bins: int):
    """
    Map angles to grid indices. Clipped rather than wrapped/asserted at the
    exact +pi/+pi/2 edge -- floating point can land a hair past either
    boundary, and losing a handful of points into the last bin is harmless
    here (unlike, say, an actual panorama stitch).
    """
    az_idx = np.floor((azimuth + np.pi) / (2 * np.pi) * azimuth_bins).astype(np.int64)
    el_idx = np.floor((elevation + np.pi / 2) / np.pi * elevation_bins).astype(np.int64)
    return (np.clip(az_idx, 0, azimuth_bins - 1),
            np.clip(el_idx, 0, elevation_bins - 1))


def build_range_image(points: np.ndarray, origin: np.ndarray,
                       azimuth_bins: int, elevation_bins: int) -> np.ndarray:
    """
    Bin points into a (elevation_bins, azimuth_bins) grid around origin,
    keeping the minimum range seen in each direction. Empty bins are
    np.inf -- a direction the given point set never returned anything in,
    treated by previously_visible_mask below as "unknown," not "empty
    space" or "far away."
    """
    image = np.full((elevation_bins, azimuth_bins), np.inf)
    if len(points) == 0:
        return image

    ranges, azimuth, elevation = points_to_spherical(points, origin)
    az_idx, el_idx = _bin_indices(azimuth, elevation, azimuth_bins, elevation_bins)
    np.minimum.at(image, (el_idx, az_idx), ranges)
    return image


def previously_visible_mask(points: np.ndarray, prev_origin: np.ndarray,
                             prev_range_image: np.ndarray,
                             azimuth_bins: int, elevation_bins: int,
                             tolerance: float) -> np.ndarray:
    """
    For each point, look up the previous frame's range image in the same
    direction from prev_origin (the previous frame's own sensor position)
    and return True only where that direction had a previous entry *and*
    this point is more than tolerance closer than it -- the signature of
    something now blocking a line of sight the previous scan had clear to
    a farther surface, as opposed to a direction the previous scan never
    reached (no entry) or one where the range is roughly the same or
    farther (revealed background, not something new). See the module
    docstring for the full reasoning.

    tolerance (m) should be a little looser than the raw Euclidean
    change_threshold this gate runs downstream of -- it's comparing a
    single ray's range across two different viewpoints and bin
    discretisations, a noisier signal than nearest-neighbour distance
    within one point cloud.
    """
    if len(points) == 0:
        return np.zeros(0, dtype=bool)

    ranges, azimuth, elevation = points_to_spherical(points, prev_origin)
    az_idx, el_idx = _bin_indices(azimuth, elevation, azimuth_bins, elevation_bins)
    previous_ranges = prev_range_image[el_idx, az_idx]

    return np.isfinite(previous_ranges) & (ranges < previous_ranges - tolerance)
