"""
pointcloud.py -- PointCloud2 parsing and voxel downsampling for the online
perception node.

Split out of online_perception_node.py so both pieces can be unit tested
without needing rclpy or a running ROS graph -- pointcloud2_to_xyz only
touches the fields it needs off the message object (name/offset/datatype/
data/width/height/point_step), so a plain object with those attributes set
is enough to exercise it in a test.

Units: metres throughout, matching FastLIO's /cloud_registered directly.
"""
import numpy as np


# sensor_msgs/msg/PointField datatype constants -> numpy dtype characters.
_POINTFIELD_TO_NUMPY = {
    1: "i1", 2: "u1",
    3: "i2", 4: "u2",
    5: "i4", 6: "u4",
    7: "f4", 8: "f8",
}


def pointcloud2_to_xyz(msg) -> np.ndarray:
    """
    Parse a PointCloud2 into an (N, 3) float64 array of x, y, z (metres).

    Builds a numpy structured dtype directly from the message's own field
    layout (name/offset/datatype) rather than assuming FLOAT32 XYZ packed
    at offsets 0/4/8 -- works whether or not an intensity field is present,
    and regardless of field order. Uses np.frombuffer to reinterpret the
    message bytes directly instead of a per-point Python loop (the offline
    pipeline's export_fastlio.py can afford that loop since it runs once
    per recording; this runs every scan, live).
    """
    names, formats, offsets = [], [], []
    for field in msg.fields:
        if field.name not in ("x", "y", "z"):
            continue
        names.append(field.name)
        formats.append(_POINTFIELD_TO_NUMPY[field.datatype])
        offsets.append(field.offset)

    dtype = np.dtype({
        "names": names,
        "formats": formats,
        "offsets": offsets,
        "itemsize": msg.point_step,
    })

    count = msg.width * msg.height
    structured = np.frombuffer(msg.data, dtype=dtype, count=count)
    return np.column_stack(
        [structured["x"], structured["y"], structured["z"]]
    ).astype(np.float64)


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """
    Voxel-grid downsample: average the points falling in each occupied cell
    of size voxel_size (metres) into a single centroid point.

    This used to keep an arbitrary point per cell (whichever np.unique
    happened to return first) rather than averaging, which was fine for
    distance-threshold change detection but threw away real position
    information for no reason -- the mean is just as cheap to compute and
    gives every downstream step (change detection, clustering, the track
    centroids themselves) a less noisy representative point. Matches
    Open3D's voxel_down_sample semantics, without the Open3D dependency.

    Implementation: floor-divide by voxel_size to get integer cell
    coordinates, lexsort so points in the same cell are contiguous, then
    sum/count each contiguous run with np.add.reduceat. One sort plus one
    vectorised reduction, no per-point Python loop and no per-cell Python
    loop either.
    """
    if len(points) == 0:
        return points

    cell = np.floor(points / voxel_size).astype(np.int64)
    order = np.lexsort(cell.T[::-1])
    sorted_cell = cell[order]
    sorted_points = points[order]

    is_new_cell = np.any(sorted_cell[1:] != sorted_cell[:-1], axis=1)
    group_starts = np.concatenate(([0], np.nonzero(is_new_cell)[0] + 1))

    sums = np.add.reduceat(sorted_points, group_starts, axis=0)
    counts = np.diff(np.append(group_starts, len(sorted_points)))
    return sums / counts[:, None]
