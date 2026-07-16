"""
pcd_io.py -- minimal binary PCD reader for evaluation/.

Handles exactly the format kei-stuff/ros2-go2/scripts/export_fastlio.py
writes (FIELDS x y z intensity, TYPE F F F F, DATA binary) -- this is a
narrow reader for that one format, not a general PCD parser. Written
separately from perception/pointcloud.py's PointCloud2 parser because a PCD
file and a ROS PointCloud2 message have different headers even though the
underlying point layout ends up the same; there's no shared code to factor
out between the two beyond "structured numpy dtype from an offset/size
list," which is a few lines either way.

Units: millimetres, matching the PCD files themselves (see
lidar-perception/README.md's "Important: coordinate units"). Callers
comparing against perception/'s metre-based pipeline need to divide by
1000 themselves -- kept explicit at the call site rather than folded into
this reader, so a unit mismatch is never silently hidden here.
"""
import numpy as np


def load_pcd_xyz_mm(path) -> np.ndarray:
    """Read a binary PCD file's x/y/z fields (mm) into an (N, 3) array."""
    with open(path, "rb") as f:
        data = f.read()

    marker = b"DATA binary\n"
    header_end = data.index(marker) + len(marker)
    header_text = data[:header_end].decode("ascii")

    fields = sizes = types = counts = points = None
    for line in header_text.splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "FIELDS":
            fields = parts[1:]
        elif parts[0] == "SIZE":
            sizes = [int(x) for x in parts[1:]]
        elif parts[0] == "TYPE":
            types = parts[1:]
        elif parts[0] == "COUNT":
            counts = [int(x) for x in parts[1:]]
        elif parts[0] == "POINTS":
            points = int(parts[1])

    if fields[:3] != ["x", "y", "z"] or types[:3] != ["F", "F", "F"] or sizes[:3] != [4, 4, 4]:
        raise ValueError(
            f"{path}: expected float32 x/y/z as the first three fields, "
            f"got fields={fields} types={types} sizes={sizes}")

    point_step = sum(s * c for s, c in zip(sizes, counts))
    offsets = list(np.cumsum([0] + [s * c for s, c in zip(sizes, counts)]))[:-1]
    dtype = np.dtype({
        "names": fields, "formats": ["f4"] * len(fields),
        "offsets": offsets, "itemsize": point_step,
    })

    body = data[header_end:]
    structured = np.frombuffer(body, dtype=dtype, count=points)
    return np.column_stack(
        [structured["x"], structured["y"], structured["z"]]
    ).astype(np.float64)
