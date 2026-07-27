"""Small file-format helpers shared by evaluation tests."""
import numpy as np


def write_binary_pcd(path, points: np.ndarray, extra_fields=()):
    """Write the narrow binary PCD format consumed by ``pcd_io``."""
    fields = ["x", "y", "z"] + list(extra_fields)
    n_fields = len(fields)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        f"FIELDS {' '.join(fields)}\n"
        f"SIZE {' '.join(['4'] * n_fields)}\n"
        f"TYPE {' '.join(['F'] * n_fields)}\n"
        f"COUNT {' '.join(['1'] * n_fields)}\n"
        f"WIDTH {len(points)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\n"
        "DATA binary\n"
    )
    body = np.zeros((len(points), n_fields), dtype="f4")
    body[:, :3] = points
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(body.tobytes())
