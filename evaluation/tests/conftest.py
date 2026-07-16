"""
Make evaluation/ importable from tests/ regardless of which directory
pytest is invoked from -- same convention as perception/tests/conftest.py.
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


def write_binary_pcd(path, points: np.ndarray, extra_fields=()):
    """
    Write a minimal binary PCD matching export_fastlio.py's own format
    (FIELDS x y z [intensity ...], TYPE F..., DATA binary) -- enough to
    exercise pcd_io.load_pcd_xyz_mm without needing Open3D as a test
    dependency. Shared between test_pcd_io.py and test_offline_pipeline.py
    rather than duplicated in each.
    """
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
