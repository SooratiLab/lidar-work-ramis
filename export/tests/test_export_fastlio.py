"""
Unit tests for export_fastlio.py's pure functions -- read_pointcloud2_xyzi
and write_pcd_binary. Both only touch plain data (a PointCloud2-shaped
object's attributes, or a numpy array), so neither needs rclpy installed to
test, matching perception/tests/test_pointcloud.py's approach for the same
message type.
"""
import sys
from pathlib import Path

import numpy as np

from export_fastlio import read_pointcloud2_xyzi, write_pcd_binary

# Round-trip write_pcd_binary's output through evaluation/'s own PCD reader
# rather than re-implementing a second parser here -- it already reads
# exactly this format (see its own docstring), and a passing round-trip
# here is direct evidence the two stay compatible, which is the whole
# point of the format being shared between them.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent / "evaluation"))
from pcd_io import load_pcd_xyz_mm  # noqa: E402


class _FakeField:
    def __init__(self, name, offset, datatype):
        self.name = name
        self.offset = offset
        self.datatype = datatype


class _FakePointCloud2:
    def __init__(self, points_xyzi: np.ndarray, include_intensity=True):
        fields = ["x", "y", "z"] + (["intensity"] if include_intensity else [])
        n_fields = len(fields)
        self.fields = [
            _FakeField(name, i * 4, 7) for i, name in enumerate(fields)
        ]
        self.point_step = n_fields * 4
        self.width = len(points_xyzi)
        self.height = 1

        structured = np.zeros(len(points_xyzi), dtype=np.dtype({
            "names": fields, "formats": ["f4"] * n_fields,
            "offsets": [i * 4 for i in range(n_fields)],
            "itemsize": self.point_step,
        }))
        structured["x"] = points_xyzi[:, 0]
        structured["y"] = points_xyzi[:, 1]
        structured["z"] = points_xyzi[:, 2]
        if include_intensity:
            structured["intensity"] = points_xyzi[:, 3]
        self.data = structured.tobytes()


def test_read_pointcloud2_xyzi_roundtrips_xyz_and_intensity():
    points = np.array([
        [1.0, 2.0, 3.0, 100.0],
        [-1.5, 0.0, 4.25, 255.0],
    ], dtype=np.float32)
    msg = _FakePointCloud2(points)

    result = read_pointcloud2_xyzi(msg)

    assert result.shape == (2, 4)
    np.testing.assert_allclose(result, points, rtol=1e-6)


def test_read_pointcloud2_xyzi_defaults_intensity_to_zero_when_absent():
    points = np.array([[1.0, 2.0, 3.0, 0.0]], dtype=np.float32)
    msg = _FakePointCloud2(points, include_intensity=False)

    result = read_pointcloud2_xyzi(msg)

    assert result.shape == (1, 4)
    np.testing.assert_allclose(result[:, :3], points[:, :3], rtol=1e-6)
    assert result[0, 3] == 0.0


def test_write_pcd_binary_roundtrips_through_evaluations_reader(tmp_path):
    # Values chosen to already look like millimetres, matching what
    # cloud_cb actually passes in (scaled x1000 before calling this).
    points = np.array([
        [1000.0, 2000.0, -500.0, 42.0],
        [0.0, 0.0, 0.0, 0.0],
    ], dtype=np.float32)
    path = tmp_path / "frame_000000.pcd"

    write_pcd_binary(str(path), points)
    result = load_pcd_xyz_mm(path)

    assert result.shape == (2, 3)
    np.testing.assert_allclose(result, points[:, :3], atol=1e-3)
