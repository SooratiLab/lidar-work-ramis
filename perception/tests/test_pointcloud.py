"""
Unit tests for pointcloud.py -- PointCloud2 parsing and voxel downsampling.

pointcloud2_to_xyz only reads plain attributes off the message object
(fields/data/width/height/point_step), so a small fake stand-in for
sensor_msgs/msg/PointCloud2 exercises it without needing rclpy installed.
"""
import numpy as np

from pointcloud import pointcloud2_to_xyz, voxel_downsample


class _FakeField:
    def __init__(self, name, offset, datatype):
        self.name = name
        self.offset = offset
        self.datatype = datatype


class _FakePointCloud2:
    def __init__(self, points: np.ndarray):
        # x, y, z, intensity (float32), matching a typical /cloud_registered
        # layout -- the intensity field is deliberately included to check
        # that pointcloud2_to_xyz only picks out x/y/z and ignores it.
        self.fields = [
            _FakeField("x", 0, 7),
            _FakeField("y", 4, 7),
            _FakeField("z", 8, 7),
            _FakeField("intensity", 12, 7),
        ]
        self.point_step = 16
        self.width = len(points)
        self.height = 1

        structured = np.zeros(len(points), dtype=np.dtype({
            "names": ["x", "y", "z", "intensity"],
            "formats": ["f4", "f4", "f4", "f4"],
            "offsets": [0, 4, 8, 12],
            "itemsize": 16,
        }))
        structured["x"] = points[:, 0]
        structured["y"] = points[:, 1]
        structured["z"] = points[:, 2]
        structured["intensity"] = 42.0
        self.data = structured.tobytes()


def test_pointcloud2_to_xyz_roundtrips_float32_points():
    points = np.array([[1.0, 2.0, 3.0], [-1.5, 0.0, 4.25]], dtype=np.float32)
    msg = _FakePointCloud2(points)

    result = pointcloud2_to_xyz(msg)

    assert result.shape == (2, 3)
    np.testing.assert_allclose(result, points, rtol=1e-6)


def test_voxel_downsample_averages_points_within_a_cell():
    # Two points in the same 0.1 m cell, one point far away in its own cell.
    points = np.array([
        [0.01, 0.01, 0.01],
        [0.03, 0.02, 0.00],
        [5.0, 5.0, 5.0],
    ])

    result = voxel_downsample(points, voxel_size=0.1)

    assert result.shape == (2, 3)
    # The near-origin pair should have been merged into their mean, not an
    # arbitrary pick of one of the two.
    near_origin = result[np.argmin(np.linalg.norm(result, axis=1))]
    np.testing.assert_allclose(near_origin, [0.02, 0.015, 0.005], atol=1e-9)


def test_voxel_downsample_empty_input():
    result = voxel_downsample(np.empty((0, 3)), voxel_size=0.05)
    assert len(result) == 0
