"""
Unit tests for pcd_io.py -- the binary PCD reader used to replay Kei's
already-exported PCD frames without a live ROS graph (see
offline_pipeline.py's module docstring for why).
"""
import numpy as np
import pytest

from conftest import write_binary_pcd
from pcd_io import load_pcd_xyz_mm


def test_load_pcd_xyz_mm_roundtrips_xyz_only(tmp_path):
    points = np.array([[1.0, 2.0, 3.0], [-500.0, 0.0, 4250.0]])
    path = tmp_path / "frame.pcd"
    write_binary_pcd(path, points)

    result = load_pcd_xyz_mm(path)

    assert result.shape == (2, 3)
    np.testing.assert_allclose(result, points, atol=1e-3)


def test_load_pcd_xyz_mm_ignores_intensity_field(tmp_path):
    points = np.array([[10.0, 20.0, 30.0]])
    path = tmp_path / "frame.pcd"
    write_binary_pcd(path, points, extra_fields=["intensity"])

    result = load_pcd_xyz_mm(path)

    assert result.shape == (1, 3)
    np.testing.assert_allclose(result, points, atol=1e-3)


def test_load_pcd_xyz_mm_rejects_non_xyz_leading_fields(tmp_path):
    path = tmp_path / "frame.pcd"
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS intensity x y z\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        "WIDTH 0\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 0\nDATA binary\n"
    )
    path.write_bytes(header.encode("ascii"))

    with pytest.raises(ValueError):
        load_pcd_xyz_mm(path)
