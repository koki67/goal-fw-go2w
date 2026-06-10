import numpy as np
import pytest

from nav_goal_go2w_map import pcd_io


def test_roundtrip(tmp_path):
    points = np.random.default_rng(0).normal(size=(100, 3)).astype(np.float32)
    path = tmp_path / "cloud.pcd"
    pcd_io.save_xyz(path, points)
    loaded = pcd_io.load_xyz(path)
    np.testing.assert_allclose(loaded, points, atol=1e-6)


def test_load_drops_non_finite(tmp_path):
    points = np.array(
        [[0, 0, 0], [np.nan, 1, 1], [2, 2, 2]], dtype=np.float32
    )
    path = tmp_path / "cloud.pcd"
    pcd_io.save_xyz(path, points)
    loaded = pcd_io.load_xyz(path)
    assert len(loaded) == 2


def test_missing_file_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        pcd_io.load_xyz(tmp_path / "absent.pcd")


def test_save_rejects_bad_shape(tmp_path):
    with pytest.raises(ValueError):
        pcd_io.save_xyz(tmp_path / "bad.pcd", np.zeros((3, 2), np.float32))
