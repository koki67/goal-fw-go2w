import numpy as np
import pytest
from nav_goal_go2w_web.prep_grid_core import downsample_voxel, project_points


def test_projects_z_band_to_occupied_grid():
    grid = project_points(np.array([[0, 0, 0], [1, 0, 0.5], [5, 5, 3]], dtype=np.float32), resolution=0.5)
    assert np.count_nonzero(grid.data == 100) == 2
    assert grid.width > 1 and grid.height >= 1


def test_empty_cloud_is_valid_single_cell():
    grid = project_points(np.empty((0, 3), dtype=np.float32))
    assert (grid.width, grid.height, grid.data.tolist()) == (1, 1, [0])


def test_large_grid_is_coarsened():
    grid = project_points(np.array([[0, 0, 0], [100, 100, 0]], dtype=np.float32), resolution=0.01, max_cells=10000)
    assert grid.width * grid.height <= 10201
    assert grid.resolution > 0.01


def test_invalid_parameters_rejected():
    with pytest.raises(ValueError):
        project_points(np.empty((0, 3)), resolution=0)


def test_downsample_keeps_one_point_per_voxel():
    points = np.array([[0, 0, 0], [0.01, 0.01, 0.01], [1, 1, 1]], dtype=np.float32)
    out = downsample_voxel(points, leaf=0.1)
    assert len(out) == 2
    assert out.tolist()[0] == [0, 0, 0]


def test_downsample_caps_point_budget():
    points = np.random.default_rng(0).random((5000, 3)).astype(np.float32) * 100
    out = downsample_voxel(points, leaf=0.01, max_points=1000)
    assert len(out) <= 1000


def test_downsample_empty_and_invalid():
    assert len(downsample_voxel(np.empty((0, 3), dtype=np.float32))) == 0
    with pytest.raises(ValueError):
        downsample_voxel(np.empty((0, 3)), leaf=0)
