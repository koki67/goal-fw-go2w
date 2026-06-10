import numpy as np
import pytest

from nav_goal_go2w_map.map_prep_core import (
    crop_box,
    remove_sparse_voxels,
    voxel_downsample,
)


def test_voxel_downsample_one_centroid_per_voxel():
    points = np.array(
        [
            [0.01, 0.01, 0.01],
            [0.03, 0.03, 0.03],
            [1.01, 0.01, 0.01],
        ],
        dtype=np.float32,
    )
    out = voxel_downsample(points, 0.1)
    assert out.shape == (2, 3)
    merged = out[np.argsort(out[:, 0])]
    np.testing.assert_allclose(merged[0], [0.02, 0.02, 0.02], atol=1e-6)
    np.testing.assert_allclose(merged[1], [1.01, 0.01, 0.01], atol=1e-6)


def test_voxel_downsample_handles_negative_coordinates():
    points = np.array([[-0.05, -0.05, -0.05], [-0.01, -0.01, -0.01]], np.float32)
    out = voxel_downsample(points, 0.1)
    assert out.shape == (1, 3)
    np.testing.assert_allclose(out[0], [-0.03, -0.03, -0.03], atol=1e-6)


def test_voxel_downsample_empty():
    assert voxel_downsample(np.empty((0, 3), np.float32), 0.1).shape == (0, 3)


def test_voxel_downsample_rejects_bad_voxel():
    with pytest.raises(ValueError):
        voxel_downsample(np.zeros((1, 3), np.float32), 0.0)


def test_crop_box_bounds():
    points = np.array([[0, 0, 0], [1, 1, 1], [2, 2, 2]], dtype=np.float32)
    out = crop_box(points, min_xyz=(0.5, 0.5, 0.5), max_xyz=(1.5, 1.5, 1.5))
    assert out.shape == (1, 3)
    np.testing.assert_allclose(out[0], [1, 1, 1])


def test_remove_sparse_voxels_drops_isolated_point():
    rng = np.random.default_rng(7)
    # Dense planar patch plus one far-away speckle return.
    surface = rng.uniform(0.0, 1.0, size=(500, 3)).astype(np.float32)
    surface[:, 2] *= 0.02
    speckle = np.array([[10.0, 10.0, 10.0]], dtype=np.float32)
    cloud = np.vstack((surface, speckle))
    out = remove_sparse_voxels(cloud, voxel_size=0.3, min_neighborhood_points=4)
    assert len(out) == 500
    assert not (out == speckle).all(axis=1).any()


def test_remove_sparse_voxels_keeps_dense_cloud_intact():
    rng = np.random.default_rng(3)
    cloud = rng.uniform(0.0, 0.5, size=(300, 3)).astype(np.float32)
    out = remove_sparse_voxels(cloud, voxel_size=0.5, min_neighborhood_points=4)
    assert len(out) == 300
