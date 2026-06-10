"""Pure-numpy point cloud preparation: crop, downsample, outlier removal."""

from __future__ import annotations

import numpy as np

# Voxel indices are biased by _OFFSET and bit-packed 21 bits per axis into one
# int64 key, giving a usable index range of roughly +/- 1 million voxels per
# axis. At a 0.05 m voxel that is +/- 50 km, far beyond any real map.
_AXIS_BITS = 21
_OFFSET = 1 << (_AXIS_BITS - 1)
_AXIS_RANGE = 1 << _AXIS_BITS


def _voxel_keys(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """Pack per-point voxel indices into a single sortable int64 key."""
    if voxel_size <= 0.0:
        raise ValueError("voxel_size must be positive")
    indices = np.floor(points[:, :3] / voxel_size).astype(np.int64) + _OFFSET
    if indices.min() < 0 or indices.max() >= _AXIS_RANGE:
        raise ValueError("point coordinates exceed the packable voxel range")
    return (
        (indices[:, 0] << (2 * _AXIS_BITS))
        | (indices[:, 1] << _AXIS_BITS)
        | indices[:, 2]
    )


def crop_box(
    points: np.ndarray,
    min_xyz: tuple[float, float, float] | None = None,
    max_xyz: tuple[float, float, float] | None = None,
) -> np.ndarray:
    """Keep points inside the axis-aligned box. ``None`` bounds are open."""
    points = np.asarray(points, dtype=np.float32)
    mask = np.ones(len(points), dtype=bool)
    if min_xyz is not None:
        mask &= (points[:, :3] >= np.asarray(min_xyz, dtype=np.float32)).all(axis=1)
    if max_xyz is not None:
        mask &= (points[:, :3] <= np.asarray(max_xyz, dtype=np.float32)).all(axis=1)
    return points[mask]


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """Reduce the cloud to one centroid per occupied voxel."""
    points = np.asarray(points, dtype=np.float32)
    if points.size == 0:
        return points.reshape(0, 3)
    keys = _voxel_keys(points, voxel_size)
    order = np.argsort(keys, kind="stable")
    sorted_points = points[order, :3].astype(np.float64)
    sorted_keys = keys[order]
    starts = np.concatenate(([0], np.nonzero(np.diff(sorted_keys))[0] + 1))
    counts = np.diff(np.concatenate((starts, [len(sorted_keys)])))
    sums = np.add.reduceat(sorted_points, starts, axis=0)
    return (sums / counts[:, None]).astype(np.float32)


def remove_sparse_voxels(
    points: np.ndarray,
    voxel_size: float = 0.3,
    min_neighborhood_points: int = 4,
) -> np.ndarray:
    """Drop points whose 3x3x3 voxel neighborhood holds too few points.

    A cheap, fully vectorized stand-in for radius outlier removal: isolated
    speckle returns (dust, rain, multipath) occupy voxels whose neighborhoods
    are nearly empty, while real surfaces are locally dense at any reasonable
    ``voxel_size``.
    """
    points = np.asarray(points, dtype=np.float32)
    if points.size == 0:
        return points.reshape(0, 3)
    keys = _voxel_keys(points, voxel_size)
    unique_keys, inverse, counts = np.unique(
        keys, return_inverse=True, return_counts=True
    )

    neighborhood = np.zeros(len(unique_keys), dtype=np.int64)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                offset = (dx << (2 * _AXIS_BITS)) + (dy << _AXIS_BITS) + dz
                neighbor_keys = unique_keys + offset
                pos = np.searchsorted(unique_keys, neighbor_keys)
                pos = np.clip(pos, 0, len(unique_keys) - 1)
                found = unique_keys[pos] == neighbor_keys
                neighborhood += np.where(found, counts[pos], 0)

    keep_voxel = neighborhood >= int(min_neighborhood_points)
    return points[keep_voxel[inverse]]
