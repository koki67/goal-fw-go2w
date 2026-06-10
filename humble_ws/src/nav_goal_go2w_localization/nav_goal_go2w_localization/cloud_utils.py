"""PointCloud2 <-> numpy conversion and scan cropping."""

from __future__ import annotations

import numpy as np


def pointcloud2_to_xyz(msg) -> np.ndarray:
    """Vectorized PointCloud2 -> (N, 3) float32, non-finite points dropped.

    Reads the x/y/z fields directly from the byte buffer via a strided
    structured dtype, so arbitrary extra fields (intensity, time, ring...)
    cost nothing.
    """
    offsets = {f.name: f.offset for f in msg.fields}
    missing = {"x", "y", "z"} - offsets.keys()
    if missing:
        raise ValueError(f"PointCloud2 lacks fields: {sorted(missing)}")
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": [np.float32, np.float32, np.float32],
            "offsets": [offsets["x"], offsets["y"], offsets["z"]],
            "itemsize": msg.point_step,
        }
    )
    count = (msg.width * msg.height * msg.point_step) // msg.point_step
    structured = np.frombuffer(msg.data, dtype=dtype, count=count)
    points = np.column_stack(
        (structured["x"], structured["y"], structured["z"])
    ).astype(np.float32)
    return points[np.isfinite(points).all(axis=1)]


def crop_range(
    points: np.ndarray,
    center: np.ndarray,
    min_range: float,
    max_range: float,
) -> np.ndarray:
    """Keep points whose distance from ``center`` is in [min_range, max_range]."""
    if len(points) == 0:
        return points
    squared = ((points[:, :3] - np.asarray(center, dtype=points.dtype)) ** 2).sum(
        axis=1
    )
    return points[(squared >= min_range**2) & (squared <= max_range**2)]
