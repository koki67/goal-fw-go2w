"""PCD file I/O for map preparation, backed by pypcd4.

pypcd4 reads all PCD encodings (ascii, binary, binary_compressed), which
covers both D-LIO ``save_pcd`` output and clouds exported by handheld
mapping tools.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from pypcd4 import PointCloud


def load_xyz(path: str | Path) -> np.ndarray:
    """Load a PCD file and return finite XYZ points as an (N, 3) float32 array."""
    pcd_path = Path(path)
    if not pcd_path.is_file():
        raise FileNotFoundError(f"PCD file not found: {pcd_path}")
    cloud = PointCloud.from_path(pcd_path)
    missing = {"x", "y", "z"} - set(cloud.fields)
    if missing:
        raise ValueError(f"PCD {pcd_path} lacks fields: {sorted(missing)}")
    points = cloud.numpy(("x", "y", "z")).astype(np.float32, copy=False)
    finite = np.isfinite(points).all(axis=1)
    return np.ascontiguousarray(points[finite])


def save_xyz(path: str | Path, points: np.ndarray) -> None:
    """Save an (N, 3) array as a binary PCD file."""
    points = np.ascontiguousarray(np.asarray(points, dtype=np.float32))
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    pcd_path = Path(path)
    pcd_path.parent.mkdir(parents=True, exist_ok=True)
    PointCloud.from_xyz_points(points).save(pcd_path)
