"""Pure NumPy projection used by the browser map-preparation preview."""
from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class PrepGrid:
    origin_x: float
    origin_y: float
    resolution: float
    width: int
    height: int
    data: np.ndarray


def pointcloud2_to_xyz(msg) -> np.ndarray:
    offsets = {field.name: field.offset for field in msg.fields}
    missing = {"x", "y", "z"} - offsets.keys()
    if missing:
        raise ValueError(f"PointCloud2 lacks fields: {sorted(missing)}")
    dtype = np.dtype({"names": ["x", "y", "z"], "formats": [np.float32] * 3,
        "offsets": [offsets["x"], offsets["y"], offsets["z"]], "itemsize": msg.point_step})
    structured = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
        strides=(msg.row_step, msg.point_step))
    points = np.column_stack([structured[name].ravel() for name in ("x", "y", "z")]).astype(np.float32)
    return points[np.isfinite(points).all(axis=1)]


def downsample_voxel(points: np.ndarray, leaf: float = 0.15, max_points: int = 150_000) -> np.ndarray:
    """Keep one point per voxel, then stride-decimate down to the point budget."""
    points = np.asarray(points, dtype=np.float32)
    if leaf <= 0 or max_points <= 0:
        raise ValueError("invalid downsample parameters")
    if len(points):
        keys = np.floor(points / leaf).astype(np.int64)
        keys -= keys.min(axis=0)
        dims = keys.max(axis=0) + 1
        flat = (keys[:, 0] * dims[1] + keys[:, 1]) * dims[2] + keys[:, 2]
        _, index = np.unique(flat, return_index=True)
        points = points[np.sort(index)]
    if len(points) > max_points:
        points = points[::-(-len(points) // max_points)]
    return points


def project_points(points: np.ndarray, resolution: float = 0.10, z_min: float = -0.25,
                   z_max: float = 1.75, max_cells: int = 1_000_000) -> PrepGrid:
    points = np.asarray(points, dtype=np.float32)
    if resolution <= 0 or max_cells <= 0 or z_min > z_max:
        raise ValueError("invalid projection parameters")
    selected = points[(points[:, 2] >= z_min) & (points[:, 2] <= z_max)] if len(points) else points
    if not len(selected):
        return PrepGrid(0.0, 0.0, resolution, 1, 1, np.zeros(1, dtype=np.int8))
    lo = selected[:, :2].min(axis=0) - resolution
    hi = selected[:, :2].max(axis=0) + resolution
    span = np.maximum(hi - lo, resolution)
    width, height = np.ceil(span / resolution).astype(int)
    cells = int(width) * int(height)
    while cells > max_cells:
        resolution *= max(1.01, math.sqrt(cells / max_cells))
        width, height = np.ceil(span / resolution).astype(int)
        cells = int(width) * int(height)
    width, height = max(1, int(width)), max(1, int(height))
    xy = np.floor((selected[:, :2] - lo) / resolution).astype(int)
    xy[:, 0] = np.clip(xy[:, 0], 0, width - 1)
    xy[:, 1] = np.clip(xy[:, 1], 0, height - 1)
    data = np.zeros(width * height, dtype=np.int8)
    data[xy[:, 1] * width + xy[:, 0]] = 100
    return PrepGrid(float(lo[0]), float(lo[1]), float(resolution), width, height, data)
