"""Pure-numpy 3D point cloud -> 2D occupancy grid for Nav2 planning.

Classification is ground-relative, never absolute-z: each cell estimates its
own ground height (robust low percentile, neighborhood-smoothed), and a point
is an obstacle only if it lies within the robot's collision band *above that
ground*. This keeps sloped floors free and stops ground returns from a
handheld-mapping session polluting the grid.
"""

from __future__ import annotations

import warnings
from dataclasses import dataclass

import numpy as np

from nav_goal_go2w_map.terrain_core import TerrainConfig, TerrainMap

UNKNOWN = np.int8(-1)
FREE = np.int8(0)
OCCUPIED = np.int8(100)


@dataclass(frozen=True)
class GridConfig:
    resolution: float = 0.05
    padding_m: float = 0.5
    ground_percentile: float = 10.0
    ground_fill_radius: int = 2
    obstacle_z_min: float = 0.15
    obstacle_z_max: float = 1.5
    min_obstacle_cells: int = 3
    fill_unknown_islands_smaller_than: int = 0
    terrain_classify: bool = False

    def __post_init__(self) -> None:
        if self.resolution <= 0.0:
            raise ValueError("resolution must be positive")
        if not 0.0 <= self.ground_percentile <= 50.0:
            raise ValueError("ground_percentile must be in [0, 50]")
        if self.obstacle_z_min < 0.0 or self.obstacle_z_max <= self.obstacle_z_min:
            raise ValueError("obstacle band must satisfy 0 <= z_min < z_max")
        if self.ground_fill_radius < 0:
            raise ValueError("ground_fill_radius must be >= 0")


@dataclass
class BuiltGrid:
    data: np.ndarray  # (height, width) int8, row 0 at origin_y (south edge)
    resolution: float
    origin_x: float
    origin_y: float

    @property
    def width(self) -> int:
        return self.data.shape[1]

    @property
    def height(self) -> int:
        return self.data.shape[0]


def build_grid(points_xyz: np.ndarray, config: GridConfig | None = None) -> BuiltGrid:
    """Build an occupancy grid from a map-frame point cloud."""
    config = config or GridConfig()
    points = np.asarray(points_xyz, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] < 3:
        raise ValueError("points_xyz must have shape (N, 3)")
    points = points[np.isfinite(points).all(axis=1)][:, :3]
    if len(points) == 0:
        raise ValueError("cannot build a grid from an empty cloud")

    res = config.resolution
    origin_x = float(points[:, 0].min()) - config.padding_m
    origin_y = float(points[:, 1].min()) - config.padding_m
    width = int(np.ceil((points[:, 0].max() + config.padding_m - origin_x) / res)) + 1
    height = int(np.ceil((points[:, 1].max() + config.padding_m - origin_y) / res)) + 1

    cols = np.floor((points[:, 0] - origin_x) / res).astype(np.int64)
    rows = np.floor((points[:, 1] - origin_y) / res).astype(np.int64)
    cell_ids = rows * width + cols

    ground = _per_cell_ground(cell_ids, points[:, 2], height, width, config)
    ground_ref = _smooth_ground(ground, config.ground_fill_radius)

    rel_z = points[:, 2] - ground_ref[rows, cols]
    obstacle_points = (rel_z >= config.obstacle_z_min) & (
        rel_z <= config.obstacle_z_max
    )
    ground_evidence_points = rel_z < config.obstacle_z_min

    occupied_mask = np.zeros((height, width), dtype=bool)
    occupied_mask[rows[obstacle_points], cols[obstacle_points]] = True
    ground_evidence = np.zeros((height, width), dtype=bool)
    ground_evidence[rows[ground_evidence_points], cols[ground_evidence_points]] = True

    if config.terrain_classify:
        occupied_mask |= _terrain_lethal_mask(
            points, ground_ref, origin_x, origin_y, height, width, res
        )

    occupied_mask = _remove_small_components(
        occupied_mask, config.min_obstacle_cells
    )

    grid = np.full((height, width), UNKNOWN, dtype=np.int8)
    grid[ground_evidence] = FREE
    grid[occupied_mask] = OCCUPIED

    if config.fill_unknown_islands_smaller_than > 0:
        grid = _fill_unknown_islands(
            grid, config.fill_unknown_islands_smaller_than
        )

    return BuiltGrid(data=grid, resolution=res, origin_x=origin_x, origin_y=origin_y)


def _per_cell_ground(
    cell_ids: np.ndarray,
    z: np.ndarray,
    height: int,
    width: int,
    config: GridConfig,
) -> np.ndarray:
    """Ground per cell = low z-percentile of that cell's points (NaN if empty)."""
    order = np.lexsort((z, cell_ids))
    sorted_ids = cell_ids[order]
    sorted_z = z[order]
    starts = np.concatenate(([0], np.nonzero(np.diff(sorted_ids))[0] + 1))
    counts = np.diff(np.concatenate((starts, [len(sorted_ids)])))
    pick = starts + np.floor(
        (counts - 1) * config.ground_percentile / 100.0
    ).astype(np.int64)

    ground = np.full(height * width, np.nan, dtype=np.float32)
    ground[sorted_ids[starts]] = sorted_z[pick]
    return ground.reshape(height, width)


def _smooth_ground(ground: np.ndarray, radius: int) -> np.ndarray:
    """Reference ground = min(own estimate, neighborhood median of estimates).

    The neighborhood median supplies ground under overhangs (cells whose only
    points are elevated, e.g. table tops), while the elementwise min keeps a
    raised surface classified relative to its surroundings: a 0.3 m ledge stays
    an obstacle even though its own points sit on a locally flat top.
    """
    if radius <= 0:
        return ground
    height, width = ground.shape
    window = 2 * radius + 1
    median = np.full_like(ground, np.nan)
    # Row-chunked so the (window^2, chunk, width) stack stays small.
    chunk_rows = max(1, int(4_000_000 / (window * window * width)))
    padded = np.pad(
        ground, radius, mode="constant", constant_values=np.nan
    )
    for row0 in range(0, height, chunk_rows):
        row1 = min(row0 + chunk_rows, height)
        stack = np.empty(
            (window * window, row1 - row0, width), dtype=np.float32
        )
        layer = 0
        for dr in range(window):
            for dc in range(window):
                stack[layer] = padded[
                    row0 + dr : row1 + dr, dc : dc + width
                ]
                layer += 1
        with warnings.catch_warnings():
            # All-NaN neighborhoods are expected outside the mapped area.
            warnings.simplefilter("ignore", category=RuntimeWarning)
            median[row0:row1] = np.nanmedian(stack, axis=0)
    return np.fmin(ground, median)


def _terrain_lethal_mask(
    points: np.ndarray,
    ground_ref: np.ndarray,
    origin_x: float,
    origin_y: float,
    height: int,
    width: int,
    resolution: float,
) -> np.ndarray:
    """Mark cells whose terrain (step/slope/roughness) is untraversable.

    Reuses the online TerrainMap classifier offline: feed it only near-ground
    points so vertical structure does not masquerade as terrain relief.
    """
    finite_ground = np.nanmin(ground_ref) if np.isfinite(ground_ref).any() else 0.0
    terrain_config = TerrainConfig(
        resolution=resolution,
        width_m=width * resolution,
        height_m=height * resolution,
        z_band_below=1.0e6,
        z_band_above=1.0e6,
    )
    terrain = TerrainMap(terrain_config)
    terrain.initialize_origin(
        origin_x + 0.5 * width * resolution,
        origin_y + 0.5 * height * resolution,
    )
    cols = np.floor((points[:, 0] - origin_x) / resolution).astype(np.int64)
    rows = np.floor((points[:, 1] - origin_y) / resolution).astype(np.int64)
    near_ground = (
        points[:, 2] - ground_ref[rows, cols]
    ) < terrain_config.step_max
    terrain.integrate_cloud(points[near_ground], robot_z=float(finite_ground))
    return terrain.cost == 99


def _remove_small_components(mask: np.ndarray, min_cells: int) -> np.ndarray:
    """Clear 8-connected occupied components smaller than ``min_cells``."""
    if min_cells <= 1 or not mask.any():
        return mask
    labels, sizes = _label_components(mask)
    small = np.flatnonzero(sizes < min_cells) + 1
    cleaned = mask.copy()
    cleaned[np.isin(labels, small)] = False
    return cleaned


def _fill_unknown_islands(grid: np.ndarray, max_cells: int) -> np.ndarray:
    """Convert small interior unknown pockets to free.

    Pockets touching the grid border are genuine map edge and stay unknown.
    """
    unknown = grid == UNKNOWN
    if not unknown.any():
        return grid
    labels, sizes = _label_components(unknown)
    border_labels = np.unique(
        np.concatenate(
            (labels[0, :], labels[-1, :], labels[:, 0], labels[:, -1])
        )
    )
    border_labels = border_labels[border_labels > 0]
    fill = np.flatnonzero(sizes < max_cells) + 1
    fill = np.setdiff1d(fill, border_labels)
    result = grid.copy()
    result[np.isin(labels, fill)] = FREE
    return result


def _label_components(mask: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """8-connected component labeling. Returns (labels, sizes).

    Labels are 1-based; 0 marks background. ``sizes[i]`` is the cell count of
    label ``i + 1``. Run-based two-pass labeling with union-find: rows are
    decomposed into runs (vectorized), runs are merged against the previous
    row, labels resolved at the end.
    """
    height, width = mask.shape
    labels = np.zeros((height, width), dtype=np.int64)
    parent: list[int] = [0]  # parent[label] for union-find; label 0 unused

    def find(x: int) -> int:
        root = x
        while parent[root] != root:
            root = parent[root]
        while parent[x] != root:
            parent[x], x = root, parent[x]
        return root

    prev_starts = np.empty(0, dtype=np.int64)
    prev_ends = np.empty(0, dtype=np.int64)
    prev_labels = np.empty(0, dtype=np.int64)
    for row in range(height):
        line = mask[row]
        if not line.any():
            prev_starts = prev_ends = prev_labels = np.empty(0, dtype=np.int64)
            continue
        edges = np.diff(line.astype(np.int8))
        starts = np.flatnonzero(edges == 1) + 1
        ends = np.flatnonzero(edges == -1) + 1
        if line[0]:
            starts = np.concatenate(([0], starts))
        if line[-1]:
            ends = np.concatenate((ends, [width]))

        run_labels = np.empty(len(starts), dtype=np.int64)
        for i, (start, end) in enumerate(zip(starts, ends)):
            # 8-connectivity: overlap with previous row extended by one cell.
            touching = (prev_starts < end + 1) & (prev_ends > start - 1)
            candidates = prev_labels[touching]
            if len(candidates) == 0:
                parent.append(len(parent))
                run_labels[i] = len(parent) - 1
            else:
                roots = sorted({find(int(c)) for c in candidates})
                keep = roots[0]
                for other in roots[1:]:
                    parent[other] = keep
                run_labels[i] = keep
            labels[row, start:end] = run_labels[i]
        prev_starts, prev_ends, prev_labels = starts, ends, run_labels

    if len(parent) == 1:
        return labels, np.empty(0, dtype=np.int64)
    roots = np.array([find(i) for i in range(len(parent))], dtype=np.int64)
    unique_roots, compact = np.unique(roots[1:], return_inverse=True)
    lookup = np.zeros(len(parent), dtype=np.int64)
    lookup[1:] = compact + 1
    labels = lookup[roots[labels]]
    sizes = np.bincount(labels.ravel(), minlength=len(unique_roots) + 1)[1:]
    return labels, sizes
