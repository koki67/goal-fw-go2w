"""Pure-numpy 2.5D terrain traversability mapping."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


UNKNOWN_COST = np.int8(-1)
LETHAL_COST = np.int8(99)


@dataclass(frozen=True)
class TerrainConfig:
    resolution: float = 0.10
    width_m: float = 40.0
    height_m: float = 40.0
    z_band_below: float = 2.0
    z_band_above: float = 2.0
    step_free: float = 0.05
    step_max: float = 0.20
    slope_free_deg: float = 10.0
    slope_max_deg: float = 30.0
    roughness_free: float = 0.03
    roughness_max: float = 0.08

    def __post_init__(self) -> None:
        if self.resolution <= 0.0:
            raise ValueError("resolution must be positive")
        if self.width_m <= 0.0 or self.height_m <= 0.0:
            raise ValueError("map dimensions must be positive")
        for lower, upper, name in (
            (self.step_free, self.step_max, "step"),
            (self.slope_free_deg, self.slope_max_deg, "slope"),
            (self.roughness_free, self.roughness_max, "roughness"),
        ):
            if lower < 0.0 or upper <= lower:
                raise ValueError(f"{name} thresholds must satisfy 0 <= free < max")


class TerrainMap:
    """Fixed-extent latest-observation-wins traversability raster."""

    def __init__(self, config: TerrainConfig | None = None) -> None:
        self.config = config or TerrainConfig()
        self.width = int(round(self.config.width_m / self.config.resolution))
        self.height = int(round(self.config.height_m / self.config.resolution))
        if self.width <= 0 or self.height <= 0:
            raise ValueError("map dimensions must contain at least one cell")

        shape = (self.height, self.width)
        self.elevation = np.full(shape, np.nan, dtype=np.float32)
        self.relief = np.full(shape, np.nan, dtype=np.float32)
        self.cost = np.full(shape, UNKNOWN_COST, dtype=np.int8)
        self.origin_x: float | None = None
        self.origin_y: float | None = None
        self.last_filtered_points = np.empty((0, 3), dtype=np.float32)

    @property
    def initialized(self) -> bool:
        return self.origin_x is not None and self.origin_y is not None

    def initialize_origin(self, robot_x: float, robot_y: float) -> None:
        """Center the fixed map once around the first robot pose."""
        if self.initialized:
            return
        self.origin_x = float(robot_x) - 0.5 * self.width * self.config.resolution
        self.origin_y = float(robot_y) - 0.5 * self.height * self.config.resolution

    def integrate_cloud(
        self,
        points_xyz: np.ndarray,
        robot_z: float,
        robot_xy: tuple[float, float] | None = None,
    ) -> np.ndarray:
        """Integrate an odom-frame cloud and return lethal obstacle points."""
        if not self.initialized:
            self.initialize_origin(*(robot_xy or (0.0, 0.0)))

        points = np.asarray(points_xyz, dtype=np.float32)
        if points.size == 0:
            self.last_filtered_points = np.empty((0, 3), dtype=np.float32)
            return np.empty((0, 3), dtype=np.float32)
        if points.ndim != 2 or points.shape[1] < 3:
            raise ValueError("points_xyz must have shape (N, 3)")
        points = points[:, :3]

        finite = np.isfinite(points).all(axis=1)
        z_min = float(robot_z) - self.config.z_band_below
        z_max = float(robot_z) + self.config.z_band_above
        band = finite & (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
        points = points[band]
        if points.size == 0:
            self.last_filtered_points = np.empty((0, 3), dtype=np.float32)
            return np.empty((0, 3), dtype=np.float32)

        cols, rows, in_bounds = self._point_cells(points)
        points = points[in_bounds]
        cols = cols[in_bounds]
        rows = rows[in_bounds]
        self.last_filtered_points = points.astype(np.float32, copy=True)
        if points.size == 0:
            return np.empty((0, 3), dtype=np.float32)

        flat = rows * self.width + cols
        unique_flat = np.unique(flat)
        scan_min = np.full(self.width * self.height, np.inf, dtype=np.float32)
        scan_max = np.full(self.width * self.height, -np.inf, dtype=np.float32)
        np.minimum.at(scan_min, flat, points[:, 2])
        np.maximum.at(scan_max, flat, points[:, 2])

        touched_rows, touched_cols = np.divmod(unique_flat, self.width)
        self.elevation[touched_rows, touched_cols] = scan_min[unique_flat]
        self.relief[touched_rows, touched_cols] = (
            scan_max[unique_flat] - scan_min[unique_flat]
        )

        affected = self._expand_cells(touched_rows, touched_cols)
        self._classify_cells(affected)

        point_costs = self.cost[rows, cols]
        obstacle_points = points[point_costs == LETHAL_COST]
        drop_points = self._dropoff_points(affected)
        if drop_points.size:
            obstacle_points = np.vstack((obstacle_points, drop_points))
        return obstacle_points.astype(np.float32, copy=False)

    def occupancy_data(self) -> np.ndarray:
        """Return an OccupancyGrid-compatible row-major copy."""
        return self.cost.ravel(order="C").copy()

    def _point_cells(
        self, points: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        cols = np.floor(
            (points[:, 0] - float(self.origin_x)) / self.config.resolution
        ).astype(np.int64)
        rows = np.floor(
            (points[:, 1] - float(self.origin_y)) / self.config.resolution
        ).astype(np.int64)
        in_bounds = (
            (cols >= 0)
            & (cols < self.width)
            & (rows >= 0)
            & (rows < self.height)
        )
        return cols, rows, in_bounds

    def _expand_cells(self, rows: np.ndarray, cols: np.ndarray) -> np.ndarray:
        mask = np.zeros((self.height, self.width), dtype=bool)
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                rr = rows + dr
                cc = cols + dc
                valid = (
                    (rr >= 0)
                    & (rr < self.height)
                    & (cc >= 0)
                    & (cc < self.width)
                )
                mask[rr[valid], cc[valid]] = True
        return np.argwhere(mask)

    def _classify_cells(self, cells: np.ndarray) -> None:
        affected = np.zeros((self.height, self.width), dtype=bool)
        affected[cells[:, 0], cells[:, 1]] = True
        observed = np.isfinite(self.elevation)
        elevation = np.nan_to_num(self.elevation, nan=0.0)

        step_height = np.zeros_like(elevation)
        slope_tan = np.zeros_like(elevation)
        count = np.zeros_like(elevation)
        sum_z = np.zeros_like(elevation)
        sum_z2 = np.zeros_like(elevation)

        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                shifted = np.roll(elevation, shift=(-dr, -dc), axis=(0, 1))
                valid = np.roll(observed, shift=(-dr, -dc), axis=(0, 1))
                if dr < 0:
                    valid[: -dr, :] = False
                elif dr > 0:
                    valid[-dr :, :] = False
                if dc < 0:
                    valid[:, : -dc] = False
                elif dc > 0:
                    valid[:, -dc :] = False

                count += valid
                sum_z += np.where(valid, shifted, 0.0)
                sum_z2 += np.where(valid, shifted * shifted, 0.0)
                if dr == 0 and dc == 0:
                    continue
                comparable = observed & valid
                delta = np.where(comparable, np.abs(shifted - elevation), 0.0)
                step_height = np.maximum(step_height, delta)
                distance = self.config.resolution * math.hypot(dr, dc)
                slope_tan = np.maximum(slope_tan, delta / distance)

        mean = np.divide(sum_z, count, out=np.zeros_like(sum_z), where=count > 0)
        variance = np.divide(sum_z2, count, out=np.zeros_like(sum_z2), where=count > 0) - mean * mean
        roughness = np.sqrt(np.maximum(variance, 0.0))
        slope_deg = np.degrees(np.arctan(slope_tan))
        continuity_limit = 2.0 * self.config.resolution * math.tan(
            math.radians(self.config.slope_max_deg)
        )
        discontinuity = step_height > continuity_limit
        slope_deg[discontinuity] = 0.0

        obstruction_cost = np.where(
            self.relief > self.config.step_max + 1.0e-6, int(LETHAL_COST), 0
        )
        step_cost = self._scaled_cost_array(
            step_height, self.config.step_free, self.config.step_max
        )
        slope_cost = self._scaled_cost_array(
            slope_deg, self.config.slope_free_deg, self.config.slope_max_deg
        )
        roughness_cost = self._scaled_cost_array(
            roughness, self.config.roughness_free, self.config.roughness_max
        )
        graded_step = discontinuity & (step_height <= self.config.step_max + 1.0e-6)
        roughness_cost[graded_step] = np.minimum(roughness_cost[graded_step], 98)
        classified = np.maximum.reduce(
            (obstruction_cost, step_cost, slope_cost, roughness_cost)
        ).astype(np.int8)
        self.cost[affected & observed] = classified[affected & observed]
        self.cost[affected & ~observed] = UNKNOWN_COST


    def _scaled_cost_array(self, values: np.ndarray, free: float, maximum: float) -> np.ndarray:
        costs = np.zeros(values.shape, dtype=np.int16)
        graded = (values > free) & (values <= maximum + 1.0e-6)
        costs[graded] = np.ceil(
            98.0 * (values[graded] - free) / (maximum - free)
        ).astype(np.int16)
        costs[values > maximum + 1.0e-6] = int(LETHAL_COST)
        return np.clip(costs, 0, int(LETHAL_COST))

    def _dropoff_points(self, affected: np.ndarray) -> np.ndarray:
        affected_mask = np.zeros((self.height, self.width), dtype=bool)
        affected_mask[affected[:, 0], affected[:, 1]] = True
        point_sets: list[np.ndarray] = []
        for dr, dc in ((0, 1), (1, 0), (1, 1), (1, -1)):
            row0 = slice(0, self.height - dr)
            row1 = slice(dr, self.height)
            if dc >= 0:
                col0 = slice(0, self.width - dc)
                col1 = slice(dc, self.width)
                col_offset = 0
            else:
                col0 = slice(-dc, self.width)
                col1 = slice(0, self.width + dc)
                col_offset = -dc

            z0 = self.elevation[row0, col0]
            z1 = self.elevation[row1, col1]
            pair_affected = affected_mask[row0, col0] | affected_mask[row1, col1]
            drops = (
                pair_affected
                & np.isfinite(z0)
                & np.isfinite(z1)
                & (np.abs(z1 - z0) > self.config.step_max + 1.0e-6)
            )
            rows, cols = np.nonzero(drops)
            if rows.size == 0:
                continue
            rows0 = rows
            cols0 = cols + col_offset
            rows1 = rows0 + dr
            cols1 = cols0 + dc
            x = float(self.origin_x) + (cols0 + cols1 + 1) * 0.5 * self.config.resolution
            y = float(self.origin_y) + (rows0 + rows1 + 1) * 0.5 * self.config.resolution
            z = np.minimum(z0[drops], z1[drops])
            point_sets.append(np.column_stack((x, y, z)).astype(np.float32))

        if not point_sets:
            return np.empty((0, 3), dtype=np.float32)
        return np.vstack(point_sets)
