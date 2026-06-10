"""Pure 2D simulator primitives for desktop frontier testing."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

import numpy as np


FREE = 0
OCCUPIED = 100


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Twist2D:
    vx: float
    vy: float
    wz: float


@dataclass(frozen=True)
class RaycastResult:
    ranges: np.ndarray
    seen_free_cells: set[tuple[int, int]]
    hit_cells: set[tuple[int, int]]


class World2D:
    """2D occupancy grid with bottom-left origin and optional floor elevation."""

    def __init__(
        self,
        *,
        grid: np.ndarray,
        resolution: float,
        origin: tuple[float, float],
        name: str = "world",
        spawn: Pose2D | None = None,
        elevation: np.ndarray | None = None,
    ) -> None:
        if grid.ndim != 2:
            raise ValueError("grid must be a 2D array")
        if resolution <= 0.0:
            raise ValueError("resolution must be positive")
        self.grid = grid.astype(np.uint8, copy=True)
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))
        self.name = name
        self.spawn = spawn or Pose2D(0.0, 0.0, 0.0)
        if elevation is None:
            self.elevation = np.zeros_like(self.grid, dtype=np.float32)
        else:
            if elevation.shape != self.grid.shape:
                raise ValueError("elevation must match grid shape")
            self.elevation = elevation.astype(np.float32, copy=True)

    @property
    def width(self) -> int:
        return int(self.grid.shape[1])

    @property
    def height(self) -> int:
        return int(self.grid.shape[0])

    def world_to_grid(self, x: float, y: float) -> tuple[int, int] | None:
        gx = int(math.floor((x - self.origin[0]) / self.resolution))
        gy = int(math.floor((y - self.origin[1]) / self.resolution))
        if gx < 0 or gy < 0 or gx >= self.width or gy >= self.height:
            return None
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> tuple[float, float]:
        return (
            self.origin[0] + (gx + 0.5) * self.resolution,
            self.origin[1] + (gy + 0.5) * self.resolution,
        )

    def is_occupied(self, x: float, y: float) -> bool:
        cell = self.world_to_grid(x, y)
        if cell is None:
            return True
        gx, gy = cell
        return bool(self.grid[gy, gx] >= OCCUPIED)

    def occupied_cells(self) -> Iterable[tuple[int, int]]:
        ys, xs = np.nonzero(self.grid >= OCCUPIED)
        for gx, gy in zip(xs.tolist(), ys.tolist()):
            yield gx, gy

    def elevation_at_cell(self, gx: int, gy: int) -> float:
        return float(self.elevation[gy, gx])


def stamp_disc(
    grid: np.ndarray,
    *,
    resolution: float,
    origin: tuple[float, float],
    x: float,
    y: float,
    radius: float,
    value: int = OCCUPIED,
) -> None:
    """Stamp a filled disc into a uint8 grid in place."""

    if grid.ndim != 2:
        raise ValueError("grid must be a 2D array")
    if resolution <= 0.0:
        raise ValueError("resolution must be positive")
    height, width = int(grid.shape[0]), int(grid.shape[1])
    span = max(int(math.ceil(radius / resolution)), 1)
    cx = int(math.floor((x - origin[0]) / resolution))
    cy = int(math.floor((y - origin[1]) / resolution))
    if cx + span < 0 or cy + span < 0 or cx - span >= width or cy - span >= height:
        return
    radius_check = radius + resolution * 0.5
    for gy in range(max(0, cy - span), min(height, cy + span + 1)):
        wy = origin[1] + (gy + 0.5) * resolution
        dy = wy - y
        for gx in range(max(0, cx - span), min(width, cx + span + 1)):
            wx = origin[0] + (gx + 0.5) * resolution
            if math.hypot(wx - x, dy) <= radius_check:
                grid[gy, gx] = value


def clamp_twist(
    twist: Twist2D,
    *,
    vx_max: float,
    vy_max: float,
    wz_max: float,
) -> Twist2D:
    """Clamp each velocity axis independently."""

    return Twist2D(
        vx=max(-vx_max, min(vx_max, twist.vx)),
        vy=max(-vy_max, min(vy_max, twist.vy)),
        wz=max(-wz_max, min(wz_max, twist.wz)),
    )


def integrate_omni(pose: Pose2D, body_twist: Twist2D, dt: float) -> Pose2D:
    """Integrate an omni base body-frame twist with a simple Euler step."""

    cos_yaw = math.cos(pose.yaw)
    sin_yaw = math.sin(pose.yaw)
    world_vx = cos_yaw * body_twist.vx - sin_yaw * body_twist.vy
    world_vy = sin_yaw * body_twist.vx + cos_yaw * body_twist.vy
    return Pose2D(
        x=pose.x + world_vx * dt,
        y=pose.y + world_vy * dt,
        yaw=normalize_angle(pose.yaw + body_twist.wz * dt),
    )


def realized_twist(prev_pose: Pose2D, new_pose: Pose2D, dt: float) -> Twist2D:
    """Compute body-frame velocity that moved prev_pose to new_pose."""

    if dt <= 0.0:
        return Twist2D(0.0, 0.0, 0.0)
    dx = (new_pose.x - prev_pose.x) / dt
    dy = (new_pose.y - prev_pose.y) / dt
    cos_yaw = math.cos(prev_pose.yaw)
    sin_yaw = math.sin(prev_pose.yaw)
    body_vx = cos_yaw * dx + sin_yaw * dy
    body_vy = -sin_yaw * dx + cos_yaw * dy
    return Twist2D(
        vx=body_vx,
        vy=body_vy,
        wz=normalize_angle(new_pose.yaw - prev_pose.yaw) / dt,
    )


def check_collision(
    world: World2D,
    pose: Pose2D,
    footprint: Iterable[tuple[float, float]],
) -> bool:
    """Return true if the robot footprint intersects an occupied cell."""

    if world.is_occupied(pose.x, pose.y):
        return True
    points = list(footprint)
    if len(points) < 3:
        raise ValueError("footprint must contain at least 3 points")

    cos_yaw = math.cos(pose.yaw)
    sin_yaw = math.sin(pose.yaw)

    def occupied_body_point(body_x: float, body_y: float) -> bool:
        world_x = pose.x + cos_yaw * body_x - sin_yaw * body_y
        world_y = pose.y + sin_yaw * body_x + cos_yaw * body_y
        return world.is_occupied(world_x, world_y)

    def point_in_footprint(body_x: float, body_y: float) -> bool:
        inside = False
        previous = points[-1]
        for current in points:
            crosses_y = (current[1] > body_y) != (previous[1] > body_y)
            if crosses_y:
                slope_x = (
                    (previous[0] - current[0])
                    * (body_y - current[1])
                    / (previous[1] - current[1])
                    + current[0]
                )
                if body_x < slope_x:
                    inside = not inside
            previous = current
        return inside

    for index, start in enumerate(points):
        end = points[(index + 1) % len(points)]
        edge_length = math.hypot(end[0] - start[0], end[1] - start[1])
        samples = max(int(math.ceil(edge_length / max(world.resolution * 0.5, 0.01))), 1)
        for sample in range(samples + 1):
            ratio = sample / samples
            body_x = start[0] + (end[0] - start[0]) * ratio
            body_y = start[1] + (end[1] - start[1]) * ratio
            if occupied_body_point(body_x, body_y):
                return True

    min_x = min(point[0] for point in points)
    max_x = max(point[0] for point in points)
    min_y = min(point[1] for point in points)
    max_y = max(point[1] for point in points)
    step = max(world.resolution * 0.5, 0.01)
    x = min_x
    while x <= max_x + 1.0e-9:
        y = min_y
        while y <= max_y + 1.0e-9:
            if point_in_footprint(x, y) and occupied_body_point(x, y):
                return True
            y += step
        x += step
    return False


def raycast_scan(
    world: World2D,
    pose: Pose2D,
    *,
    angle_min: float,
    angle_max: float,
    angle_inc: float,
    range_min: float,
    range_max: float,
) -> RaycastResult:
    """Raycast a 2D LaserScan from the robot pose through the world."""

    if angle_inc <= 0.0:
        raise ValueError("angle_inc must be positive")
    beam_count = int(math.floor((angle_max - angle_min) / angle_inc)) + 1
    ranges = np.full(beam_count, math.inf, dtype=np.float32)
    seen_free: set[tuple[int, int]] = set()
    hit_cells: set[tuple[int, int]] = set()
    step = max(world.resolution * 0.5, 0.01)

    for beam_index in range(beam_count):
        angle = pose.yaw + angle_min + beam_index * angle_inc
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        traveled = range_min
        previous_cell: tuple[int, int] | None = None

        while traveled <= range_max + 1.0e-9:
            x = pose.x + cos_a * traveled
            y = pose.y + sin_a * traveled
            cell = world.world_to_grid(x, y)
            if cell is None:
                ranges[beam_index] = traveled
                break
            if world.grid[cell[1], cell[0]] >= OCCUPIED:
                ranges[beam_index] = traveled
                hit_cells.add(cell)
                break
            if cell != previous_cell:
                seen_free.add(cell)
                previous_cell = cell
            traveled += step

    return RaycastResult(ranges=ranges, seen_free_cells=seen_free, hit_cells=hit_cells)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def raycast_pointcloud(
    world: World2D,
    pose: Pose2D,
    params: dict[str, float] | None = None,
) -> np.ndarray:
    """Raycast an odom-frame multi-ring XYZ cloud through occupancy/elevation."""

    p = params or {}
    range_max = float(p.get("range_max", 8.0))
    num_rings = int(p.get("num_rings", 16))
    vfov_deg = float(p.get("vfov_deg", 30.0))
    hfov_deg = float(p.get("hfov_deg", 360.0))
    az_step_deg = float(p.get("az_step_deg", 0.5))
    sensor_height_m = float(p.get("sensor_height_m", 0.3))
    wall_z_max = float(p.get("wall_z_max", 0.55))
    if (
        range_max <= 0.0
        or num_rings <= 0
        or vfov_deg <= 0.0
        or hfov_deg <= 0.0
        or az_step_deg <= 0.0
        or sensor_height_m < 0.0
        or wall_z_max <= 0.0
    ):
        raise ValueError("pointcloud raycast parameters are out of range")

    robot_cell = world.world_to_grid(pose.x, pose.y)
    if robot_cell is None:
        return np.empty((0, 3), dtype=np.float32)

    vfov_half = math.radians(vfov_deg) * 0.5
    el_angles = np.linspace(-vfov_half, vfov_half, num_rings, dtype=np.float32)
    tan_els = np.tan(el_angles)
    sensor_z = world.elevation_at_cell(*robot_cell) + sensor_height_m
    h_step = max(world.resolution * 0.5, 0.01)
    azimuth_count = max(int(math.floor(hfov_deg / az_step_deg)), 1)
    az_step = math.radians(az_step_deg)
    az_start = pose.yaw if hfov_deg >= 360.0 else pose.yaw - math.radians(hfov_deg) * 0.5

    points: list[tuple[float, float, float]] = []
    for azimuth_index in range(azimuth_count):
        azimuth = az_start + azimuth_index * az_step
        cos_az = math.cos(azimuth)
        sin_az = math.sin(azimuth)
        hit = np.zeros(num_rings, dtype=bool)
        previous_cell: tuple[int, int] | None = None
        traveled = h_step

        while not bool(np.all(hit)) and traveled <= range_max + 1.0e-9:
            px = pose.x + cos_az * traveled
            py = pose.y + sin_az * traveled
            cell = world.world_to_grid(px, py)
            if cell is None:
                break
            if cell == previous_cell:
                traveled += h_step
                continue
            previous_cell = cell

            gx, gy = cell
            base_z = world.elevation_at_cell(gx, gy)
            occupied = world.grid[gy, gx] >= OCCUPIED
            pzs = sensor_z + tan_els * traveled
            floor_mask = ~hit & (pzs <= base_z)
            wall_mask = (
                ~hit
                & occupied
                & (pzs > base_z)
                & (pzs <= base_z + wall_z_max)
            )
            for _ring in np.where(floor_mask)[0]:
                points.append((px, py, base_z))
            for ring in np.where(wall_mask)[0]:
                points.append((px, py, float(pzs[ring])))
            hit |= floor_mask | wall_mask
            traveled += h_step

    if not points:
        return np.empty((0, 3), dtype=np.float32)
    return np.asarray(points, dtype=np.float32).reshape(-1, 3)


def apply_odom_drift(
    twist: Twist2D, drift_x_per_m: float, drift_yaw_per_m: float
) -> Twist2D:
    """Corrupt a realized body twist the way drifting odometry would.

    drift_x_per_m scales longitudinal motion (wheel-radius style error);
    drift_yaw_per_m adds yaw rate proportional to translational speed
    (heading drift per meter traveled). With both zero this is identity.
    """
    speed = math.hypot(twist.vx, twist.vy)
    return Twist2D(
        twist.vx * (1.0 + drift_x_per_m),
        twist.vy,
        twist.wz + drift_yaw_per_m * speed,
    )


def world_points_to_drifted_odom(
    points: np.ndarray, true_pose: Pose2D, odom_pose: Pose2D
) -> np.ndarray:
    """Re-express world-frame sensor points in the drifted odom frame.

    The sensor really is at ``true_pose`` (raycasting happens in the world),
    but a drifting odometry believes the robot is at ``odom_pose``; the cloud
    it would publish is the world geometry seen through that error:
    p_odom = T_odom_base(odom_pose) * T_base_world(true_pose) * p_world.
    """
    points = np.asarray(points, dtype=np.float32)
    if points.size == 0:
        return points.reshape(0, 3)
    delta_yaw = odom_pose.yaw - true_pose.yaw
    cos_d, sin_d = math.cos(delta_yaw), math.sin(delta_yaw)
    x = points[:, 0] - true_pose.x
    y = points[:, 1] - true_pose.y
    out = np.empty_like(points)
    out[:, 0] = cos_d * x - sin_d * y + odom_pose.x
    out[:, 1] = sin_d * x + cos_d * y + odom_pose.y
    out[:, 2] = points[:, 2]
    return out
