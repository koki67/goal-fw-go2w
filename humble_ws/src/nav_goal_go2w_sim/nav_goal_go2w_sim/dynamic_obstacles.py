"""Random-walk + bounce dynamic obstacles for the desktop 2D simulator."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

import numpy as np

from nav_goal_go2w_sim.sim_core import (
    OCCUPIED,
    Pose2D,
    World2D,
    stamp_disc,
)


@dataclass
class DynamicObstacle:
    """A circular dynamic obstacle moving in 2D."""

    x: float
    y: float
    vx: float
    vy: float
    radius: float


def _disc_blocked(
    base_grid: np.ndarray,
    world: World2D,
    x: float,
    y: float,
    radius: float,
) -> bool:
    """Return True if a disc centered at (x, y) overlaps an occupied cell or leaves the world."""

    height, width = base_grid.shape[0], base_grid.shape[1]
    radius_check = radius + world.resolution * 0.5
    span = max(int(math.ceil(radius_check / world.resolution)), 1)
    cx = int(math.floor((x - world.origin[0]) / world.resolution))
    cy = int(math.floor((y - world.origin[1]) / world.resolution))
    for gy in range(cy - span, cy + span + 1):
        for gx in range(cx - span, cx + span + 1):
            wx = world.origin[0] + (gx + 0.5) * world.resolution
            wy = world.origin[1] + (gy + 0.5) * world.resolution
            if math.hypot(wx - x, wy - y) > radius_check:
                continue
            if gx < 0 or gx >= width or gy < 0 or gy >= height:
                return True
            if base_grid[gy, gx] >= OCCUPIED:
                return True
    return False


def spawn_random_obstacles(
    base_world: World2D,
    count: int,
    *,
    radius: float,
    speed: float,
    seed: int,
    exclusion_pose: Pose2D,
    exclusion_radius: float,
) -> list[DynamicObstacle]:
    """Sample free positions inside the world, excluding the robot spawn neighborhood."""

    if count <= 0:
        return []
    if radius <= 0.0:
        raise ValueError("radius must be positive")
    if speed < 0.0:
        raise ValueError("speed must be non-negative")

    rng = random.Random(seed if seed != 0 else None)
    placed: list[DynamicObstacle] = []
    inter_clearance = 2.0 * radius + base_world.resolution
    max_attempts = max(200 * count, 200)

    for _ in range(max_attempts):
        if len(placed) == count:
            break
        gx = rng.randrange(base_world.width)
        gy = rng.randrange(base_world.height)
        wx, wy = base_world.grid_to_world(gx, gy)
        if _disc_blocked(base_world.grid, base_world, wx, wy, radius):
            continue
        if math.hypot(wx - exclusion_pose.x, wy - exclusion_pose.y) < exclusion_radius:
            continue
        if any(
            math.hypot(wx - ob.x, wy - ob.y) < inter_clearance for ob in placed
        ):
            continue
        heading = rng.uniform(-math.pi, math.pi)
        placed.append(
            DynamicObstacle(
                x=wx,
                y=wy,
                vx=speed * math.cos(heading),
                vy=speed * math.sin(heading),
                radius=radius,
            )
        )

    return placed


def step_dynamic_obstacles(
    obstacles: list[DynamicObstacle],
    base_grid: np.ndarray,
    world: World2D,
    dt: float,
) -> None:
    """Advance each obstacle by dt, bouncing component-wise off static walls."""

    if dt <= 0.0:
        return
    for ob in obstacles:
        if _disc_blocked(base_grid, world, ob.x, ob.y, ob.radius):
            continue

        nx = ob.x + ob.vx * dt
        if _disc_blocked(base_grid, world, nx, ob.y, ob.radius):
            ob.vx = -ob.vx
            nx = ob.x + ob.vx * dt
            if _disc_blocked(base_grid, world, nx, ob.y, ob.radius):
                nx = ob.x

        ny = ob.y + ob.vy * dt
        if _disc_blocked(base_grid, world, nx, ny, ob.radius):
            ob.vy = -ob.vy
            ny = ob.y + ob.vy * dt
            if _disc_blocked(base_grid, world, nx, ny, ob.radius):
                ny = ob.y

        ob.x = nx
        ob.y = ny


def stamp_obstacles(
    grid_dst: np.ndarray,
    base_grid: np.ndarray,
    obstacles: list[DynamicObstacle],
    *,
    resolution: float,
    origin: tuple[float, float],
) -> None:
    """Reset grid_dst to base_grid, then stamp each obstacle as an occupied disc."""

    np.copyto(grid_dst, base_grid)
    for ob in obstacles:
        stamp_disc(
            grid_dst,
            resolution=resolution,
            origin=origin,
            x=ob.x,
            y=ob.y,
            radius=ob.radius,
            value=OCCUPIED,
        )
