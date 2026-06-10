import math

import numpy as np
import pytest

from nav_goal_go2w_sim.dynamic_obstacles import (
    DynamicObstacle,
    spawn_random_obstacles,
    stamp_obstacles,
    step_dynamic_obstacles,
)
from nav_goal_go2w_sim.sim_core import OCCUPIED, Pose2D, World2D


def _make_world(width=20, height=20, resolution=0.5):
    return World2D(
        grid=np.zeros((height, width), dtype=np.uint8),
        resolution=resolution,
        origin=(0.0, 0.0),
    )


def _vertical_wall_world():
    world = _make_world()
    world.grid[:, 15] = OCCUPIED
    return world


def test_spawn_avoids_walls():
    world = _vertical_wall_world()
    obstacles = spawn_random_obstacles(
        world,
        count=4,
        radius=0.3,
        speed=0.2,
        seed=42,
        exclusion_pose=Pose2D(0.0, 0.0, 0.0),
        exclusion_radius=0.5,
    )
    assert len(obstacles) == 4
    for ob in obstacles:
        cell = world.world_to_grid(ob.x, ob.y)
        assert cell is not None
        gx, gy = cell
        assert world.grid[gy, gx] == 0


def test_spawn_respects_robot_exclusion():
    world = _make_world()
    obstacles = spawn_random_obstacles(
        world,
        count=5,
        radius=0.3,
        speed=0.2,
        seed=7,
        exclusion_pose=Pose2D(5.0, 5.0, 0.0),
        exclusion_radius=2.5,
    )
    assert len(obstacles) == 5
    for ob in obstacles:
        assert math.hypot(ob.x - 5.0, ob.y - 5.0) >= 2.5


def test_spawn_no_inter_overlap():
    world = _make_world()
    obstacles = spawn_random_obstacles(
        world,
        count=4,
        radius=0.5,
        speed=0.2,
        seed=11,
        exclusion_pose=Pose2D(0.0, 0.0, 0.0),
        exclusion_radius=0.5,
    )
    assert len(obstacles) == 4
    for i in range(len(obstacles)):
        for j in range(i + 1, len(obstacles)):
            distance = math.hypot(
                obstacles[i].x - obstacles[j].x,
                obstacles[i].y - obstacles[j].y,
            )
            assert distance >= 2.0 * 0.5


def test_bounce_x_axis():
    world = _vertical_wall_world()
    base = world.grid.copy()
    obstacle = DynamicObstacle(x=6.5, y=5.0, vx=2.0, vy=0.0, radius=0.4)
    step_dynamic_obstacles([obstacle], base, world, dt=0.5)
    assert obstacle.vx < 0.0
    assert obstacle.vy == 0.0


def test_bounce_corner():
    world = _make_world()
    world.grid[:, 15] = OCCUPIED
    world.grid[15, :] = OCCUPIED
    base = world.grid.copy()
    obstacle = DynamicObstacle(x=6.5, y=6.5, vx=2.0, vy=2.0, radius=0.4)
    step_dynamic_obstacles([obstacle], base, world, dt=0.5)
    assert obstacle.vx < 0.0
    assert obstacle.vy < 0.0


def test_step_never_tunnels():
    world = _vertical_wall_world()
    base = world.grid.copy()
    obstacle = DynamicObstacle(x=6.0, y=5.0, vx=5.0, vy=0.0, radius=0.4)
    for _ in range(100):
        step_dynamic_obstacles([obstacle], base, world, dt=0.05)
        cell = world.world_to_grid(obstacle.x, obstacle.y)
        assert cell is not None
        gx, gy = cell
        assert base[gy, gx] == 0


def test_stamping_round_trips():
    world = _make_world()
    base = world.grid.copy()
    obstacle = DynamicObstacle(x=4.0, y=4.0, vx=0.0, vy=0.0, radius=0.5)
    live = base.copy()
    stamp_obstacles(
        live,
        base,
        [obstacle],
        resolution=world.resolution,
        origin=world.origin,
    )
    cell = world.world_to_grid(obstacle.x, obstacle.y)
    assert cell is not None
    gx, gy = cell
    assert live[gy, gx] == OCCUPIED
    assert base[gy, gx] == 0
    np.copyto(live, base)
    assert live[gy, gx] == 0


def test_stuck_obstacle_initial_collision_no_move():
    world = _vertical_wall_world()
    base = world.grid.copy()
    obstacle = DynamicObstacle(x=7.5, y=5.0, vx=1.0, vy=1.0, radius=0.4)
    start_x, start_y = obstacle.x, obstacle.y
    start_vx, start_vy = obstacle.vx, obstacle.vy
    step_dynamic_obstacles([obstacle], base, world, dt=0.1)
    assert obstacle.x == pytest.approx(start_x)
    assert obstacle.y == pytest.approx(start_y)
    assert obstacle.vx == pytest.approx(start_vx)
    assert obstacle.vy == pytest.approx(start_vy)
