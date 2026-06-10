import numpy as np

from nav_goal_go2w_sim.sim_core import OCCUPIED
from nav_goal_go2w_sim.world_loader import world_from_dict


def test_world_loader_uses_bottom_left_grid_coordinates():
    world = world_from_dict({
        "name": "bottom_left",
        "resolution": 0.5,
        "origin": [-1.0, -1.0],
        "size": [4, 3],
        "spawn": [0.0, 0.0, 0.0],
        "walls": [
            {"type": "rect", "x": -1.0, "y": -1.0, "w": 0.5, "h": 0.5},
        ],
    })

    assert world.width == 4
    assert world.height == 3
    assert world.grid[0, 0] == OCCUPIED
    assert world.grid[0, 1] == 0
    assert world.world_to_grid(-0.75, -0.75) == (0, 0)
    assert world.grid_to_world(0, 0) == (-0.75, -0.75)
    assert world.spawn.x == 0.0


def test_fractal_noise_elevation_covers_positive_and_negative():
    world = world_from_dict({
        "name": "noise_test",
        "resolution": 0.05,
        "origin": [0.0, 0.0],
        "size": [100, 100],
        "spawn": [0.0, 0.0, 0.0],
        "elevation_features": [
            {"type": "fractal_noise", "amplitude": 0.10, "wavelength": 2.0,
             "octaves": 2, "persistence": 0.5, "seed": 0},
        ],
    })
    assert world.elevation.max() > 0.02
    assert world.elevation.min() < -0.02
    assert world.elevation.max() < 0.30
    assert world.grid.max() == 0


def test_gaussian_bump_creates_localised_peak():
    world = world_from_dict({
        "name": "bump_test",
        "resolution": 0.05,
        "origin": [-3.0, -3.0],
        "size": [120, 120],
        "spawn": [0.0, 0.0, 0.0],
        "elevation_features": [
            {"type": "gaussian_bump", "x": 0.0, "y": 0.0, "amplitude": 0.40, "sigma": 0.30},
        ],
    })
    # Peak is near centre
    cx, cy = world.world_to_grid(0.0, 0.0)
    assert world.elevation[cy, cx] > 0.35
    # Falls off with distance: 2 m away should be near zero
    fx, fy = world.world_to_grid(2.0, 0.0)
    assert world.elevation[fy, fx] < 0.01


def test_fractal_noise_is_deterministic():
    spec = {
        "name": "det",
        "resolution": 0.05,
        "origin": [0.0, 0.0],
        "size": [40, 40],
        "spawn": [0.0, 0.0, 0.0],
        "elevation_features": [
            {"type": "fractal_noise", "amplitude": 0.10, "wavelength": 2.0,
             "octaves": 2, "persistence": 0.5, "seed": 7},
        ],
    }
    w1 = world_from_dict(spec)
    w2 = world_from_dict(spec)
    assert np.array_equal(w1.elevation, w2.elevation)
