"""Load simple YAML worlds for the desktop 2D simulator."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from nav_goal_go2w_sim.sim_core import OCCUPIED, Pose2D, World2D, stamp_disc


def load_world(path: str | Path) -> World2D:
    """Load a World2D from a YAML file."""

    world_path = Path(path)
    with world_path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    return world_from_dict(data, name_fallback=world_path.stem)


def world_from_dict(data: dict[str, Any], *, name_fallback: str = "world") -> World2D:
    """Create a World2D from the simulator YAML schema."""

    resolution = float(data["resolution"])
    origin_raw = data.get("origin", [0.0, 0.0])
    size_raw = data["size"]
    width = int(size_raw[0])
    height = int(size_raw[1])
    grid = np.zeros((height, width), dtype=np.uint8)
    elevation = np.zeros((height, width), dtype=np.float32)

    world = World2D(
        grid=grid,
        resolution=resolution,
        origin=(float(origin_raw[0]), float(origin_raw[1])),
        name=str(data.get("name", name_fallback)),
        spawn=_pose_from_list(data.get("spawn", [0.0, 0.0, 0.0])),
        elevation=elevation,
    )

    for wall in data.get("walls", []):
        wall_type = wall.get("type")
        if wall_type == "rect":
            _rasterize_rect(world, wall)
        elif wall_type == "segment":
            _rasterize_segment(world, wall)
        else:
            raise ValueError(f"unsupported wall type: {wall_type}")

    if "elevation" in data:
        raw_elevation = np.asarray(data["elevation"], dtype=np.float32)
        if raw_elevation.shape != world.grid.shape:
            raise ValueError("elevation must match world size as [height][width]")
        world.elevation[:, :] = raw_elevation

    for feature in data.get("elevation_features", []):
        feature_type = feature.get("type")
        if feature_type == "rect":
            _paint_elevation_rect(world, feature)
        elif feature_type == "ramp":
            _paint_elevation_ramp(world, feature)
        elif feature_type == "fractal_noise":
            _paint_elevation_fractal_noise(world, feature)
        elif feature_type == "gaussian_bump":
            _paint_elevation_gaussian_bump(world, feature)
        else:
            raise ValueError(f"unsupported elevation feature type: {feature_type}")

    return world


def _pose_from_list(raw: list[float]) -> Pose2D:
    return Pose2D(float(raw[0]), float(raw[1]), float(raw[2]))


def _rasterize_rect(world: World2D, wall: dict[str, Any]) -> None:
    x0 = float(wall["x"])
    y0 = float(wall["y"])
    x1 = x0 + float(wall["w"])
    y1 = y0 + float(wall["h"])
    min_x, max_x = sorted((x0, x1))
    min_y, max_y = sorted((y0, y1))

    gx0 = max(0, int(math.floor((min_x - world.origin[0]) / world.resolution)))
    gy0 = max(0, int(math.floor((min_y - world.origin[1]) / world.resolution)))
    gx1 = min(
        world.width - 1,
        int(math.ceil((max_x - world.origin[0]) / world.resolution)) - 1,
    )
    gy1 = min(
        world.height - 1,
        int(math.ceil((max_y - world.origin[1]) / world.resolution)) - 1,
    )
    if gx0 > gx1 or gy0 > gy1:
        return
    world.grid[gy0:gy1 + 1, gx0:gx1 + 1] = OCCUPIED


def _rasterize_segment(world: World2D, wall: dict[str, Any]) -> None:
    x0 = float(wall["x1"])
    y0 = float(wall["y1"])
    x1 = float(wall["x2"])
    y1 = float(wall["y2"])
    thickness = float(wall.get("thickness", world.resolution))
    length = math.hypot(x1 - x0, y1 - y0)
    samples = max(int(math.ceil(length / (world.resolution * 0.5))), 1)

    for i in range(samples + 1):
        t = i / samples
        x = x0 + (x1 - x0) * t
        y = y0 + (y1 - y0) * t
        _mark_disc(world, x, y, thickness * 0.5)


def _mark_disc(world: World2D, x: float, y: float, radius: float) -> None:
    stamp_disc(
        world.grid,
        resolution=world.resolution,
        origin=world.origin,
        x=x,
        y=y,
        radius=radius,
        value=OCCUPIED,
    )


def _grid_bounds(world: World2D, feature: dict[str, Any]) -> tuple[int, int, int, int] | None:
    x0 = float(feature["x"])
    y0 = float(feature["y"])
    x1 = x0 + float(feature["w"])
    y1 = y0 + float(feature["h"])
    min_x, max_x = sorted((x0, x1))
    min_y, max_y = sorted((y0, y1))
    gx0 = max(0, int(math.floor((min_x - world.origin[0]) / world.resolution)))
    gy0 = max(0, int(math.floor((min_y - world.origin[1]) / world.resolution)))
    gx1 = min(world.width - 1, int(math.ceil((max_x - world.origin[0]) / world.resolution)) - 1)
    gy1 = min(world.height - 1, int(math.ceil((max_y - world.origin[1]) / world.resolution)) - 1)
    if gx0 > gx1 or gy0 > gy1:
        return None
    return gx0, gy0, gx1, gy1


def _paint_elevation_rect(world: World2D, feature: dict[str, Any]) -> None:
    bounds = _grid_bounds(world, feature)
    if bounds is None:
        return
    gx0, gy0, gx1, gy1 = bounds
    world.elevation[gy0:gy1 + 1, gx0:gx1 + 1] = float(feature["z"])


def _paint_elevation_ramp(world: World2D, feature: dict[str, Any]) -> None:
    bounds = _grid_bounds(world, feature)
    if bounds is None:
        return
    gx0, gy0, gx1, gy1 = bounds
    axis = str(feature.get("axis", "x"))
    start_z = float(feature.get("start_z", 0.0))
    end_z = float(feature["end_z"])
    if axis not in {"x", "y"}:
        raise ValueError("ramp axis must be x or y")
    for gy in range(gy0, gy1 + 1):
        for gx in range(gx0, gx1 + 1):
            if axis == "x":
                denom = max(gx1 - gx0, 1)
                ratio = (gx - gx0) / denom
            else:
                denom = max(gy1 - gy0, 1)
                ratio = (gy - gy0) / denom
            world.elevation[gy, gx] = start_z + ratio * (end_z - start_z)


def _paint_elevation_fractal_noise(world: World2D, feature: dict[str, Any]) -> None:
    """Paint octave value noise (bilinear-upsampled random grid) over the entire elevation grid."""
    amplitude = float(feature.get("amplitude", 0.15))
    wavelength = float(feature.get("wavelength", 3.0))
    octaves = int(feature.get("octaves", 4))
    persistence = float(feature.get("persistence", 0.6))
    seed = int(feature.get("seed", 0))

    rng = np.random.default_rng(seed)
    h, w = world.elevation.shape
    acc = np.zeros((h, w), dtype=np.float32)
    amp = amplitude
    wl = wavelength

    for _ in range(octaves):
        scale = max(2, int(round(wl / world.resolution)))
        ch = h // scale + 2
        cw = w // scale + 2
        coarse = rng.standard_normal((ch, cw)).astype(np.float32)

        row_f = np.linspace(0.0, ch - 1, h)
        col_f = np.linspace(0.0, cw - 1, w)
        r0 = np.floor(row_f).astype(np.int32).clip(0, ch - 2)
        c0 = np.floor(col_f).astype(np.int32).clip(0, cw - 2)
        dr = (row_f - r0).astype(np.float32)[:, np.newaxis]
        dc = (col_f - c0).astype(np.float32)[np.newaxis, :]
        r1 = r0 + 1
        c1 = c0 + 1

        layer = (coarse[r0[:, np.newaxis], c0[np.newaxis, :]] * (1 - dr) * (1 - dc)
                 + coarse[r1[:, np.newaxis], c0[np.newaxis, :]] * dr * (1 - dc)
                 + coarse[r0[:, np.newaxis], c1[np.newaxis, :]] * (1 - dr) * dc
                 + coarse[r1[:, np.newaxis], c1[np.newaxis, :]] * dr * dc)

        acc += amp * layer
        amp *= persistence
        wl *= 0.5

    world.elevation[:, :] += acc


def _paint_elevation_gaussian_bump(world: World2D, feature: dict[str, Any]) -> None:
    """Add a Gaussian elevation mound centred at (x, y)."""
    cx = float(feature["x"])
    cy = float(feature["y"])
    amplitude = float(feature["amplitude"])
    sigma = float(feature.get("sigma", 0.5))
    sigma_x = float(feature.get("sigma_x", sigma))
    sigma_y = float(feature.get("sigma_y", sigma))

    h, w = world.elevation.shape
    xs = world.origin[0] + (np.arange(w) + 0.5) * world.resolution
    ys = world.origin[1] + (np.arange(h) + 0.5) * world.resolution
    XX, YY = np.meshgrid(xs, ys)

    dx = (XX - cx) / sigma_x
    dy = (YY - cy) / sigma_y
    world.elevation[:, :] += (amplitude * np.exp(-0.5 * (dx ** 2 + dy ** 2))).astype(np.float32)
