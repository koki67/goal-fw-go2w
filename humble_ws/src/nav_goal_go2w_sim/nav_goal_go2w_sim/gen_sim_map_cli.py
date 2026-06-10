"""CLI: generate a prepared map directory from a simulator world YAML.

The simulator world IS the ground truth, so the occupancy grid is rasterized
directly from it and the localization PCD is synthesized by extruding
occupied cells into wall points (z up to the raycaster's wall_z_max) plus
floor points on free cells. Output matches `prepare_map` exactly, so
map_servers.launch.py and the localizer consume it unchanged:

    ros2 run nav_goal_go2w_sim gen_sim_map --world open_room \
        --output maps/sim_open_room
"""

from __future__ import annotations

import argparse
import datetime
from pathlib import Path

import numpy as np
import yaml

from nav_goal_go2w_map import pcd_io
from nav_goal_go2w_map.grid_builder_core import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    BuiltGrid,
)
from nav_goal_go2w_map.pgm_yaml import write_map_files
from nav_goal_go2w_sim.sim_core import OCCUPIED as WORLD_OCCUPIED
from nav_goal_go2w_sim.world_loader import load_world


def _resolve_world(value: str) -> Path:
    candidate = Path(value).expanduser()
    if candidate.is_file():
        return candidate
    name = value if value.endswith(".yaml") else f"{value}.yaml"
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory("nav_goal_go2w_sim"))
        resolved = share / "worlds" / name
        if resolved.is_file():
            return resolved
    except Exception:
        pass
    local = Path(__file__).resolve().parents[1] / "worlds" / name
    if local.is_file():
        return local
    raise FileNotFoundError(f"world file not found: {value}")


def world_to_grid(world) -> BuiltGrid:
    """Rasterize the ground-truth world grid into prepare_map's format."""
    data = np.full(world.grid.shape, UNKNOWN, dtype=np.int8)
    data[world.grid < WORLD_OCCUPIED] = FREE
    data[world.grid >= WORLD_OCCUPIED] = OCCUPIED
    return BuiltGrid(
        data=data,
        resolution=world.resolution,
        origin_x=world.origin[0],
        origin_y=world.origin[1],
    )


def world_to_cloud(
    world,
    *,
    wall_z_max: float = 0.55,
    wall_z_step: float = 0.1,
    floor_step_cells: int = 2,
) -> np.ndarray:
    """Synthesize the map point cloud the simulated LiDAR would observe.

    Wall heights match sim_core.raycast_pointcloud's wall_z_max so live scans
    and the map share the same surfaces.
    """
    res = world.resolution
    origin_x, origin_y = world.origin
    points: list[np.ndarray] = []

    occupied_rows, occupied_cols = np.nonzero(world.grid >= WORLD_OCCUPIED)
    if len(occupied_rows):
        wall_x = origin_x + (occupied_cols + 0.5) * res
        wall_y = origin_y + (occupied_rows + 0.5) * res
        base_z = world.elevation[occupied_rows, occupied_cols]
        for z in np.arange(wall_z_step * 0.5, wall_z_max, wall_z_step):
            points.append(
                np.column_stack((wall_x, wall_y, base_z + z)).astype(np.float32)
            )

    free_rows, free_cols = np.nonzero(world.grid < WORLD_OCCUPIED)
    keep = (free_rows % floor_step_cells == 0) & (
        free_cols % floor_step_cells == 0
    )
    free_rows, free_cols = free_rows[keep], free_cols[keep]
    if len(free_rows):
        floor_x = origin_x + (free_cols + 0.5) * res
        floor_y = origin_y + (free_rows + 0.5) * res
        floor_z = world.elevation[free_rows, free_cols]
        points.append(
            np.column_stack((floor_x, floor_y, floor_z)).astype(np.float32)
        )

    if not points:
        raise ValueError("world produced no map points")
    return np.vstack(points)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="gen_sim_map", description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--world", required=True,
                        help="world name (e.g. open_room) or YAML path")
    parser.add_argument("--output", required=True, help="output map directory")
    parser.add_argument("--wall-z-max", type=float, default=0.55)
    parser.add_argument("--wall-z-step", type=float, default=0.1)
    parser.add_argument("--floor-step-cells", type=int, default=2)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    world_path = _resolve_world(args.world)
    world = load_world(world_path)
    output_dir = Path(args.output)

    grid = world_to_grid(world)
    write_map_files(output_dir, grid)

    cloud = world_to_cloud(
        world,
        wall_z_max=args.wall_z_max,
        wall_z_step=args.wall_z_step,
        floor_step_cells=args.floor_step_cells,
    )
    pcd_io.save_xyz(output_dir / "map.pcd", cloud)
    pcd_io.save_xyz(output_dir / "viz.pcd", cloud)

    metadata = {
        "name": output_dir.name,
        "source": str(world_path),
        "created": datetime.datetime.now().astimezone().isoformat(),
        "generator": "gen_sim_map",
        "points": int(len(cloud)),
        "grid": {
            "width": int(grid.width),
            "height": int(grid.height),
            "resolution": float(grid.resolution),
            "origin": [float(grid.origin_x), float(grid.origin_y)],
        },
    }
    with (output_dir / "metadata.yaml").open("w", encoding="utf-8") as handle:
        yaml.safe_dump(metadata, handle, sort_keys=False)
    print(f"sim map written to {output_dir} ({len(cloud)} points)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
