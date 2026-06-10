"""CLI: convert a raw PCD into a ready-to-serve map directory.

Outputs in ``--output``:
  map.pcd        localization map (downsampled at --loc-voxel)
  viz.pcd        lightweight RViz cloud (downsampled at --viz-voxel)
  grid.pgm/.yaml 2D occupancy grid for the Nav2 global costmap
  metadata.yaml  provenance and all parameters used
"""

from __future__ import annotations

import argparse
import datetime
from pathlib import Path

import yaml

from nav_goal_go2w_map import pcd_io
from nav_goal_go2w_map.grid_builder_core import GridConfig, build_grid
from nav_goal_go2w_map.map_prep_core import (
    crop_box,
    remove_sparse_voxels,
    voxel_downsample,
)
from nav_goal_go2w_map.pgm_yaml import write_map_files


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="prepare_map", description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--input", required=True, help="source PCD file")
    parser.add_argument("--output", required=True, help="output map directory")
    parser.add_argument("--resolution", type=float, default=0.05,
                        help="occupancy grid resolution [m] (default 0.05)")
    parser.add_argument("--loc-voxel", type=float, default=0.20,
                        help="localization map voxel size [m] (default 0.20)")
    parser.add_argument("--viz-voxel", type=float, default=0.40,
                        help="RViz cloud voxel size [m] (default 0.40)")
    parser.add_argument("--z-min", type=float, default=None,
                        help="crop: discard points below this z [m]")
    parser.add_argument("--z-max", type=float, default=None,
                        help="crop: discard points above this z [m]")
    parser.add_argument("--outlier-voxel", type=float, default=0.30,
                        help="outlier removal voxel size [m]; 0 disables")
    parser.add_argument("--outlier-min-points", type=int, default=4,
                        help="min points in a 3x3x3 voxel neighborhood")
    parser.add_argument("--obstacle-z-min", type=float, default=0.15,
                        help="obstacle band lower bound above ground [m]")
    parser.add_argument("--obstacle-z-max", type=float, default=1.5,
                        help="obstacle band upper bound above ground [m]")
    parser.add_argument("--ground-percentile", type=float, default=10.0)
    parser.add_argument("--ground-fill-radius", type=int, default=2)
    parser.add_argument("--min-obstacle-cells", type=int, default=3)
    parser.add_argument("--fill-unknown-islands", type=int, default=0,
                        help="fill interior unknown pockets smaller than N cells")
    parser.add_argument("--terrain-classify", action="store_true",
                        help="also mark untraversable terrain (step/slope/roughness)")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    output_dir = Path(args.output)

    points = pcd_io.load_xyz(args.input)
    raw_count = len(points)
    print(f"loaded {raw_count} points from {args.input}")

    if args.z_min is not None or args.z_max is not None:
        big = 1.0e9
        points = crop_box(
            points,
            min_xyz=(-big, -big, args.z_min if args.z_min is not None else -big),
            max_xyz=(big, big, args.z_max if args.z_max is not None else big),
        )
        print(f"z-crop kept {len(points)} points")

    if args.outlier_voxel > 0.0:
        points = remove_sparse_voxels(
            points, args.outlier_voxel, args.outlier_min_points
        )
        print(f"outlier removal kept {len(points)} points")

    grid_config = GridConfig(
        resolution=args.resolution,
        ground_percentile=args.ground_percentile,
        ground_fill_radius=args.ground_fill_radius,
        obstacle_z_min=args.obstacle_z_min,
        obstacle_z_max=args.obstacle_z_max,
        min_obstacle_cells=args.min_obstacle_cells,
        fill_unknown_islands_smaller_than=args.fill_unknown_islands,
        terrain_classify=args.terrain_classify,
    )
    grid = build_grid(points, grid_config)
    write_map_files(output_dir, grid)
    print(f"grid: {grid.width}x{grid.height} cells at {grid.resolution} m")

    loc_points = voxel_downsample(points, args.loc_voxel)
    pcd_io.save_xyz(output_dir / "map.pcd", loc_points)
    viz_points = voxel_downsample(points, args.viz_voxel)
    pcd_io.save_xyz(output_dir / "viz.pcd", viz_points)
    print(f"map.pcd: {len(loc_points)} points, viz.pcd: {len(viz_points)} points")

    metadata = {
        "name": output_dir.name,
        "source": str(Path(args.input).resolve()),
        "created": datetime.datetime.now().astimezone().isoformat(),
        "raw_points": int(raw_count),
        "localization_points": int(len(loc_points)),
        "viz_points": int(len(viz_points)),
        "grid": {
            "width": int(grid.width),
            "height": int(grid.height),
            "resolution": float(grid.resolution),
            "origin": [float(grid.origin_x), float(grid.origin_y)],
        },
        "parameters": {
            key: (value if not isinstance(value, Path) else str(value))
            for key, value in vars(args).items()
        },
    }
    with (output_dir / "metadata.yaml").open("w", encoding="utf-8") as handle:
        yaml.safe_dump(metadata, handle, sort_keys=False)
    print(f"map written to {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
