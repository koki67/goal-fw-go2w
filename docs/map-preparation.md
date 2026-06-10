# Map preparation

The stack consumes a **map directory** with five artifacts:

```
maps/<name>/
├── map.pcd        localization map (voxel-downsampled point cloud)
├── viz.pcd        lightweight cloud for RViz (coarser voxels)
├── grid.pgm       2D occupancy grid image (nav2 map_server)
├── grid.yaml      map_server metadata (resolution, origin, thresholds)
└── metadata.yaml  provenance + all parameters used
```

## Source A: frontier-fw-go2w exploration run

While (or after) the frontier stack runs, save D-LIO's aggregated map:

```bash
ros2 service call /dlio_map/save_pcd direct_lidar_inertial_odometry/srv/SavePCD \
    "{leaf_size: 0.05, save_path: '/external/maps'}"
```

(check the exact service name with `ros2 service list | grep save_pcd`).

## Source B: handheld LiDAR walk

Any odometry/mapping pipeline that outputs a registered point cloud works —
carry the LiDAR through the environment, export a `.pcd` (pypcd4 reads
ascii, binary, and binary_compressed encodings).

## Convert with prepare_map

```bash
ros2 run nav_goal_go2w_map prepare_map \
    --input /external/maps/raw_office.pcd \
    --output /external/maps/office \
    --resolution 0.05
```

Useful options:

| Option | Default | Meaning |
|---|---|---|
| `--loc-voxel` | 0.20 | map.pcd voxel size (smaller = sharper registration, more CPU) |
| `--viz-voxel` | 0.40 | viz.pcd voxel size (keep coarse for Wi-Fi RViz) |
| `--z-min/--z-max` | off | hard z-crop before everything (cut basement/ceiling returns) |
| `--outlier-voxel/--outlier-min-points` | 0.30 / 4 | sparse-voxel outlier removal; `--outlier-voxel 0` disables |
| `--obstacle-z-min/--obstacle-z-max` | 0.15 / 1.5 | obstacle band **above local ground**; tune to robot clearance/height |
| `--ground-percentile` | 10 | robust per-cell ground estimate |
| `--min-obstacle-cells` | 3 | remove occupied speckles smaller than this |
| `--fill-unknown-islands` | 0 (off) | fill interior unknown pockets smaller than N cells |
| `--terrain-classify` | off | additionally mark step/slope/roughness-untraversable terrain occupied |

### How the grid is classified (ground-relative)

Per 2D cell, ground height is the low percentile of that cell's z values,
smoothed by a neighborhood median (and the minimum of the two — so a raised
ledge stays an obstacle, and a table top above a point-less floor patch is
classified against its neighbors' ground). A point is an **obstacle** only
if it lies `obstacle_z_min..obstacle_z_max` above that local ground. Sloped
floors therefore stay free no matter their absolute z, and ceilings or
overhangs above the robot are ignored.

- **free**: cell has near-ground returns and no obstacle returns
- **occupied**: any obstacle-band return (after speckle cleanup)
- **unknown (-1)**: no returns — *plannable*: NavFn runs with
  `allow_unknown` and the executor treats unknown as reachable, since the
  pre-built map is allowed to be incomplete

## Quality checklist before first use

1. Open `grid.pgm` — walls closed? floor free (not speckled)? stairs/ledges
   occupied? If ground bleed-through appears, raise `--obstacle-z-min`
   slightly or check `--z-min` cropping.
2. `metadata.yaml` — `localization_points` should land in the 50k–1M range;
   bigger maps still work but cost registration time.
3. Load it in RViz (`ros2 launch nav_goal_go2w_map map_servers.launch.py
   map:=<dir>`) and confirm `/map` and `/map_cloud` overlap.

## Simulator maps

`gen_sim_map` produces the same directory from a simulator world (the world
is ground truth, so the grid is exact and the PCD is synthesized to match
the simulated LiDAR's geometry):

```bash
ros2 run nav_goal_go2w_sim gen_sim_map --world open_room --output maps/sim_open_room
```
