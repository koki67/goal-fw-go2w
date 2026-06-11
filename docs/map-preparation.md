# Map preparation

The stack consumes a **map directory** with five required artifacts. The
robot-driven workflow also retains its source cloud:

```
maps/<name>/
├── map.pcd        localization map (voxel-downsampled point cloud)
├── viz.pcd        lightweight cloud for RViz (coarser voxels)
├── grid.pgm       2D occupancy grid image (nav2 map_server)
├── grid.yaml      map_server metadata (resolution, origin, thresholds)
├── metadata.yaml  provenance + all parameters used
└── raw/
    └── dlio_map.pcd  retained source cloud from robot-driven collection
```

## Robot-driven collection: Go2W + Hesai LiDAR

This repository's integrated collection workflow is specifically for a
Unitree Go2W equipped with the configured Hesai PandarXT-16 3D LiDAR. Robot
motion is intentionally outside this framework: install and start
[`go2w_teleop_gamepad`](https://github.com/koki67/go2w_teleop_gamepad) first.
Its `/go2w_teleop_gamepad_node` must be visible on the same ROS domain.

Inside the goal-fw-go2w container:

```bash
bash /external/scripts/prepare_map_tmux.sh output:=/external/maps/office
```

For a tablet workflow, add `web_ui:=true`, open `http://<jetson-ip>:8080`,
and use **Finish & Save** after coverage is complete. The Enter flow remains
available.

The helper fails closed when the teleop node is absent, when goal navigation
is already active, or when Hesai/IMU/D-LIO collection nodes are already
running. It also refuses to overwrite an existing output directory.

The tmux session has two windows:

- `collect`: Hesai driver + Go2W IMU publisher + D-LIO; no Nav2, localizer,
  velocity bridge, or teleop node is launched by this repository
- `finish`: waits for `/save_pcd`; drive the robot, then press
  Enter here to save and convert the map, then stop collection

Useful collection options:

| Argument | Default | Meaning |
|---|---|---|
| `use_rviz:=true` | `false` | start D-LIO's live map/path RViz view |
| `dlio_output:=log` | `screen` | send D-LIO output to ROS logs |
| `save_leaf_size:=0.05` | `0.05` | D-LIO save-time voxel leaf size [m] |
| `web_ui:=true` | `false` | start the browser preview and finish panel |

The finished directory is immediately usable with standard goal navigation:

```bash
# Stop the external teleop system before using bridge_dry_run:=false.
bash /external/scripts/bringup_tmux.sh map:=/external/maps/office
```

Do not leave the external teleop publisher and the live goal-navigation
velocity bridge active together; both publish `/api/sport/request`.

To retune map conversion later without driving again, use the retained raw
cloud and a new output directory:

```bash
ros2 run nav_goal_go2w_map prepare_map \
    --input /external/maps/office/raw/dlio_map.pcd \
    --output /external/maps/office-retuned \
    --obstacle-z-min 0.20
```

## Alternative source A: frontier-fw-go2w exploration run

While (or after) the frontier stack runs, save D-LIO's aggregated map:

```bash
ros2 service call /save_pcd direct_lidar_inertial_odometry/srv/SavePCD \
    "{leaf_size: 0.05, save_path: '/external/maps'}"
```

(The service writes `dlio_map.pcd` inside `save_path`.)

## Alternative source B: handheld LiDAR walk

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
