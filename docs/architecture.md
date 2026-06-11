# Architecture

goal-fw-go2w navigates the Unitree Go2-W to operator-selected goals inside a
**pre-built point cloud map**. The map can be collected by remotely driving
the Hesai-equipped Go2W with a separately deployed gamepad system, or supplied
by another mapping pipeline. It may be incomplete — the stack treats unknown
space as plannable and relies on the live local costmap for reality.

## Dataflow

```
prepare-map phase:
external gamepad ─► /api/sport/request ─► Go2W motion
Hesai + Go2W IMU ─► D-LIO ─► /dlio_map_node/save_pcd ─► raw/dlio_map.pcd
raw/dlio_map.pcd ─► prepare_map ─► maps/<name>/{map.pcd, viz.pcd, grid.pgm, grid.yaml, metadata.yaml}

offline:  raw.pcd ──prepare_map──► maps/<name>/{map.pcd, viz.pcd, grid.pgm, grid.yaml, metadata.yaml}

runtime:
Hesai /points_raw ─┬─► D-LIO ─► TF odom->base_link + /dlio/odom_node/odom
/go2w/imu ─────────┘        └─► /dlio/odom_node/pointcloud/deskewed (odom frame)
                                              │
nav2_map_server grid.yaml ─► /map             ▼
map_cloud_publisher viz.pcd ─► /map_cloud   scan_to_map_localizer (small_gicp GICP @2 Hz)
RViz "2D Pose Estimate" ─► /initialpose ──────┤
                                              └─► TF map->odom @50 Hz + /localization/{pose,state,fitness}
RViz "2D Nav Goal" ─► /goal_pose ─► goal_pose_executor ─► NavigateToPose
                                              │ (gated on /localization/state)
/points_raw ─► pointcloud_to_laserscan ─► /scan ─► Nav2 local costmap
                                              ▼
                    Nav2 (NavFn allow_unknown + MPPI Omni + velocity_smoother)
                                              │ /cmd_vel
                    velocity_bridge (50 Hz, watchdog, dry_run)
                                              │ /api/sport/request (Move=1008 / Stop=1003)
                                          Go2W Sport API
```

## TF tree and frame ownership

```
map ──(scan_to_map_localizer, 50 Hz, future-dated)──► odom
odom ──(D-LIO)──► base_link
base_link ──(static, bringup)──► hesai_lidar   t=(0.1634, 0, 0.116)  yaw=+π/2
base_link ──(static, bringup)──► imu_link      t=(0, 0, 0)           yaw=+π/2
```

No `map -> odom` TF exists until the operator sets `/initialpose`. With the
TF missing, Nav2's global costmap cannot resolve the robot pose, so the
navigation stack stays safely inert — this is intentional.

## Packages

| Package | Role |
|---|---|
| `nav_goal_go2w_map` | offline `prepare_map` CLI; runtime map_server wrapper + `/map_cloud` publisher |
| `nav_goal_go2w_localization` | small_gicp scan-to-map registration, map->odom TF, health state machine |
| `nav_goal_go2w_planner` | Nav2 stack (NavFn + MPPI Omni) + `/goal_pose` executor with localization gating |
| `nav_goal_go2w_bridge` | `/cmd_vel` -> Sport API bridge (verbatim from frontier-fw-go2w) |
| `nav_goal_go2w_sim` | closed-loop desktop sim with synthetic odometry drift + `gen_sim_map` |
| `nav_goal_go2w_bringup` | goal-navigation launch plus sensor/D-LIO-only prepare-map collection launch |
| vendored | `direct_lidar_inertial_odometry`, `hesai_lidar`, `go2w_imu_publisher`, `go2w_description`, `unitree_api`, `unitree_go` |

## Operator workflow

Map preparation uses `scripts/prepare_map_tmux.sh`. It requires the external
`/go2w_teleop_gamepad_node`, rejects active navigation and duplicate mapping
nodes, and never launches a velocity bridge or teleop publisher itself. The
finish window saves D-LIO, converts the map, and retains the source PCD.

1. `ros2 launch nav_goal_go2w_bringup bringup.launch.py map:=/external/maps/<name>`
   (or `scripts/bringup_tmux.sh map:=...`)
2. RViz shows `/map_cloud` + `/map`. Click **2D Pose Estimate** at the
   robot's true location and rough heading.
3. Watch `/localization/state`: CONVERGING -> **TRACKING** (re-click if it
   does not converge within ~5 s).
4. Click **2D Nav Goal**. The robot navigates autonomously. A new click
   preempts the current goal. `/goal_executor/status` and `/goal_markers`
   show progress.
5. If localization degrades to LOST, the active goal is canceled and the
   bridge watchdog stops the robot; re-click 2D Pose Estimate to recover.
