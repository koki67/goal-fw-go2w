# Remote Visualization

The validated default keeps CycloneDDS bound to the robot/internal interface
(`eth0`). Use the optional remote-visualization mode when you want a laptop on
the robot Wi-Fi network to inspect the same ROS 2 graph in RViz.

Start the container on the robot with Wi-Fi DDS enabled:

```bash
bash docker/run.sh --remote-viz
```

By default this keeps `eth0` for the robot DDS graph and adds `wlan0` for the
remote laptop. If the robot uses another Wi-Fi interface name, pass it
explicitly:

```bash
bash docker/run.sh --remote-viz --remote-viz-iface wlan1
```

The option generates a runtime CycloneDDS profile and does not edit
`config/cyclonedds.xml`. If `wlan0` is missing, the script falls back to `eth0`
only and prints a warning.

## Laptop Setup

The laptop must use the same ROS domain and a compatible DDS implementation:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
rviz2
```

If the laptop has multiple network interfaces and discovery is unreliable, bind
CycloneDDS to the laptop Wi-Fi interface with a local profile, for example:

```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="wlan0" priority="1" multicast="true" /></Interfaces></General></Domain></CycloneDDS>'
rviz2
```

Confirm discovery before opening a full RViz layout:

```bash
ros2 topic list | grep -E '^/map$|^/goal_pose$|^/cmd_vel$'
```

### Humble Devcontainer On Ubuntu 24.04

For an Ubuntu 24.04 desktop, use the repo's `.devcontainer/` setup to run RViz
from a ROS 2 Humble container instead of mixing Jazzy on the laptop with Humble
on the robot.

Before opening the devcontainer, allow local Docker GUI access on the desktop:

```bash
xhost +local:docker
```

Then open the repository in VS Code and choose **Dev Containers: Reopen in
Container**. Inside the container, start with automatic CycloneDDS interface
selection:

```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep -E '^/map$|^/goal_pose$|^/cmd_vel$'
rviz2 -d .devcontainer/goal_remote.rviz (humble_ws/src/nav_goal_go2w_bringup/config/)
```

If the desktop has multiple interfaces and discovery is unreliable, bind
CycloneDDS to the Wi-Fi interface connected to the robot network:

```bash
source .devcontainer/setup_remote_viz.bash enp97s0
ros2 topic list | grep -E '^/map$|^/goal_pose$|^/cmd_vel$'
rviz2 -d .devcontainer/goal_remote.rviz (humble_ws/src/nav_goal_go2w_bringup/config/)
```

For your current desktop, the robot-network interface is `enp97s0` with IP
`192.168.111.100`. Replace it only if that host interface changes.

For the usual desktop setup, this helper runs the same DDS setup and opens the
saved RViz layout:

```bash
.devcontainer/start_remote_rviz.bash
```

For a 3D camera view of the D-LIO map cloud with the 2D map, costmap, frontier,
and path overlays still projected on the ground plane, use:

```bash
.devcontainer/start_remote_rviz_3d.bash
```

## What To Inspect

DDS exposes the ROS 2 graph on the selected interfaces; this option does not
filter by topic. The devcontainer RViz profile sets the fixed frame to `map`
and preloads the displays below. If launching RViz manually, add the same
displays for the current debugging session.

Core goal-navigation workflow:

| RViz display | Topic |
|--------------|-------|
| TF | `/tf`, `/tf_static` |
| Map | `/map` |
| Map | `/map_planning` |
| Map | `/global_costmap/costmap_known_only` |
| Map | `/local_costmap/costmap` |
| Pose | `/goal_pose` |
| MarkerArray | `/frontier_markers` |
| Odometry | `/dlio/odom_node/odom` |
| Path | `/dlio/odom_node/path` |

Sensor and controller diagnostics:

| RViz display | Topic | Notes |
|--------------|-------|-------|
| LaserScan | `/scan` | Lighter than the full point cloud. |
| PointCloud2 | `/points_raw` | Useful but bandwidth-heavy over Wi-Fi. |
| PointCloud2 | `/dlio/map_node/map` | D-LIO map cloud; bandwidth-heavy. |
| MarkerArray | `/mppi_trajectory_lines` | Thin-line visualization of MPPI sampled trajectories and Nav2's optimal MPPI trajectory namespace. |
| MarkerArray | `/trajectories` | Raw MPPI sampled trajectory markers, disabled by default in RViz. |
| Path | `/transformed_global_plan` | MPPI/controller transformed global plan, disabled by default. |

The saved RViz layout shows the Slam Toolbox `/map` normally, including unknown
space. It does not show the raw `/global_costmap/costmap` by default; instead,
`map_viz_layers` republishes `/global_costmap/costmap_known_only` with unknown
cells replaced by free cells for visualization only. Nav2 still plans on the
real global costmap. Inspect `/map`, `/frontier_markers`, and `/goal_pose`
when debugging frontier selection, and inspect the filtered global costmap when
debugging obstacle/inflation effects.

Use the top-down layout for precise costmap/path inspection. Use the 3D layout
when judging the D-LIO point-cloud map geometry against the projected 2D
navigation overlays.

## Bandwidth Notes

Start with `/map`, costmaps, TF, `/goal_pose`, `/frontier_markers`, and the
D-LIO map cloud. Keep `/points_raw` disabled unless you need raw LiDAR detail;
it is the topic most likely to saturate Wi-Fi or make RViz lag.
