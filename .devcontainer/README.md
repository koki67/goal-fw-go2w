# goal-fw-go2w desktop devcontainer

ROS 2 Humble devcontainer for desktop-side development: remote RViz toward
the robot over Wi-Fi DDS, and the closed-loop desktop simulator.

## Remote visualization (robot running elsewhere)

```bash
.devcontainer/start_remote_rviz.bash            # default iface enp97s0
.devcontainer/start_remote_rviz.bash wlp2s0     # explicit Wi-Fi iface
```

Opens RViz with the goal-navigation layout (pre-built map cloud, static map,
costmaps, localization pose, goal markers). Use the **2D Pose Estimate** and
**2D Nav Goal** tools to drive the robot.

Sanity check the DDS link first:

```bash
source .devcontainer/setup_remote_viz.bash [iface]
ros2 topic list | grep -E '^/map$|^/map_cloud$|^/localization/state$'
```

## Desktop simulator (no robot)

```bash
.devcontainer/build_desktop_sim_workspace.bash
ros2 run nav_goal_go2w_sim gen_sim_map --world open_room --output maps/sim_open_room
.devcontainer/run_desktop_sim.bash --world open_room --rviz map:=maps/sim_open_room
```

Add odometry drift to exercise the localizer:

```bash
.devcontainer/run_desktop_sim.bash --world open_room --rviz \
    map:=maps/sim_open_room odom_drift_yaw_per_m:=0.02
```

Then click 2D Pose Estimate at the robot spawn, wait for TRACKING on
`/localization/state`, and click 2D Nav Goal.

`macos/` contains the macOS (VNC-based) variant; see docs/macos-simulation.md
in frontier-fw-go2w for the workflow it mirrors.
