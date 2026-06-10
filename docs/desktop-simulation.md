# Desktop simulation

Closed-loop simulation of the complete stack — map serving, scan-to-map
localization, goal execution, Nav2, with the velocity loop closed through a
2D kinematic robot — no hardware required.

## Quick start

```bash
.devcontainer/build_desktop_sim_workspace.bash
# committed sample map; regenerate for other worlds:
ros2 run nav_goal_go2w_sim gen_sim_map --world open_room --output maps/sim_open_room
.devcontainer/run_desktop_sim.bash --world open_room --rviz
```

Then exactly the operator workflow: *2D Pose Estimate* at the spawn point
(world origin unless the world YAML moves it), wait for `TRACKING`, then
*2D Nav Goal*.

## Exercising the localizer with odometry drift

By default sim odometry is perfect, so map->odom stays put. Add drift to
make the localizer actually work:

```bash
.devcontainer/run_desktop_sim.bash --world narrow_corridor --rviz \
    odom_drift_yaw_per_m:=0.02 odom_drift_x_per_m:=0.02
```

The sim integrates ground truth (drives raycasting and collisions) and a
*separate* drifted odometry (drives TF, `/dlio/odom_node/odom`, and the
deskewed cloud frame). Watch the red odometry arrows diverge from the green
localization pose while the robot still tracks its goals — that divergence
is the correction the localizer provides.

## Bypassing localization

```bash
.devcontainer/run_desktop_sim.bash --world doorway --rviz sim_localization:=false
```

publishes a static identity map->odom instead of running the localizer —
useful to debug Nav2 / executor behavior in isolation (only meaningful with
zero drift).

## Other knobs

- `--world {open_room, doorway, narrow_corridor, dead_end, t_junction, stairs, ...}`
  (worlds under `nav_goal_go2w_sim/worlds/`; regenerate the matching map!)
- `num_dynamic_obstacles:=3` — moving obstacles to exercise the local
  costmap against the static map
- pause/resume: `ros2 topic pub --once /sim/pause std_msgs/msg/Bool '{data: true}'`
- metrics: `ros2 topic echo /sim/metrics` (pose, distance, collisions)

## Smoke test

`scripts/smoke_test.sh` runs this whole loop headless (initialpose →
TRACKING → goal → `/cmd_vel`) and is the fastest regression check inside
the container.
