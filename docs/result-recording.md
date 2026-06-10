# Result recording

The top-level bringup can record a timestamped rosbag of a goal-navigation
run for offline RViz inspection. Recording is disabled by default.

```bash
ros2 launch nav_goal_go2w_bringup bringup.launch.py \
    map:=/external/maps/office \
    bridge_dry_run:=false \
    record_results:=true
```

Bags are written inside the container to `/external/bags` (the host repo
mounted by `docker/run.sh`), i.e. `./bags/goal_nav_results_YYYYMMDD_HHMMSS`
on the host. Overrides: `record_bag_dir`, `record_bag_prefix`,
`record_storage`.

## What gets recorded

The regex in `bringup.launch.py` (`RESULT_RECORDING_REGEX`) captures:

| Group | Topics |
|---|---|
| Map serving | `/map`, `/map_cloud` |
| Localization | `/initialpose`, `/localization/{pose,state,fitness}`, `/diagnostics` |
| Goals | `/goal_pose`, `/goal_markers`, `/goal_executor/status` |
| Odometry | `/dlio/odom_node/*`, deskewed/keyframe clouds, `/dlio/map_node/map` |
| Navigation | costmaps, `/plan`, MPPI trajectories, `/cmd_vel_nav`, `/cmd_vel` |
| Robot I/O | `/scan`, `/api/sport/{request,response}` |
| Actions | NavigateToPose / ComputePathToPose / FollowPath / SmoothPath (hidden topics) |
| Infra | `/tf`, `/tf_static`, `/clock`, `/robot_description` |

Node logs for the run land next to the bag in `<bag>/logs/`.

## Replaying

```bash
source /opt/ros/humble/setup.bash && source humble_ws/install/setup.bash
ros2 bag play bags/goal_nav_results_YYYYMMDD_HHMMSS --clock
rviz2 -d humble_ws/src/nav_goal_go2w_bringup/config/goal_nav.rviz   # set use_sim_time
```

Because the bag contains `/tf`, `/map`, and `/map_cloud`, replay needs no
live nodes — RViz alone reconstructs the run. For localization debugging,
`/localization/state` + `/localization/fitness` plotted against `/cmd_vel`
shows exactly when gating rejected registrations during motion.
