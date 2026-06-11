# Topics, TF, and actions

## Sensors and odometry

| Topic | Type | Producer | Consumers | Notes |
|---|---|---|---|---|
| `/points_raw` | sensor_msgs/PointCloud2 | hesai_lidar | D-LIO, pointcloud_to_laserscan | ~10 Hz, frame `hesai_lidar` |
| `/go2w/imu` | sensor_msgs/Imu | go2w_imu_publisher | D-LIO | ~500 Hz, frame `imu_link` |
| `/dlio/odom_node/odom` | nav_msgs/Odometry | D-LIO | Nav2 (bt_navigator, velocity_smoother) | odom -> base_link |
| `/dlio/odom_node/pointcloud/deskewed` | sensor_msgs/PointCloud2 | D-LIO | scan_to_map_localizer | motion-compensated, **odom frame** |
| `/dlio/map_node/map` | sensor_msgs/PointCloud2 | D-LIO map node | prepare-map RViz | accumulated sparse map in `odom` frame |
| `/scan` | sensor_msgs/LaserScan | pointcloud_to_laserscan | Nav2 local costmap | base_link slice [-0.10, 0.40] m |

## Map serving

| Topic | Type | Producer | Notes |
|---|---|---|---|
| `/map` | nav_msgs/OccupancyGrid | nav2_map_server | TRANSIENT_LOCAL; from `grid.yaml`; global costmap static layer + goal validation |
| `/map_cloud` | sensor_msgs/PointCloud2 | map_cloud_publisher | TRANSIENT_LOCAL, frame `map`; viz.pcd for RViz; slow republish for lossy Wi-Fi |

## Localization

| Topic | Type | Producer | Notes |
|---|---|---|---|
| `/initialpose` | geometry_msgs/PoseWithCovarianceStamped | RViz "2D Pose Estimate" | resets the localizer; required once at startup |
| `/localization/pose` | geometry_msgs/PoseWithCovarianceStamped | localizer | map-frame base pose per accepted registration |
| `/localization/state` | std_msgs/String | localizer | TRANSIENT_LOCAL; UNINITIALIZED / CONVERGING / TRACKING / DEGRADED / LOST |
| `/localization/fitness` | std_msgs/Float32 | localizer | mean registration residual per inlier |
| `/diagnostics` | diagnostic_msgs/DiagnosticArray | localizer | entry `goal_nav/localizer`: inlier fraction, latency, rejects |

## Goal execution and navigation

| Topic | Type | Producer | Notes |
|---|---|---|---|
| `/goal_pose` | geometry_msgs/PoseStamped | RViz "2D Nav Goal" | frame `map`; new goal preempts (default) |
| `/goal_executor/status` | std_msgs/String | goal_pose_executor | TRANSIENT_LOCAL; IDLE / ACTIVE / PREEMPTING / REJECTED ... |
| `/goal_markers` | visualization_msgs/MarkerArray | goal_pose_executor | green=active, yellow=pending, red=failed + status text |
| `/global_costmap/costmap` | nav_msgs/OccupancyGrid | Nav2 | static layer from `/map` + inflation |
| `/local_costmap/costmap` | nav_msgs/OccupancyGrid | Nav2 | rolling 6x6 m, odom frame, obstacle layer from `/scan` |
| `/plan` | nav_msgs/Path | NavFn | global plan |
| `/mppi_trajectory_lines` | visualization_msgs/MarkerArray | mppi_trajectory_lines | normalized MPPI candidate/optimal trajectories |
| `/cmd_vel_nav` | geometry_msgs/Twist | MPPI controller | pre-smoothing |
| `/cmd_vel` | geometry_msgs/Twist | velocity_smoother | consumed by the bridge |
| `/api/sport/request` | unitree_api/Request | velocity_bridge or external teleop | goal-navigation commands or prepare-map manual control; never run both together |

## TF

| Transform | Publisher | Rate |
|---|---|---|
| `map -> odom` | scan_to_map_localizer | 50 Hz, future-dated by `transform_tolerance` (0.3 s); absent before `/initialpose` |
| `odom -> base_link` | D-LIO | per scan |
| `base_link -> hesai_lidar`, `base_link -> imu_link` | bringup static TFs | static |

## Actions

| Server | Type | Client |
|---|---|---|
| `/navigate_to_pose` | nav2_msgs/NavigateToPose | goal_pose_executor (BasicNavigator) |

## Services

| Service | Type | Server | Notes |
|---|---|---|---|
| `/dlio_map_node/save_pcd` | direct_lidar_inertial_odometry/srv/SavePCD | D-LIO map node | writes `dlio_map.pcd` during prepare-map finalization |
