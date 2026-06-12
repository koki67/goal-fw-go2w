# Vendored upstreams

The packages under `humble_ws/src/` that are not authored in this repo were copied from upstream sources at the commits below. To re-sync any package, fetch the upstream at a newer SHA and overwrite the package directory in this tree, preserving any local fixes documented here.

| Package                            | Upstream repo                                                | Branch       | Commit SHA                                 | Notes |
|------------------------------------|--------------------------------------------------------------|--------------|--------------------------------------------|-------|
| `direct_lidar_inertial_odometry`   | `https://github.com/koki67/direct_lidar_inertial_odometry`   | `feature/ros2` | `6b118082211783da51f0196722f0759abb6b3765` | D-LIO with continuous-time motion correction. **Local fix:** `launch/dlio.launch.py` — `map` topic remapped to `dlio/map_node/map` to avoid clash with the nav2 map_server `/map` (OccupancyGrid). |
| `hesai_lidar`                      | `https://github.com/koki67/go2w-hesai-lidar-driver`          | `main`       | `6c6a93aa3c844a49d2cb26360e5bdbc4687b427d` | Hesai PandarXT-16 driver, ROS 2 port. Vendored SDK warnings suppressed upstream. |
| `go2w_imu_publisher`               | `https://github.com/koki67/go2w-imu-publisher`               | `main`       | `ace34a8602013026a9a9a682cb5776d8c4f451f0` | LowState (`unitree_go/LowState`) → `sensor_msgs/Imu` on `/go2w/imu`. |
| `unitree_api`                      | `https://github.com/koki67/unitree_ros2`                     | `master`     | `7db14822c51bcbba94cdd4a59ef1a6d6e56e7432` | **msg-only subset** (curated downstream). Used for `Request`/`Response` types of the Sport API. |
| `unitree_go`                       | `https://github.com/koki67/unitree_ros2`                     | `master`     | `7db14822c51bcbba94cdd4a59ef1a6d6e56e7432` | **msg-only subset** (curated downstream). Used for `LowState` consumed by the IMU publisher. |

## Local modifications

- **D-LIO**: `launch/dlio.launch.py:80` — added `('map', 'dlio/map_node/map')` remap on `dlio_map_node` so D-LIO's PointCloud2 map publication does not collide with the map_server's `OccupancyGrid` on `/map`.

When updating any vendored package, re-apply the local modifications above and update the SHA column.

## Browser asset

| Asset | Version | License | Source |
|---|---|---|---|
| `nav_goal_go2w_web/www/vendor/roslib.min.js` | 1.4.1 | BSD-3-Clause | `https://cdn.jsdelivr.net/npm/roslib@1.4.1/build/roslib.min.js` |
