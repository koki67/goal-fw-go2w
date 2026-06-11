# Troubleshooting

## Localization

**`/initialpose` click does nothing / "no odom->base_link TF yet"**
D-LIO is not running or has no data. Check the `dlio` tmux window and
`ros2 topic hz /points_raw /go2w/imu`.

**Never leaves CONVERGING**
The click was too far off (especially yaw), or the scan barely overlaps the
map. Re-click carefully; verify in RViz that `/map_cloud` resembles what the
robot should currently see. If the environment changed a lot since mapping,
rebuild the map.

**TRACKING but pose visibly wrong (locked into the wrong corridor)**
GICP found a plausible local minimum — symmetric environments do this.
Re-click 2D Pose Estimate somewhere geometrically distinctive. Consider
mapping with more distinguishing structure in view.

**Frequent DEGRADED in open areas**
Few/far returns → low inlier fractions. Lower `min_inlier_fraction` or
raise `max_correspondence_distance` slightly; watch `/localization/fitness`
to pick thresholds with margin.

**TF errors: "extrapolation into the future/past" on costmaps**
The future-dated map->odom should prevent this. If it appears, check that
`transform_tolerance` (localizer) ≥ the registration period jitter and that
`use_sim_time` is consistent across all nodes (bag replay!).

## Maps

**`prepare_map_tmux.sh` says `/go2w_teleop_gamepad_node` is absent**
Start the separately deployed `go2w_teleop_gamepad` system and confirm both
containers use the same `ROS_DOMAIN_ID` and compatible CycloneDDS interface.
The collection helper intentionally will not start without this node.

**`prepare_map_tmux.sh` reports a conflicting node**
Stop goal navigation or the previous mapping session before collecting. The
helper rejects `/velocity_bridge` and duplicate Hesai/IMU/D-LIO nodes so two
systems cannot command or process the robot concurrently.

**Finish window waits forever for `/dlio_map_node/save_pcd`**
Inspect the `collect` window and check `ros2 topic hz /points_raw /go2w/imu`.
D-LIO must start successfully before its map-save service appears.

**Save or conversion failed after pressing Enter**
The requested output directory is not published on failure. The finish window
prints the hidden staging directory containing any successfully saved raw PCD;
use it to diagnose or rerun `prepare_map` manually.

**Floor shows as obstacles in grid.pgm**
Ground bleed: raise `--obstacle-z-min` (e.g. 0.20), check the source cloud
for double floors (z-crop), or increase `--outlier-min-points`.

**Walls have holes / map too sparse**
Mapping pass was too fast or too far from surfaces. Lower `--loc-voxel`
only helps registration, not the grid — re-capture the area instead.

**Goal rejected: "goal_not_traversable" in clearly free space**
The grid disagrees with reality at that cell — inspect `grid.pgm`. Unknown
cells are accepted by default (`treat_unknown_as_reachable`), so this means
*occupied*.

## Navigation

**Goal accepted but robot does not move**
`bridge_dry_run` is still true (intended default!), or Nav2 lifecycle nodes
are not active — check the bringup log for the lifecycle manager, and
`ros2 topic echo /cmd_vel`.

**Robot drives into something the map does not show**
Expected behavior path: the live `/scan` local costmap must catch it. If it
did not, check `pointcloud_to_laserscan` min/max_height — obstacles below
~-0.10 m or above ~0.40 m in base_link are invisible to the local costmap.

**Plan goes through unknown space toward a far goal and oscillates at the
boundary** — that is `allow_unknown` doing its job against an incomplete
map; if undesirable for a deployment, set `treat_unknown_as_reachable:=false`
on the executor and re-prepare the map with `--fill-unknown-islands`.

## Infrastructure

**No topics on the desktop over Wi-Fi**
`docker/run.sh --remote-viz` on the robot side; on the desktop
`source .devcontainer/setup_remote_viz.bash <iface>`. Same ROS_DOMAIN_ID,
CycloneDDS on both ends. Robot Jetson uses `wlan0`, desktop `enp97s0`.

**small_gicp import error in the container**
The pip install built from source on aarch64; check the image build log. If
the build is flaky, build the wheel once and vendor it under
`docker/wheels/` (see docker/Dockerfile comment).
