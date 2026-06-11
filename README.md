# goal-fw-go2w

Goal-directed autonomous navigation framework for the Unitree Go2-W.

Given a pre-built 3D point cloud map, this stack localizes the robot against
that map and navigates to goals picked by an operator in RViz. This repository
can collect that map with a remotely controlled Go2W equipped with its Hesai
3D LiDAR, or consume a PCD produced by another mapping workflow.

```
prepare_map ─► map dir ─► map_server + map_cloud ─► RViz operator
Hesai + IMU ─► D-LIO odometry ─► small_gicp scan-to-map localizer (map->odom)
RViz 2D Nav Goal ─► goal executor ─► Nav2 (NavFn + MPPI) ─► velocity bridge ─► Sport API
```

## Quick start (robot)

```bash
bash scripts/build_image.sh                  # once, on the Jetson
bash docker/run.sh
# inside the container:
bash /external/scripts/prepare_map_tmux.sh output:=/external/maps/office
# drive with the separately deployed go2w_teleop_gamepad, then press Enter
# in the tmux finish window
# stop the external teleop before enabling the live navigation velocity bridge
bash /external/scripts/bringup_tmux.sh map:=/external/maps/office
```

The robot-driven map preparation workflow is specifically for the Go2W with
the Hesai LiDAR. It requires the separately deployed
[`go2w_teleop_gamepad`](https://github.com/koki67/go2w_teleop_gamepad)
node to already be running and visible as `/go2w_teleop_gamepad_node`.
See [docs/map-preparation.md](docs/map-preparation.md) before the first run.

In RViz (desktop: `.devcontainer/start_remote_rviz.bash <wifi-iface>`):

1. **2D Pose Estimate** at the robot's actual location → wait for
   `/localization/state: TRACKING`
2. **2D Nav Goal** anywhere on the map → the robot navigates there
   autonomously; a new click preempts the current goal

The velocity bridge starts in **dry-run** (logs Move/Stop, robot stays
still); add `bridge_dry_run:=false` when ready.

## Quick start (no robot — desktop sim)

```bash
.devcontainer/build_desktop_sim_workspace.bash
.devcontainer/run_desktop_sim.bash --world open_room --rviz odom_drift_yaw_per_m:=0.02
```

Same operator workflow, with synthetic odometry drift for the localizer to
correct. `bash scripts/smoke_test.sh` runs the loop headless.

## Documentation

| Doc | Content |
|---|---|
| [docs/architecture.md](docs/architecture.md) | dataflow, TF tree, package roles, operator workflow |
| [docs/map-preparation.md](docs/map-preparation.md) | getting PCDs, `prepare_map`, grid classification, quality checklist |
| [docs/localization.md](docs/localization.md) | registration design, state machine, gating, tuning |
| [docs/goal-navigation.md](docs/goal-navigation.md) | operator runbook, goal acceptance rules, first live run |
| [docs/topics.md](docs/topics.md) | full topic/TF/action catalog |
| [docs/desktop-simulation.md](docs/desktop-simulation.md) | closed-loop sim, odometry drift, bypass modes |
| [docs/tuning-parameters.md](docs/tuning-parameters.md) | the knobs that matter, per subsystem |
| [docs/troubleshooting.md](docs/troubleshooting.md) | localization, map, navigation, DDS issues |
| [docs/remote-visualization.md](docs/remote-visualization.md) | desktop RViz over Wi-Fi DDS |
| [docs/result-recording.md](docs/result-recording.md) | rosbag capture + replay |
| [docs/vendored-upstreams.md](docs/vendored-upstreams.md) | upstream SHAs + local fixes |

## License

MIT (vendored packages retain their original licenses — see LICENSE).
