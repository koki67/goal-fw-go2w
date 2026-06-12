# goal-fw-go2w

Goal-directed autonomous navigation framework for the Unitree Go2-W.

Given a pre-built 3D point cloud map, this stack localizes the robot against
that map and navigates to goals picked by an operator in RViz or the optional
browser UI. This repository
can collect that map with a remotely controlled Go2W equipped with its Hesai
3D LiDAR, or consume a PCD produced by another mapping workflow.

```
prepare_map ─► map dir ─► map_server + map_cloud ─► RViz operator
Hesai + IMU ─► D-LIO odometry ─► small_gicp scan-to-map localizer (map->odom)
RViz 2D Nav Goal ─► goal executor ─► Nav2 (NavFn + MPPI) ─► velocity bridge ─► Sport API
```

## Workflow

### Browser UI deployment model

Run the framework and browser UI on the robot's onboard Jetson, then open the page
from a separate laptop, tablet, or phone:

```
operator device browser -> HTTP :8080 / WebSocket :9090 -> Jetson -> ROS 2 -> Go2W
```

The operator device does not run this repository or need ROS 2. It only needs
to be on a network that can reach the Jetson. A remote-desktop/VNC session is
not required: open a local browser on the operator device and navigate to
`http://<jetson-ip>:8080`. The Jetson serves the page and rosbridge connects
the page to the ROS 2 graph.

To find the address, run `hostname -I` on the Jetson and use the address on
the network shared with the operator device. For example, if it reports
`192.168.1.42`, open `http://192.168.1.42:8080`.

The browser UI is opt-in. Pass `web_ui:=true` to either map preparation or
navigation bringup. Because `docker/run.sh` uses host networking, no Docker
port publishing is needed. Ports 8080 and 9090 must be reachable from the
operator device. They have no authentication or TLS, so expose them only on
the trusted robot LAN.

### Step 1 — Build the map (once per environment)

Map preparation requires the separately deployed
[`go2w_teleop_gamepad`](https://github.com/koki67/go2w_teleop_gamepad)
to be running and visible as `/go2w_teleop_gamepad_node` before you start.
This framework never publishes motion commands during mapping.

```bash
bash scripts/build_image.sh          # once, on the Jetson
bash docker/run.sh
# inside the container:
bash /external/scripts/prepare_map_tmux.sh \
  output:=/external/maps/office \
  web_ui:=true
```

On the operator device, open `http://<jetson-ip>:8080`. The browser shows a
low-rate top-down preview while you drive the robot with the gamepad. When
coverage is complete, press **Finish & Save** and confirm. The page progresses
through `SAVING`, `CONVERTING`, and `DONE`, then disconnects when collection
shuts down. Pressing Enter in the tmux `finish` window remains an alternative.

Two tmux windows open:

| Window | What runs |
|---|---|
| `collect` | Hesai driver + Go2W IMU publisher + D-LIO (no Nav2, no bridge) |
| `finish` | Waits for your Enter key, then saves and converts the map |

Drive the robot around the space with the gamepad. When coverage is
complete, press Enter in the `finish` window. The script saves D-LIO's
aggregated cloud and runs `prepare_map` to produce the map directory:

```
maps/office/
├── map.pcd        # localization target (voxel-downsampled)
├── viz.pcd        # lightweight RViz cloud
├── grid.pgm       # Nav2 occupancy grid image
├── grid.yaml      # map_server metadata
├── metadata.yaml  # provenance + parameters
└── raw/dlio_map.pcd
```

Unknown space in the grid is plannable — the map is allowed to be incomplete.
See [docs/map-preparation.md](docs/map-preparation.md) for quality checks,
`prepare_map` options, and alternative map sources (frontier-fw exploration,
handheld LiDAR).

---

### Step 2 — Bringup

Stop the external teleop before this step — both it and the navigation
velocity bridge publish `/api/sport/request`.

```bash
bash /external/scripts/bringup_tmux.sh \
  map:=/external/maps/office \
  web_ui:=true
```

On the operator device, open or reload `http://<jetson-ip>:8080`. Use this
same page to set the initial pose and send navigation goals. RViz remains
available independently, but is not required for the browser workflow.

Three tmux windows start simultaneously:

| Window | What runs |
|---|---|
| `dlio` | D-LIO LiDAR-inertial odometry → `odom→base_link` TF |
| `bringup` | Nav2 + localizer + map server + velocity bridge |
| `health` | Streams `/localization/state` |

**The bridge starts in dry-run by default** — Move/Stop commands are logged
but the robot does not move. Add `bridge_dry_run:=false` when ready for live
motion.

On the desktop, start RViz over Wi-Fi:

```bash
.devcontainer/start_remote_rviz.bash <wifi-iface>
```

---

### Step 3 — Set the initial pose (required each session)

Until you do this, the `map` TF frame does not exist and Nav2 is completely
inert. This is intentional — the safety property is that the robot cannot
receive navigation commands before localization is established.

In the browser UI:

1. Choose **Set initial pose**
2. Click the robot's actual position on the map and drag toward its heading
   (yaw accuracy matters more than position accuracy)
3. Watch the `health` window: `UNINITIALIZED → CONVERGING → TRACKING`

If it stays in CONVERGING after ~5 s, click again with a better yaw estimate.
The equivalent RViz tool is **2D Pose Estimate**.

---

### Step 4 — Navigate

1. In the browser UI, choose **Send goal**, then click a destination and drag
   toward the desired final heading
2. The robot navigates autonomously — NavFn plans the global path, MPPI Omni
   executes it
3. Click a new goal at any time to preempt the current one
4. To stop: `Ctrl-C` in the `bringup` window — the bridge watchdog halts the
   robot within 0.5 s

Goals are **rejected** when localization is not TRACKING/DEGRADED, or the
goal cell is marked occupied on `/map`.

The equivalent RViz tool is **2D Nav Goal**. The browser page publishes only
`/initialpose` and `/goal_pose`; it never publishes velocity commands.

---

### Localization states

```
UNINITIALIZED ──/initialpose──► CONVERGING ──3 good regs──► TRACKING
                                                                │
                                               4 bad regs ◄────┘
                                                   │
                                               DEGRADED ──12 total──► LOST
                                                   │                    │
                                               good reg              /initialpose
                                                   └──────► TRACKING    ▼
                                                                     CONVERGING
```

| State | Meaning |
|---|---|
| **UNINITIALIZED** | No TF, Nav2 inert. Waiting for 2D Pose Estimate. |
| **CONVERGING** | Relaxed registration after initial click; building confidence. |
| **TRACKING** | Full navigation allowed. |
| **DEGRADED** | Several consecutive registration failures; pose held from odometry. Goals still run; slow down. |
| **LOST** | Active goal canceled, robot stopped. Re-click 2D Pose Estimate to recover. |

---

### Recommended first-run sequence

1. Run bringup with `bridge_dry_run:=true` (the default). Set the initial
   pose, send a goal, and verify the planned path looks sane in RViz. Check
   that the `bringup` window logs `[DRY_RUN] Move ...` lines.
2. Re-launch with `bridge_dry_run:=false vx_max:=0.2` and a short goal (2–3 m)
   in open space. Keep a hand on the e-stop.
3. Test preemption (second click mid-run) and the kill path (`Ctrl-C` →
   robot stops within 0.5 s).
4. Verify LOST recovery: briefly cover the LiDAR — state should go
   DEGRADED → LOST and the active goal must cancel.

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
| [docs/web-ui.md](docs/web-ui.md) | browser navigation and map-preparation operator UI |
| [docs/result-recording.md](docs/result-recording.md) | rosbag capture + replay |
| [docs/vendored-upstreams.md](docs/vendored-upstreams.md) | upstream SHAs + local fixes |

## License

MIT (vendored packages retain their original licenses — see LICENSE).
