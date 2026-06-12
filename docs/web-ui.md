# Browser operator UI

The optional browser UI serves a static, build-free operator page on port 8080 and connects to ROS 2 through rosbridge on port 9090. It works alongside RViz and publishes only `/initialpose` and `/goal_pose`; it never publishes `/cmd_vel`.

## Navigation

Start standard bringup with the opt-in argument:

```bash
bash /external/scripts/bringup_tmux.sh map:=/external/maps/office web_ui:=true
```

Open `http://<jetson-ip>:8080` from a laptop, tablet, or phone on the robot Wi-Fi. RViz can remain connected simultaneously. Drag the map to pan; scroll wheel or pinch to zoom; press **Fit map** to reset the view.

### 3D point cloud view

Press **3D View** to switch the map area to a perspective point cloud rendered like RViz: points are colored by height, with the robot pose (white triangle), planned path (blue line), and active goal (green marker) overlaid. Drag to orbit, scroll or pinch to zoom, right-drag (or Shift-drag, or two-finger drag) to pan, and **Fit map** to re-center on the cloud. In navigation mode the view shows the latched `/map_cloud`; in preparation mode it shows `/web/prep_cloud`, a voxel-downsampled snapshot of the live D-LIO map refreshed every preview interval (`preview_leaf_size` / `preview_max_points` parameters bound the bandwidth). The pose tools remain top-down operations: selecting **2D Pose Estimate** or **2D Nav Goal** switches back to the 2D view automatically. Rendering uses plain WebGL with no extra dependencies, so the page stays build-free and offline.

### Step-by-step workflow (mirrors RViz)

1. **Set the initial pose — 2D Pose Estimate**
   Click **2D Pose Estimate** (same name as in RViz), then on the map drag from the robot's physical position toward the direction it faces and release. The Localization status changes to `CONVERGING`, then `TRACKING` within ~5 s. If it stays `CONVERGING` for more than 5 s, click **2D Pose Estimate** again and pay attention to the heading direction.

2. **Send a navigation goal — 2D Nav Goal**
   Once Localization shows `TRACKING`, click **2D Nav Goal** (same name as in RViz), then on the map drag from the destination point toward the robot's desired arrival heading and release. The Goal status changes to `ACTIVE`, the blue line shows the planned path, and the green marker shows the active goal. The robot navigates autonomously.

3. **Re-route**
   Click **2D Nav Goal** again at any time to send a new goal; the previous goal is preempted within one control cycle.

4. **If localization degrades**
   If Localization shows `DEGRADED` or `LOST`, click **2D Pose Estimate** again to re-initialize. The active goal is automatically canceled.

> The hint text at the bottom of the map updates automatically to reflect the current step.

## Map preparation

```bash
bash /external/scripts/prepare_map_tmux.sh output:=/external/maps/office web_ui:=true
```

The page shows a low-rate preview rather than streaming the full D-LIO cloud: a top-down projection by default, or a voxel-downsampled 3D cloud via the **3D View** button. When coverage is complete, press **Finish & Save** and confirm. Status advances through `SAVING`, `CONVERTING`, and `DONE <path>`. A few seconds after `DONE`, the whole collection launch (Hesai, IMU, D-LIO, rosbridge, and this page's server) shuts down automatically — the same as Ctrl-C in the tmux `collect` window — so the page switches to `DISCONNECTED` and goal navigation can be started without duplicate publishers. The tmux Enter flow remains available; whichever path completes first wins and the other safely refuses to overwrite the output.

## Offline and security model

`roslib.min.js` is vendored, so the page needs no internet access or frontend build. Ports 8080 and 9090 have no authentication or TLS. Expose them only on the isolated robot LAN; this uses the same trusted-network model as remote DDS/RViz access.

## Troubleshooting

- `DISCONNECTED`: verify rosbridge is running, ports 8080/9090 are reachable, and use `?rosbridge_port=<port>` when overriding the websocket port. Right after `DONE <path>` in preparation mode this is expected: finalization stops the collection launch, including rosbridge.
- `DETECTING`: wait for `/map` in navigation mode or `/web/prep_grid` in preparation mode.
- Empty preparation preview: verify `/dlio/map_node/map` is publishing.
- A late-opened page retries transient-local grid/status subscriptions every three seconds until the first message arrives.
