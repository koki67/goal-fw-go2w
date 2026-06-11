# Browser operator UI

The optional browser UI serves a static, build-free operator page on port 8080 and connects to ROS 2 through rosbridge on port 9090. It works alongside RViz and publishes only `/initialpose` and `/goal_pose`; it never publishes `/cmd_vel`.

## Navigation

Start standard bringup with the opt-in argument:

```bash
bash /external/scripts/bringup_tmux.sh map:=/external/maps/office web_ui:=true
```

Open `http://<jetson-ip>:8080` from a laptop, tablet, or phone on the robot Wi-Fi. Choose **Set initial pose** or **Send goal**, then press and drag from position toward heading. Drag the map to pan; wheel or pinch to zoom. RViz can remain connected simultaneously.

## Map preparation

```bash
bash /external/scripts/prepare_map_tmux.sh output:=/external/maps/office web_ui:=true
```

The page shows a low-rate top-down projection rather than streaming the large D-LIO cloud. When coverage is complete, press **Finish & Save** and confirm. Status advances through `SAVING`, `CONVERTING`, and `DONE <path>`. The tmux Enter flow remains available; whichever path completes first wins and the other safely refuses to overwrite the output.

## Offline and security model

`roslib.min.js` is vendored, so the page needs no internet access or frontend build. Ports 8080 and 9090 have no authentication or TLS. Expose them only on the isolated robot LAN; this uses the same trusted-network model as remote DDS/RViz access.

## Troubleshooting

- `DISCONNECTED`: verify rosbridge is running, ports 8080/9090 are reachable, and use `?rosbridge_port=<port>` when overriding the websocket port.
- `DETECTING`: wait for `/map` in navigation mode or `/web/prep_grid` in preparation mode.
- Empty preparation preview: verify `/dlio/map_node/map` is publishing.
- A late-opened page retries transient-local grid/status subscriptions every three seconds until the first message arrives.
