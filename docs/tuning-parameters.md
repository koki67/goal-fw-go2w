# Tuning parameters

Where the knobs live and which ones actually matter, per subsystem.

## Map preparation (`prepare_map` CLI flags)

| Knob | Default | Raise when | Lower when |
|---|---|---|---|
| `--resolution` | 0.05 | huge outdoor maps (0.10) | tight indoor clutter |
| `--obstacle-z-min` | 0.15 | floor bleeds into the grid | low obstacles missed (curbs) |
| `--obstacle-z-max` | 1.5 | — | tall shelves over-mark passable underhangs |
| `--loc-voxel` | 0.20 | registration too slow on Jetson (0.3) | sparse maps, poor convergence (0.15) |
| `--min-obstacle-cells` | 3 | speckle noise | thin real obstacles vanish (poles!) |

## Localization (`nav_goal_go2w_localization/config/localization.yaml`)

Most impactful first:

1. `registration_rate_hz` (2.0) — CPU vs correction latency. 1.0 is fine
   at walking speeds; odometry covers the gap.
2. `scan_voxel_size` (0.25) / `num_threads` (4) — direct CPU knobs.
3. `min_inlier_fraction` (0.6) / `max_mean_error` (0.10) — the accept/reject
   line. Record a healthy run, look at `/localization/fitness`, place the
   thresholds with ~2x margin over the typical value.
4. `max_translation_jump_m` (0.5) / `jump_confirm_count` (3) — how fast a
   genuine odometry slip is re-accepted vs glitch immunity.
5. `correction_blend` (1.0) — set 0.3–0.7 if the published pose jitters.
6. `registration_type` (GICP) — VGICP trades a little accuracy for cheaper
   correspondence queries on big maps.

## Goal executor (`nav_goal_go2w_planner/config/goal_pose_executor.yaml`)

- `goal_update_strategy` (preempt) — `queue` only makes sense for scripted
  multi-goal use.
- `goal_timeout_sec` (300) — scale with site size.
- `treat_unknown_as_reachable` (true) — set false only when the map is
  known-complete and you want hard geofencing to mapped space.
- `dispatch_localization_states` / `cancel_localization_states` — tighten to
  `["TRACKING"]` / `["DEGRADED", "LOST"]` for conservative deployments.

## Nav2 (`nav_goal_go2w_planner/config/nav2_params.yaml`)

Unchanged from the frontier stack (already Go2W-tuned): NavFn
`allow_unknown: true`, tolerance 0.5; MPPI Omni ceilings vx 0.8 / vy 0.6 /
wz 1.0; footprint ±0.35 × ±0.215 m; inflation 0.45 m. The **effective**
speed caps are the bridge's (`vx_max` etc. on the bringup command line,
default 0.30/0.20/0.50) — raise those last, after everything else is solid.

## Velocity bridge (`nav_goal_go2w_bridge/config/velocity_bridge.yaml`)

Same as frontier-fw-go2w: `watchdog_timeout` 0.5 s, 50 Hz, dry-run default
true at bringup. Don't touch the api ids.

## pointcloud_to_laserscan (`nav_goal_go2w_bringup/config/pointcloud_to_laserscan.yaml`)

`min_height`/`max_height` (-0.10 / 0.40 in base_link) decide what the LIVE
local costmap can see — the first knob to check when the robot ignores or
hallucinates obstacles that the static map doesn't explain.
