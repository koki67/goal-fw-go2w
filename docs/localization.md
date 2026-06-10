# Scan-to-map localization

`nav_goal_go2w_localization/scan_to_map_localizer` keeps the robot localized
inside the pre-built map by registering live LiDAR clouds against
`map.pcd` with [small_gicp](https://github.com/koide3/small_gicp) and
publishing the `map -> odom` TF correction on top of D-LIO odometry.

## Why this decomposition works

D-LIO publishes its deskewed cloud **already in the odom frame**
(`direct_lidar_inertial_odometry/src/dlio/odom.cc`, `publishCloud`).
Registering an odom-frame source against the map-frame target with the
current `T_map_odom` as the initial guess returns the refined `T_map_odom`
directly — no per-point pose bookkeeping, no extrinsics, deskewing for free.
D-LIO handles fast motion at full rate; the localizer only corrects the
slow drift of odom relative to the map, so 1–2 Hz registration is enough.

## State machine

```
UNINITIALIZED --/initialpose--> CONVERGING --3 good--> TRACKING
TRACKING --4 consecutive rejects--> DEGRADED --12 total--> LOST
DEGRADED/LOST --good registration--> TRACKING
LOST --/initialpose--> CONVERGING
```

- **UNINITIALIZED**: no TF published; Nav2 stays inert. Waiting for the
  operator's RViz "2D Pose Estimate".
- **CONVERGING**: relaxed correspondence distance
  (`converging_max_correspondence_distance`, 2.0 m) absorbs a coarse click;
  no jump gating; accepted transforms replace `T_map_odom` outright.
- **TRACKING**: full gating (below). Goal dispatch allowed.
- **DEGRADED**: several consecutive rejected registrations — pose is held
  from odometry alone. Goals still dispatch; the operator should slow down.
- **LOST**: the executor cancels the active goal; the velocity bridge
  watchdog stops the robot. Recover by re-clicking 2D Pose Estimate.

## Gating

A registration result moves `T_map_odom` only if ALL pass:

| Gate | Parameter | Default |
|---|---|---|
| enough scan points after crop/downsample | `min_points` | 200 |
| optimizer converged | — | — |
| inlier fraction | `min_inlier_fraction` | 0.6 |
| mean residual per inlier | `max_mean_error` | 0.10 |
| jump vs current estimate (TRACKING/DEGRADED) | `max_translation_jump_m` / `max_rotation_jump_deg` | 0.5 m / 10° |

**Jump confirmation**: a genuine odometry slip produces the *same* large
correction in consecutive registrations; a registration glitch does not.
After `jump_confirm_count` (3) consecutive consistent jumps the correction
is accepted.

`constrain_2d: true` flattens `T_map_odom` to yaw-only so registration noise
can never tilt the map frame (which would skew every 2D costmap lookup); the
z translation is kept since it corrects genuine odometry z drift.

## TF timing

Registration runs at `registration_rate_hz` (2.0) in its own callback group;
a separate 50 Hz timer re-broadcasts the latest `T_map_odom` future-dated by
`transform_tolerance` (0.3 s) — the same trick AMCL uses — so Nav2 TF
lookups never extrapolate between registration updates.

## Tuning

Start with `config/localization.yaml`. The most likely knobs:

- **Jetson CPU too high** → `registration_rate_hz: 1.0`,
  `scan_voxel_size: 0.4`, or `registration_type: VGICP` (cheaper queries).
- **Rejects in open/sparse areas** → lower `min_inlier_fraction`, raise
  `max_mean_error` (watch `/localization/fitness` for the working range).
- **Pose jitter** → `correction_blend: 0.5` (exponential smoothing toward
  each accepted registration).
- **Initial clicks failing** → raise
  `converging_max_correspondence_distance`; click with more care for yaw
  (the gate that usually matters).

## Diagnostics

`/diagnostics` entry `goal_nav/localizer` carries per-cycle values:
`inlier_fraction`, `mean_error`, `source_points`, `registration_ms`,
`cycles_since_accept`, plus the last gate-rejection reason. Registration
latency on the Jetson should stay well under 200 ms; if not, see tuning.
