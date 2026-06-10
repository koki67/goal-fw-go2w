# Goal navigation — operator runbook

## Bring up (robot, Jetson)

```bash
bash docker/run.sh                       # enter the container
bash /external/scripts/bringup_tmux.sh map:=/external/maps/office
```

tmux windows: `dlio` (odometry), `bringup` (everything else), `health`
(`/localization/state` echo). The bridge starts in **dry-run** — Move/Stop
are logged, the robot does not move — until you pass
`bridge_dry_run:=false`.

On the desktop:

```bash
.devcontainer/start_remote_rviz.bash <wifi-iface>
```

## Drive sequence

1. **Initial pose**: RViz → *2D Pose Estimate* → click at the robot's true
   position, drag toward its heading. Health window: `CONVERGING` →
   `TRACKING` within a few seconds. Not tracking after ~5 s? Click again,
   paying attention to yaw.
2. **Goal**: RViz → *2D Nav Goal* → click destination (drag = final
   heading). Status text appears at the goal marker; the green arrow is the
   active goal.
3. **Re-route**: just click a new goal — the default strategy is `preempt`,
   the previous goal is canceled within a control cycle.
4. **Stop**: `Ctrl-C` in the bringup window (the bridge watchdog halts the
   robot within 0.5 s), or send a goal at the robot's own position.

## Goal acceptance rules

A clicked goal is **rejected** (with a log + `/goal_executor/status`) when:

- localization is not TRACKING/DEGRADED (`require_localization`)
- the goal cell is occupied on `/map` (unknown space is allowed — the map
  may be incomplete; the live local costmap handles surprises)
- the goal frame is not `map`

An **active** goal is canceled when: a new goal preempts it, it times out
(`goal_timeout_sec`, 300 s), the map shows it unreachable, or localization
reports LOST.

## First live run (recommended sequence)

1. Dry-run bringup; click initial pose + goal; verify `[DRY_RUN] Move ...`
   lines in the bridge log and a sane RViz plan.
2. `bridge_dry_run:=false vx_max:=0.2` with a short (2–3 m) goal in open
   space; keep a hand on the e-stop.
3. Test preemption (second click mid-run) and the kill path (Ctrl-C →
   robot stops).
4. Cover the LiDAR briefly: localization should go DEGRADED → LOST and the
   goal must cancel.
