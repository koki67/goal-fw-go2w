# goal-fw-go2w

Goal-directed autonomous navigation framework for the Unitree Go2-W.

Given a pre-built 3D point cloud map — produced by
[frontier-fw-go2w](../frontier-fw-go2w) (D-LIO `save_pcd`) or by a human
carrying a LiDAR through the environment — this stack localizes the robot
against that map and navigates to goals picked by an operator in RViz.

Operator workflow:

1. Prepare the map once: `ros2 run nav_goal_go2w_map prepare_map --input raw.pcd --output maps/<name>`
2. Bring up the stack: `ros2 launch nav_goal_go2w_bringup bringup.launch.py map:=/external/maps/<name>`
3. RViz shows the pre-built map. Click **2D Pose Estimate** at the robot's
   actual location; wait for localization state `TRACKING`.
4. Click **2D Nav Goal** anywhere on the map. The robot navigates there
   autonomously. A new click preempts the current goal.

See `docs/` for architecture, map preparation, localization internals,
simulation, and the operator runbook.

(README will be expanded as milestones land — see docs/architecture.md first.)
