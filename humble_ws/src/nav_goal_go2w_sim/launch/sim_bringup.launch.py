"""Closed-loop desktop simulation of the full goal navigation stack.

sim_node (fake odometry + deskewed cloud + /scan, optional odometry drift)
+ map servers (grid + cloud from a gen_sim_map directory)
+ scan-to-map localizer (or a static identity map->odom with
  sim_localization:=false, to debug Nav2/executor in isolation)
+ Nav2 + goal_pose_executor + RViz.

Workflow: generate the map once
    ros2 run nav_goal_go2w_sim gen_sim_map --world open_room --output maps/sim_open_room
then
    ros2 launch nav_goal_go2w_sim sim_bringup.launch.py map:=maps/sim_open_room \
        odom_drift_yaw_per_m:=0.02
and click 2D Pose Estimate + 2D Nav Goal in RViz.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    sim_share = get_package_share_directory("nav_goal_go2w_sim")
    map_share = get_package_share_directory("nav_goal_go2w_map")
    loc_share = get_package_share_directory("nav_goal_go2w_localization")
    planner_share = get_package_share_directory("nav_goal_go2w_planner")

    world_file = LaunchConfiguration("world_file").perform(context)
    map_dir = LaunchConfiguration("map").perform(context)
    sim_localization = (
        LaunchConfiguration("sim_localization").perform(context) == "true"
    )
    use_rviz = LaunchConfiguration("use_rviz").perform(context) == "true"

    if not map_dir:
        # Convenience fallback: maps/sim_<world> relative to the working dir.
        candidate = os.path.join(os.getcwd(), "maps", f"sim_{world_file}")
        if os.path.isfile(os.path.join(candidate, "grid.yaml")):
            map_dir = candidate
        else:
            raise RuntimeError(
                "map:= is required (run `ros2 run nav_goal_go2w_sim gen_sim_map "
                f"--world {world_file} --output maps/sim_{world_file}` first)"
            )

    actions = [
        Node(
            package="nav_goal_go2w_sim",
            executable="sim_node",
            name="sim_node",
            output="screen",
            parameters=[
                os.path.join(sim_share, "config", "sim_node.yaml"),
                {
                    "world_file": world_file,
                    "odom_drift_x_per_m": float(
                        LaunchConfiguration("odom_drift_x_per_m").perform(context)
                    ),
                    "odom_drift_yaw_per_m": float(
                        LaunchConfiguration("odom_drift_yaw_per_m").perform(context)
                    ),
                    "num_dynamic_obstacles": int(
                        LaunchConfiguration("num_dynamic_obstacles").perform(context)
                    ),
                },
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(map_share, "launch", "map_servers.launch.py")
            ),
            launch_arguments={
                "map": map_dir,
                "use_sim_time": "true",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(planner_share, "launch", "goal_planner.launch.py")
            ),
            launch_arguments={
                "use_sim_time": "true",
                "goal_update_strategy": LaunchConfiguration(
                    "goal_update_strategy"
                ).perform(context),
                "require_localization": (
                    "true" if sim_localization else "false"
                ),
            }.items(),
        ),
    ]

    if sim_localization:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(loc_share, "launch", "localization.launch.py")
                ),
                launch_arguments={
                    "map": map_dir,
                    "use_sim_time": "true",
                }.items(),
            )
        )
    else:
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom_static_tf",
                output="screen",
                arguments=[
                    "--frame-id", "map", "--child-frame-id", "odom",
                ],
            )
        )

    if use_rviz:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=[
                    "-d", os.path.join(sim_share, "config", "goal_sim.rviz"),
                ],
                parameters=[{"use_sim_time": True}],
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("world_file", default_value="open_room"),
            DeclareLaunchArgument(
                "map",
                default_value="",
                description="gen_sim_map output directory for the same world",
            ),
            DeclareLaunchArgument(
                "sim_localization",
                default_value="true",
                description="false = static identity map->odom (skip localizer)",
            ),
            DeclareLaunchArgument("odom_drift_x_per_m", default_value="0.0"),
            DeclareLaunchArgument("odom_drift_yaw_per_m", default_value="0.0"),
            DeclareLaunchArgument("num_dynamic_obstacles", default_value="0"),
            DeclareLaunchArgument("goal_update_strategy", default_value="preempt"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            OpaqueFunction(function=_setup),
        ]
    )
