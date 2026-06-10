"""Composite launch: Nav2 stack + operator goal executor.

Brings up the Nav2 lifecycle nodes (planner, controller with MPPI Omni, BT
navigator, smoother, velocity_smoother, behavior server, lifecycle manager)
plus the goal_pose_executor that consumes /goal_pose (RViz "2D Nav Goal")
and dispatches NavigateToPose actions, gated on localization health.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory("nav_goal_go2w_planner")
    nav2_default_params = os.path.join(pkg_share, "config", "nav2_params.yaml")
    executor_config = os.path.join(pkg_share, "config", "goal_pose_executor.yaml")
    trajectory_lines_config = os.path.join(pkg_share, "config", "mppi_trajectory_lines.yaml")
    map_viz_config = os.path.join(pkg_share, "config", "map_viz_layers.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    nav2_extra_params_file = LaunchConfiguration("nav2_extra_params_file")
    goal_update_strategy = LaunchConfiguration("goal_update_strategy")
    require_localization = LaunchConfiguration("require_localization")
    global_map_topic = LaunchConfiguration("global_map_topic")
    global_static_layer_name = LaunchConfiguration("global_static_layer_name")
    enable_map_viz_layers = LaunchConfiguration("enable_map_viz_layers")

    nav2_launch = PathJoinSubstitution([
        FindPackageShare("nav_goal_go2w_planner"),
        "launch", "nav2_navigation.launch.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("nav2_params_file", default_value=nav2_default_params),
        DeclareLaunchArgument("nav2_extra_params_file", default_value=""),
        DeclareLaunchArgument("goal_update_strategy", default_value="preempt"),
        DeclareLaunchArgument(
            "require_localization",
            default_value="true",
            description="Gate goal dispatch on /localization/state health.",
        ),
        DeclareLaunchArgument("global_map_topic", default_value="/map"),
        DeclareLaunchArgument(
            "global_static_layer_name",
            default_value="static_layer",
            description="Plugin name of the global obstacle StaticLayer.",
        ),
        DeclareLaunchArgument(
            "enable_map_viz_layers",
            default_value="true",
            description="Publish a visualization-only global costmap with unknown cells hidden.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "nav2_params_file": nav2_params_file,
                "nav2_extra_params_file": nav2_extra_params_file,
                "global_map_topic": global_map_topic,
                "global_static_layer_name": global_static_layer_name,
            }.items(),
        ),
        Node(
            package="nav_goal_go2w_planner",
            executable="goal_pose_executor",
            output="screen",
            parameters=[
                executor_config,
                {
                    "use_sim_time": use_sim_time,
                    "goal_update_strategy": goal_update_strategy,
                    "require_localization": require_localization,
                },
            ],
        ),
        Node(
            package="nav_goal_go2w_planner",
            executable="mppi_trajectory_lines",
            name="mppi_trajectory_lines",
            output="screen",
            parameters=[trajectory_lines_config, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav_goal_go2w_planner",
            executable="map_viz_layers",
            name="map_viz_layers",
            output="screen",
            condition=IfCondition(enable_map_viz_layers),
            parameters=[map_viz_config, {"use_sim_time": use_sim_time}],
        ),
    ])
