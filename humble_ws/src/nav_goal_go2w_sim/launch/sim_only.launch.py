"""Launch only the desktop 2D simulator.

This is useful for checking the simulated scan, odometry, TF, and world topics
without starting slam_toolbox or Nav2.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world_file = LaunchConfiguration("world_file")
    config_file = LaunchConfiguration("config_file")

    declared_args = [
        DeclareLaunchArgument("world_file", default_value="open_room"),
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("nav_goal_go2w_sim"),
                "config",
                "sim_node.yaml",
            ]),
        ),
        DeclareLaunchArgument("use_world_spawn", default_value="true"),
        DeclareLaunchArgument("start_x", default_value="0.0"),
        DeclareLaunchArgument("start_y", default_value="0.0"),
        DeclareLaunchArgument("start_yaw", default_value="0.0"),
        DeclareLaunchArgument("vx_max", default_value="0.30"),
        DeclareLaunchArgument("vy_max", default_value="0.20"),
        DeclareLaunchArgument("wz_max", default_value="0.50"),
        DeclareLaunchArgument("robot_length", default_value="0.70"),
        DeclareLaunchArgument("robot_width", default_value="0.43"),
        DeclareLaunchArgument("scan_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("physics_rate_hz", default_value="50.0"),
        DeclareLaunchArgument("clock_rate_hz", default_value="200.0"),
        DeclareLaunchArgument("publish_clock", default_value="true"),
        DeclareLaunchArgument("keyboard_pause", default_value="false"),
        DeclareLaunchArgument("num_dynamic_obstacles", default_value="0"),
        DeclareLaunchArgument("dynamic_obstacle_radius", default_value="0.20"),
        DeclareLaunchArgument("dynamic_obstacle_speed", default_value="0.30"),
        DeclareLaunchArgument("dynamic_obstacles_seed", default_value="0"),
        DeclareLaunchArgument("dynamic_obstacle_min_spawn_distance", default_value="1.5"),
    ]

    sim_node = Node(
        package="nav_goal_go2w_sim",
        executable="sim_node",
        name="sim_node",
        output="screen",
        parameters=[
            config_file,
            {
                "world_file": world_file,
                "use_world_spawn": LaunchConfiguration("use_world_spawn"),
                "start_x": LaunchConfiguration("start_x"),
                "start_y": LaunchConfiguration("start_y"),
                "start_yaw": LaunchConfiguration("start_yaw"),
                "vx_max": LaunchConfiguration("vx_max"),
                "vy_max": LaunchConfiguration("vy_max"),
                "wz_max": LaunchConfiguration("wz_max"),
                "robot_length": LaunchConfiguration("robot_length"),
                "robot_width": LaunchConfiguration("robot_width"),
                "scan_rate_hz": LaunchConfiguration("scan_rate_hz"),
                "physics_rate_hz": LaunchConfiguration("physics_rate_hz"),
                "clock_rate_hz": LaunchConfiguration("clock_rate_hz"),
                "publish_clock": LaunchConfiguration("publish_clock"),
                "num_dynamic_obstacles": LaunchConfiguration("num_dynamic_obstacles"),
                "dynamic_obstacle_radius": LaunchConfiguration("dynamic_obstacle_radius"),
                "dynamic_obstacle_speed": LaunchConfiguration("dynamic_obstacle_speed"),
                "dynamic_obstacles_seed": LaunchConfiguration("dynamic_obstacles_seed"),
                "dynamic_obstacle_min_spawn_distance": LaunchConfiguration("dynamic_obstacle_min_spawn_distance"),
            },
        ],
    )

    keyboard_pause_node = Node(
        package="nav_goal_go2w_sim",
        executable="keyboard_pause_node",
        name="sim_keyboard_pause",
        output="screen",
        condition=IfCondition(LaunchConfiguration("keyboard_pause")),
    )

    return LaunchDescription([*declared_args, sim_node, keyboard_pause_node])
