"""Serve a prepared map directory: occupancy grid on /map, cloud on /map_cloud.

The map_server gets its own lifecycle manager (separate from navigation) so
the grid is latched and available before Nav2 costmaps come up.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def _validate_and_setup(context, *args, **kwargs):
    map_dir = LaunchConfiguration("map").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context) == "true"

    grid_yaml = os.path.join(map_dir, "grid.yaml")
    viz_pcd = os.path.join(map_dir, "viz.pcd")
    for required in (grid_yaml, viz_pcd):
        if not os.path.isfile(required):
            raise RuntimeError(
                f"map directory {map_dir!r} is missing {os.path.basename(required)}; "
                "run `ros2 run nav_goal_go2w_map prepare_map` first"
            )

    return [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "yaml_filename": grid_yaml,
                    "topic_name": "map",
                    "frame_id": "map",
                }
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_map",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "autostart": True,
                    "node_names": ["map_server"],
                }
            ],
        ),
        Node(
            package="nav_goal_go2w_map",
            executable="map_cloud_publisher",
            name="map_cloud_publisher",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "cloud_path": viz_pcd,
                    "frame_id": "map",
                }
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                description="prepared map directory (contains grid.yaml, viz.pcd)",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_validate_and_setup),
        ]
    )
