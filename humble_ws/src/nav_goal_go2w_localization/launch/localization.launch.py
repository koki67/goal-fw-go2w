"""Launch the scan-to-map localizer against a prepared map directory."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    map_dir = LaunchConfiguration("map").perform(context)
    params_file = LaunchConfiguration("localization_params_file").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context) == "true"

    map_pcd = os.path.join(map_dir, "map.pcd")
    if not os.path.isfile(map_pcd):
        raise RuntimeError(
            f"map directory {map_dir!r} is missing map.pcd; "
            "run `ros2 run nav_goal_go2w_map prepare_map` first"
        )

    return [
        Node(
            package="nav_goal_go2w_localization",
            executable="scan_to_map_localizer",
            name="scan_to_map_localizer",
            output="screen",
            parameters=[
                params_file,
                {"use_sim_time": use_sim_time, "map_path": map_pcd},
            ],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    default_params = os.path.join(
        get_package_share_directory("nav_goal_go2w_localization"),
        "config",
        "localization.yaml",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map", description="prepared map directory (contains map.pcd)"
            ),
            DeclareLaunchArgument(
                "localization_params_file", default_value=default_params
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_setup),
        ]
    )
