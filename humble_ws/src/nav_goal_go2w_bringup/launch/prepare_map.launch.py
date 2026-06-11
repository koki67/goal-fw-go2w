"""Collect a D-LIO point cloud map from a remotely controlled Go2W.

This launch intentionally starts no teleoperation, navigation, localization,
or velocity bridge nodes. The operator must provide a separate remote-control
system, such as go2w_teleop_gamepad, before starting collection.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    dlio_output = LaunchConfiguration("dlio_output")

    declared_args = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock from a bag or simulator.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Start RViz with D-LIO's live mapping view.",
        ),
        DeclareLaunchArgument(
            "dlio_output",
            default_value="screen",
            description="D-LIO node output target: screen or log.",
        ),
    ]

    hesai_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("hesai_lidar"), "launch", "hesai_lidar_launch.py"]
            )
        ),
    )

    imu_node = Node(
        package="go2w_imu_publisher",
        executable="imu_publisher",
        name="go2w_imu_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    dlio_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("direct_lidar_inertial_odometry"),
                    "launch",
                    "dlio.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "rviz": "false",
            "dlio_output": dlio_output,
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="prepare_map_rviz",
        output="screen",
        condition=IfCondition(use_rviz),
        arguments=[
            "-d",
            PathJoinSubstitution(
                [
                    FindPackageShare("direct_lidar_inertial_odometry"),
                    "launch",
                    "dlio.rviz",
                ]
            ),
        ],
        remappings=[("/map", "/dlio/map_node/map")],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [*declared_args, hesai_include, imu_node, dlio_include, rviz_node]
    )
