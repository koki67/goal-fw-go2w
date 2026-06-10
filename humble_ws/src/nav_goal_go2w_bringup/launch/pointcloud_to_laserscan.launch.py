"""
Launch the pointcloud_to_laserscan_node that projects /points_raw (Hesai
PointCloud2, frame: hesai_lidar) into a 2D /scan (frame: base_link) for
the local costmap.

Standalone launch — useful for debugging the projection without bringing up
the local costmap. Top-level launch (go2w_hesai_the local costmap.launch.py) includes
this same file.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('nav_goal_go2w_bringup')
    default_params = os.path.join(pkg_share, 'config', 'pointcloud_to_laserscan.yaml')

    params_file = LaunchConfiguration('params_file')
    cloud_topic = LaunchConfiguration('cloud_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to pointcloud_to_laserscan parameter YAML.',
        ),
        DeclareLaunchArgument(
            'cloud_topic',
            default_value='/points_raw',
            description='Input PointCloud2 topic from the Hesai driver.',
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='Output LaserScan topic consumed by the local costmap.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock from a bag/sim instead of system time.',
        ),

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('cloud_in', cloud_topic),
                ('scan', scan_topic),
            ],
        ),
    ])
