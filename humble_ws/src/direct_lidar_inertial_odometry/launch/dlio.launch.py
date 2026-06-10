#
#   Copyright (c)     
#
#   The Verifiable & Control-Theoretic Robotics (VECTR) Lab
#   University of California, Los Angeles
#
#   Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez
#   Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition   
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    current_pkg = FindPackageShare('direct_lidar_inertial_odometry')

    # Set default arguments
    rviz = LaunchConfiguration('rviz', default='false')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic', default='points_raw')
    imu_topic = LaunchConfiguration('imu_topic', default='go2w/imu')
    dlio_output = LaunchConfiguration('dlio_output', default='screen')
    dlio_log_dir = LaunchConfiguration('dlio_log_dir', default='')

    # Define arguments
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value=rviz,
        description='Launch RViz'
    )
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time,
        description='Use simulated time from /clock'
    )
    declare_pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value=pointcloud_topic,
        description='Pointcloud topic name'
    )
    declare_imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value=imu_topic,
        description='IMU topic name'
    )
    declare_dlio_output_arg = DeclareLaunchArgument(
        'dlio_output',
        default_value=dlio_output,
        description='D-LIO node output target: screen or log'
    )
    declare_dlio_log_dir_arg = DeclareLaunchArgument(
        'dlio_log_dir',
        default_value=dlio_log_dir,
        description='Optional ROS_LOG_DIR override for D-LIO nodes'
    )

    # Load parameters
    dlio_yaml_path = PathJoinSubstitution([current_pkg, 'cfg', 'dlio.yaml'])
    dlio_params_yaml_path = PathJoinSubstitution([current_pkg, 'cfg', 'params.yaml'])

    def _dlio_nodes(context, *args, **kwargs):
        log_dir = dlio_log_dir.perform(context).strip()
        node_kwargs = {}
        if log_dir:
            node_kwargs['additional_env'] = {'ROS_LOG_DIR': log_dir}

        # DLIO Odometry Node
        dlio_odom_node = Node(
            package='direct_lidar_inertial_odometry',
            executable='dlio_odom_node',
            output=dlio_output,
            parameters=[dlio_yaml_path, dlio_params_yaml_path, {'use_sim_time': use_sim_time}],
            remappings=[
                ('pointcloud', pointcloud_topic),
                ('imu', imu_topic),
                ('odom', 'dlio/odom_node/odom'),
                ('pose', 'dlio/odom_node/pose'),
                ('path', 'dlio/odom_node/path'),
                ('kf_pose', 'dlio/odom_node/keyframes'),
                ('kf_cloud', 'dlio/odom_node/pointcloud/keyframe'),
                ('deskewed', 'dlio/odom_node/pointcloud/deskewed'),
            ],
            **node_kwargs,
        )

        # DLIO Mapping Node
        # 'map' remapped to 'dlio/map_node/map' to avoid collision with slam_toolbox /map.
        dlio_map_node = Node(
            package='direct_lidar_inertial_odometry',
            executable='dlio_map_node',
            output=dlio_output,
            parameters=[dlio_yaml_path, dlio_params_yaml_path, {'use_sim_time': use_sim_time}],
            remappings=[
                ('keyframes', 'dlio/odom_node/pointcloud/keyframe'),
                ('map', 'dlio/map_node/map'),
            ],
            **node_kwargs,
        )
        return [dlio_odom_node, dlio_map_node]

    # RViz node
    rviz_config_path = PathJoinSubstitution([current_pkg, 'launch', 'dlio.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='dlio_rviz',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        declare_rviz_arg,
        declare_use_sim_time_arg,
        declare_pointcloud_topic_arg,
        declare_imu_topic_arg,
        declare_dlio_output_arg,
        declare_dlio_log_dir_arg,
        OpaqueFunction(function=_dlio_nodes),
        rviz_node
    ])
