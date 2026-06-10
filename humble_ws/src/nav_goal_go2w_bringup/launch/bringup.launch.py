"""Top-level launch for the full Go2W goal navigation stack.

Stages composed (in start order):
    1. Static TFs        base_link -> imu_link, base_link -> hesai_lidar
    2. Robot model       robot_state_publisher with the Go2W URDF
    3. Hesai LiDAR       publishes /points_raw (PointCloud2)
    4. IMU publisher     publishes /go2w/imu (sensor_msgs/Imu)
    5. D-LIO             publishes /dlio/odom_node/odom + TF odom -> base_link
    6. Map servers       nav2 map_server (/map) + map cloud publisher
                         (/map_cloud) from the prepared map directory
    7. Localizer         scan-to-map registration -> TF map -> odom
                         (waits for /initialpose from RViz)
    8. p2l               /points_raw -> /scan for the Nav2 local costmap
    9. Nav2 + executor   /goal_pose -> NavigateToPose -> /cmd_vel
   10. Velocity bridge   /cmd_vel -> /api/sport/request (Sport API)
   11. RViz              optional, gated by use_rviz launch arg

The `map` argument is REQUIRED and must point to a directory produced by
`ros2 run nav_goal_go2w_map prepare_map` (or gen_sim_map).

Static TFs use the Go2W extrinsics from D-LIO's dlio.yaml:
    base_link -> hesai_lidar : t=[0.1634, 0, 0.116]   yaw=+pi/2
    base_link -> imu_link    : t=[0,      0, 0]       yaw=+pi/2
"""
import math
import os
from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

RESULT_RECORDING_REGEX = (
    r"^("
    r"/clock|"
    r"/robot_description|"
    r"/tf|/tf_static|"
    r"/map|/map_updates|/map_cloud|"
    r"/scan|"
    r"/initialpose|/goal_pose|/goal_markers|"
    r"/goal_executor/status|"
    r"/localization/(pose|state|fitness)|/diagnostics|"
    r"/dlio/odom_node/(odom|pose|path|keyframes)|"
    r"/dlio/odom_node/pointcloud/(keyframe|deskewed)|"
    r"/dlio/map_node/map|"
    r"/global_costmap/(costmap|costmap_raw|costmap_updates|published_footprint)|"
    r"/local_costmap/(costmap|costmap_raw|costmap_updates|published_footprint)|"
    r"/(plan|global_plan|local_plan|received_global_plan|transformed_global_plan|trajectories|trajectory)|"
    r"/mppi_trajectory_lines|"
    r"/cmd_vel_nav|/cmd_vel|"
    r"/api/sport/(request|response)|"
    r"/navigate_to_pose/_action/(goal|feedback|status|result|cancel)|"
    r"/compute_path_to_pose/_action/(goal|feedback|status|result|cancel)|"
    r"/follow_path/_action/(goal|feedback|status|result|cancel)|"
    r"/smooth_path/_action/(goal|feedback|status|result|cancel)"
    r")$"
)

_TRUE_VALUES = {"1", "true", "yes", "on"}


def _yaw_quat_z_w(yaw_rad: float) -> tuple[str, str]:
    return (str(math.sin(yaw_rad / 2.0)), str(math.cos(yaw_rad / 2.0)))


def _launch_bool(value: str) -> bool:
    return value.strip().lower() in _TRUE_VALUES


def _default_ros_log_dir() -> str:
    ros_home = Path(os.environ.get("ROS_HOME", "~/.ros")).expanduser()
    return str(ros_home / "log")


def _result_recording_setup_actions(context, *args, **kwargs):
    if not _launch_bool(LaunchConfiguration("record_results").perform(context)):
        return []

    bag_dir = Path(LaunchConfiguration("record_bag_dir").perform(context)).expanduser()
    bag_prefix = LaunchConfiguration("record_bag_prefix").perform(context).strip() or "goal_nav_results"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_path = bag_dir / f"{bag_prefix}_{timestamp}"
    logs_dir = output_path / "logs"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    default_log_dir = os.environ.get("ROS_LOG_DIR") or _default_ros_log_dir()

    return [
        SetLaunchConfiguration("record_output_path", str(output_path)),
        SetLaunchConfiguration("record_logs_dir", str(logs_dir)),
        SetLaunchConfiguration("record_default_log_dir", default_log_dir),
        SetLaunchConfiguration("dlio_log_dir_override", default_log_dir),
        LogInfo(msg=f"Recording goal navigation logs to {logs_dir}"),
    ]


def _result_recording_bag_actions(context, *args, **kwargs):
    if not _launch_bool(LaunchConfiguration("record_results").perform(context)):
        return []

    storage = LaunchConfiguration("record_storage").perform(context).strip() or "sqlite3"
    output_path = LaunchConfiguration("record_output_path").perform(context)
    default_log_dir = LaunchConfiguration("record_default_log_dir").perform(context)

    return [
        LogInfo(msg=f"Recording goal navigation rosbag to {output_path}"),
        ExecuteProcess(
            cmd=[
                "ros2", "bag", "record",
                "--storage", storage,
                "--include-hidden-topics",
                "--regex", RESULT_RECORDING_REGEX,
                "-o", str(output_path),
            ],
            additional_env={"ROS_LOG_DIR": default_log_dir},
            output="screen",
        ),
    ]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    map_dir = LaunchConfiguration("map")
    bridge_dry_run = LaunchConfiguration("bridge_dry_run")
    vx_max = LaunchConfiguration("vx_max")
    vy_max = LaunchConfiguration("vy_max")
    wz_max = LaunchConfiguration("wz_max")
    goal_update_strategy = LaunchConfiguration("goal_update_strategy")
    require_localization = LaunchConfiguration("require_localization")
    registration_rate = LaunchConfiguration("registration_rate")
    localization_params_file = LaunchConfiguration("localization_params_file")
    dlio_output = LaunchConfiguration("dlio_output")
    with_dlio = LaunchConfiguration("with_dlio")
    dlio_log_dir_override = LaunchConfiguration("dlio_log_dir_override", default="")
    enable_map_viz_layers = LaunchConfiguration("enable_map_viz_layers")

    def _validate_launch_args(context, *args, **kwargs):
        declared_arg_names = frozenset(a.name for a in declared_args)
        unknown = sorted(set(context.launch_configurations.keys()) - declared_arg_names)
        if unknown:
            raise RuntimeError(
                "\n[bringup.launch.py] Unknown launch argument(s): "
                + ", ".join(f"'{a}'" for a in unknown)
                + "\nValid arguments are: "
                + ", ".join(sorted(declared_arg_names))
            )
        resolved_map = map_dir.perform(context).strip()
        if not resolved_map:
            raise RuntimeError(
                "\n[bringup.launch.py] map:= is required. Point it at a directory "
                "produced by `ros2 run nav_goal_go2w_map prepare_map` "
                "(must contain map.pcd, grid.yaml, viz.pcd)."
            )
        for artifact in ("map.pcd", "grid.yaml", "viz.pcd"):
            if not os.path.isfile(os.path.join(resolved_map, artifact)):
                raise RuntimeError(
                    f"\n[bringup.launch.py] map directory {resolved_map!r} is "
                    f"missing {artifact}; run prepare_map first."
                )
        return []

    declared_args = [
        DeclareLaunchArgument("map",
                              description="Prepared map directory (map.pcd + grid.yaml + viz.pcd)."),
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="Use /clock from a bag or sim."),
        DeclareLaunchArgument("use_rviz", default_value="false",
                              description="Start RViz with the goal navigation config."),
        DeclareLaunchArgument("bridge_dry_run", default_value="true",
                              description="Bridge logs Move/Stop without publishing /api/sport/request."),
        DeclareLaunchArgument("vx_max", default_value="0.30"),
        DeclareLaunchArgument("vy_max", default_value="0.20"),
        DeclareLaunchArgument("wz_max", default_value="0.50"),
        DeclareLaunchArgument("goal_update_strategy", default_value="preempt",
                              description="Goal update mode: preempt (operator click wins) or queue."),
        DeclareLaunchArgument("require_localization", default_value="true",
                              description="Gate goal dispatch on localization health."),
        DeclareLaunchArgument("registration_rate", default_value="2.0",
                              description="Scan-to-map registration rate [Hz]."),
        DeclareLaunchArgument("localization_params_file", default_value="",
                              description="Override YAML for the localizer (default: package config)."),
        DeclareLaunchArgument("dlio_output", default_value="screen",
                              description="D-LIO node output target: screen or log."),
        DeclareLaunchArgument("with_dlio", default_value="true",
                              description="Launch D-LIO nodes inline. Set false when "
                                          "D-LIO is started in a separate tmux window."),
        DeclareLaunchArgument("enable_map_viz_layers", default_value="true",
                              description="Publish a visualization-only global costmap with unknown cells hidden."),
        DeclareLaunchArgument("record_results", default_value="false",
                              description="Record the goal navigation result topics to a rosbag."),
        DeclareLaunchArgument("record_bag_dir", default_value="/external/bags",
                              description="Directory where result rosbags are written."),
        DeclareLaunchArgument("record_bag_prefix", default_value="goal_nav_results",
                              description="Filename prefix for timestamped result rosbag directories."),
        DeclareLaunchArgument("record_storage", default_value="sqlite3",
                              description="rosbag2 storage plugin used for result recording."),
    ]

    # ---- 1. Static TFs (Go2W URDF extrinsics, taken from D-LIO dlio.yaml) --
    qz, qw = _yaw_quat_z_w(math.pi / 2.0)
    static_tf_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_imu_static_tf",
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0.0",
            "--qx", "0.0", "--qy", "0.0", "--qz", qz, "--qw", qw,
            "--frame-id", "base_link", "--child-frame-id", "imu_link",
        ],
    )
    static_tf_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_hesai_static_tf",
        arguments=[
            "--x", "0.1634", "--y", "0.0", "--z", "0.116",
            "--qx", "0.0", "--qy", "0.0", "--qz", qz, "--qw", qw,
            "--frame-id", "base_link", "--child-frame-id", "hesai_lidar",
        ],
    )

    # ---- 2. Robot model ----------------------------------------------------
    robot_description_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("nav_goal_go2w_bringup"),
            "launch", "robot_description.launch.py",
        ])),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # ---- 3. Hesai LiDAR ------------------------------------------------------
    hesai_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("hesai_lidar"), "launch", "hesai_lidar_launch.py",
        ])),
    )

    # ---- 4. IMU publisher ----------------------------------------------------
    imu_node = Node(
        package="go2w_imu_publisher",
        executable="imu_publisher",
        name="go2w_imu_publisher",
        output="screen",
    )

    # ---- 5. D-LIO --------------------------------------------------------------
    dlio_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("direct_lidar_inertial_odometry"), "launch", "dlio.launch.py",
        ])),
        condition=IfCondition(with_dlio),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "dlio_output": dlio_output,
            "dlio_log_dir": dlio_log_dir_override,
        }.items(),
    )

    # ---- 6. Map servers (grid + cloud) ----------------------------------------
    map_servers_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("nav_goal_go2w_map"),
            "launch", "map_servers.launch.py",
        ])),
        launch_arguments={
            "map": map_dir,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # ---- 7. Scan-to-map localizer ---------------------------------------------
    def _localization_actions(context, *args, **kwargs):
        loc_args = {
            "map": map_dir.perform(context),
            "use_sim_time": use_sim_time.perform(context),
            "registration_rate_hz": registration_rate.perform(context),
        }
        override = localization_params_file.perform(context).strip()
        if override:
            loc_args["localization_params_file"] = override
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare("nav_goal_go2w_localization"),
                    "launch", "localization.launch.py",
                ])),
                launch_arguments=loc_args.items(),
            )
        ]

    # ---- 8. pointcloud_to_laserscan -------------------------------------------
    p2l_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("nav_goal_go2w_bringup"),
            "launch", "pointcloud_to_laserscan.launch.py",
        ])),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # ---- 9. Nav2 stack + goal executor -----------------------------------------
    planner_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("nav_goal_go2w_planner"),
            "launch", "goal_planner.launch.py",
        ])),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "goal_update_strategy": goal_update_strategy,
            "require_localization": require_localization,
            "enable_map_viz_layers": enable_map_viz_layers,
        }.items(),
    )

    # ---- 10. Velocity bridge ----------------------------------------------------
    bridge_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("nav_goal_go2w_bridge"),
            "launch", "velocity_bridge.launch.py",
        ])),
        launch_arguments={
            "vx_max": vx_max,
            "vy_max": vy_max,
            "wz_max": wz_max,
            "dry_run": bridge_dry_run,
        }.items(),
    )

    # ---- 11. RViz (optional) -----------------------------------------------------
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(use_rviz),
        arguments=[
            "-d", PathJoinSubstitution([
                FindPackageShare("nav_goal_go2w_bringup"), "config", "goal_nav.rviz",
            ]),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    runtime_actions = [
        static_tf_imu,
        static_tf_lidar,
        robot_description_include,
        hesai_include,
        imu_node,
        dlio_include,
        map_servers_include,
        OpaqueFunction(function=_localization_actions),
        p2l_include,
        planner_include,
        bridge_include,
        rviz_node,
    ]

    def _runtime_actions(context, *args, **kwargs):
        if not _launch_bool(LaunchConfiguration("record_results").perform(context)):
            return runtime_actions
        # rosbag2 refuses an output directory that already exists. Give the
        # recorder time to create it before node logs create output_path/logs.
        return [
            TimerAction(
                period=2.0,
                actions=[
                    SetEnvironmentVariable("ROS_LOG_DIR", LaunchConfiguration("record_logs_dir")),
                    *runtime_actions,
                ],
            ),
        ]

    return LaunchDescription([
        *declared_args,
        OpaqueFunction(function=_validate_launch_args),
        OpaqueFunction(function=_result_recording_setup_actions),
        OpaqueFunction(function=_result_recording_bag_actions),
        OpaqueFunction(function=_runtime_actions),
    ])
