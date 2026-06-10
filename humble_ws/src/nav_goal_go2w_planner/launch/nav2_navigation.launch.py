"""Launch the Nav2 stack (NavFn + MPPI Omni) with our parameter file."""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _deep_merge(base, overlay):
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _deep_merge(base[key], value)
        else:
            base[key] = value
    return base


def _set_nested(data, dotted_key, value):
    cursor = data
    parts = dotted_key.split(".")
    for part in parts[:-1]:
        cursor = cursor.setdefault(part, {})
    cursor[parts[-1]] = value


def _load_yaml_file(path):
    if not path:
        return {}
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def _nav2_nodes(context, *args, **kwargs):
    params_path = LaunchConfiguration("nav2_params_file").perform(context)
    extra_path = LaunchConfiguration("nav2_extra_params_file").perform(context).strip()
    global_map_topic = LaunchConfiguration("global_map_topic").perform(context)
    global_static_layer_name = LaunchConfiguration("global_static_layer_name").perform(context).strip()
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")
    autostart = LaunchConfiguration("autostart")

    params = _load_yaml_file(params_path)
    if extra_path:
        params = _deep_merge(params, _load_yaml_file(extra_path))
    # The caller identifies the global StaticLayer whose map topic is overridden.
    layer_key = global_static_layer_name or "static_layer"
    _set_nested(
        params,
        f"global_costmap.global_costmap.ros__parameters.{layer_key}.map_topic",
        global_map_topic,
    )

    merged = tempfile.NamedTemporaryFile(
        mode="w",
        prefix="nav2_params_",
        suffix=".yaml",
        delete=False,
        encoding="utf-8",
    )
    with merged:
        yaml.safe_dump(params, merged, sort_keys=False)
    configured_params = merged.name

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "smoother_server",
        "velocity_smoother",
    ]

    return [
        Node(
            package="nav2_controller", executable="controller_server",
            name="controller_server", output="screen",
            parameters=[configured_params, {"use_sim_time": use_sim_time}],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_planner", executable="planner_server",
            name="planner_server", output="screen",
            parameters=[configured_params, {"use_sim_time": use_sim_time}],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=remappings,
        ),
        Node(
            package="nav2_behaviors", executable="behavior_server",
            name="behavior_server", output="screen",
            parameters=[configured_params, {"use_sim_time": use_sim_time}],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_bt_navigator", executable="bt_navigator",
            name="bt_navigator", output="screen",
            parameters=[configured_params, {"use_sim_time": use_sim_time}],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=remappings,
        ),
        Node(
            package="nav2_smoother", executable="smoother_server",
            name="smoother_server", output="screen",
            parameters=[configured_params, {"use_sim_time": use_sim_time}],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=remappings,
        ),
        Node(
            package="nav2_velocity_smoother", executable="velocity_smoother",
            name="velocity_smoother", output="screen",
            parameters=[configured_params, {"use_sim_time": use_sim_time}],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=remappings + [
                ("cmd_vel", "cmd_vel_nav"),
                ("cmd_vel_smoothed", "cmd_vel"),
            ],
        ),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_navigation", output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"autostart": autostart},
                {"node_names": lifecycle_nodes},
            ],
            arguments=["--ros-args", "--log-level", log_level],
        ),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("nav_goal_go2w_planner")
    default_params = os.path.join(pkg_share, "config", "nav2_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("nav2_params_file", default_value=default_params),
        DeclareLaunchArgument("nav2_extra_params_file", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("global_map_topic", default_value="/map"),
        DeclareLaunchArgument(
            "global_static_layer_name",
            default_value="static_layer",
            description="Plugin name of the global obstacle StaticLayer.",
        ),
        OpaqueFunction(function=_nav2_nodes),
    ])
