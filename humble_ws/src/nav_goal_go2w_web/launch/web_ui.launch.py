"""Launch rosbridge, rosapi, static assets, and optional preparation relay."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _validate(context, *args, **kwargs):
    if LaunchConfiguration("prep_mode").perform(context).lower() in ("1", "true", "yes", "on"):
        output = LaunchConfiguration("prep_output").perform(context).strip()
        if not output or not Path(output).is_absolute():
            raise RuntimeError("prep_output must be a non-empty absolute path when prep_mode:=true")
    return []


def generate_launch_description():
    args = [
        DeclareLaunchArgument("http_port", default_value="8080"),
        DeclareLaunchArgument("rosbridge_port", default_value="9090"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("prep_mode", default_value="false"),
        DeclareLaunchArgument("prep_output", default_value=""),
        DeclareLaunchArgument("save_leaf_size", default_value="0.05"),
        DeclareLaunchArgument("prep_cloud_topic", default_value="/dlio/map_node/map"),
    ]
    www = str(Path(get_package_share_directory("nav_goal_go2w_web")) / "www")
    return LaunchDescription([*args, OpaqueFunction(function=_validate),
        Node(package="rosbridge_server", executable="rosbridge_websocket", name="rosbridge_websocket",
             parameters=[{"port": ParameterValue(LaunchConfiguration("rosbridge_port"), value_type=int)}]),
        Node(package="rosapi", executable="rosapi_node", name="rosapi"),
        ExecuteProcess(cmd=["python3", "-m", "http.server", LaunchConfiguration("http_port"),
                            "--bind", "0.0.0.0", "--directory", www], output="screen"),
        Node(package="nav_goal_go2w_web", executable="prep_web_node", name="prep_web_node",
             condition=IfCondition(LaunchConfiguration("prep_mode")), output="screen",
             parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time"),
                          "output": LaunchConfiguration("prep_output"),
                          "save_leaf_size": ParameterValue(LaunchConfiguration("save_leaf_size"), value_type=float),
                          "cloud_topic": LaunchConfiguration("prep_cloud_topic")}]),
    ])
