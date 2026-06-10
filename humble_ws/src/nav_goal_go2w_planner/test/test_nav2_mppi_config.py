from __future__ import annotations

from pathlib import Path

import pytest
import yaml


PLANNER_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PLANNER_ROOT.parent
BRIDGE_ROOT = SRC_ROOT / "nav_goal_go2w_bridge"


def _load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def test_nav2_uses_navfn_and_mppi_omni():
    config = _load_yaml(PLANNER_ROOT / "config" / "nav2_params.yaml")

    navigator = config["bt_navigator"]["ros__parameters"]["navigate_to_pose"]
    planner = config["planner_server"]["ros__parameters"]["GridBased"]
    controller = config["controller_server"]["ros__parameters"]["FollowPath"]
    smoother = config["smoother_server"]["ros__parameters"]["simple_smoother"]
    behaviors = config["behavior_server"]["ros__parameters"]

    assert navigator["plugin"] == "nav2_bt_navigator/NavigateToPoseNavigator"
    assert planner["plugin"] == "nav2_navfn_planner/NavfnPlanner"
    assert planner["allow_unknown"] is True
    assert controller["plugin"] == "nav2_mppi_controller::MPPIController"
    assert controller["motion_model"] == "Omni"
    assert "VelocityDeadbandCritic" in controller["critics"]
    assert smoother["plugin"] == "nav2_smoother::SimpleSmoother"
    assert behaviors["spin"]["plugin"] == "nav2_behaviors/Spin"
    assert behaviors["backup"]["plugin"] == "nav2_behaviors/BackUp"
    assert behaviors["drive_on_heading"]["plugin"] == "nav2_behaviors/DriveOnHeading"
    assert behaviors["wait"]["plugin"] == "nav2_behaviors/Wait"


def test_nav2_topics_match_dlio_and_hesai_scan_pipeline():
    config = _load_yaml(PLANNER_ROOT / "config" / "nav2_params.yaml")

    assert config["bt_navigator"]["ros__parameters"]["odom_topic"] == "/dlio/odom_node/odom"
    assert config["velocity_smoother"]["ros__parameters"]["odom_topic"] == "/dlio/odom_node/odom"

    local_costmap = config["local_costmap"]["local_costmap"]["ros__parameters"]
    global_costmap = config["global_costmap"]["global_costmap"]["ros__parameters"]

    assert local_costmap["global_frame"] == "odom"
    assert local_costmap["obstacle_layer"]["scan"]["topic"] == "/scan"
    assert global_costmap["global_frame"] == "map"
    assert global_costmap["static_layer"]["map_topic"] == "/map"
    assert global_costmap["static_layer"]["map_subscribe_transient_local"] is True
    assert global_costmap["track_unknown_space"] is True


def test_humble_costmap_dimension_parameter_types():
    config = _load_yaml(PLANNER_ROOT / "config" / "nav2_params.yaml")
    local_costmap = config["local_costmap"]["local_costmap"]["ros__parameters"]

    assert local_costmap["width"] == 6
    assert local_costmap["height"] == 6
    assert isinstance(local_costmap["width"], int)
    assert isinstance(local_costmap["height"], int)


def test_mppi_model_dt_matches_controller_period():
    config = _load_yaml(PLANNER_ROOT / "config" / "nav2_params.yaml")
    controller_server = config["controller_server"]["ros__parameters"]
    controller = controller_server["FollowPath"]

    controller_period = 1.0 / controller_server["controller_frequency"]
    assert controller["model_dt"] + 1.0e-9 >= controller_period


def test_costmaps_use_go2w_rectangular_footprint():
    config = _load_yaml(PLANNER_ROOT / "config" / "nav2_params.yaml")
    controller = config["controller_server"]["ros__parameters"]["FollowPath"]
    local_costmap = config["local_costmap"]["local_costmap"]["ros__parameters"]
    global_costmap = config["global_costmap"]["global_costmap"]["ros__parameters"]
    footprint = "[[0.35, 0.215], [0.35, -0.215], [-0.35, -0.215], [-0.35, 0.215]]"

    assert "robot_radius" not in local_costmap
    assert "robot_radius" not in global_costmap
    assert local_costmap["footprint"] == footprint
    assert global_costmap["footprint"] == footprint
    assert controller["CostCritic"]["consider_footprint"] is False


def test_nav2_velocity_ceiling_allows_bridge_defaults():
    nav2 = _load_yaml(PLANNER_ROOT / "config" / "nav2_params.yaml")
    bridge = _load_yaml(BRIDGE_ROOT / "config" / "velocity_bridge.yaml")

    controller = nav2["controller_server"]["ros__parameters"]["FollowPath"]
    smoother = nav2["velocity_smoother"]["ros__parameters"]
    bridge_params = bridge["velocity_bridge"]["ros__parameters"]

    assert controller["vx_max"] >= bridge_params["vx_max"]
    assert controller["vy_max"] >= bridge_params["vy_max"]
    assert controller["wz_max"] >= bridge_params["wz_max"]
    assert smoother["max_velocity"][0] >= bridge_params["vx_max"]
    assert smoother["max_velocity"][1] >= bridge_params["vy_max"]
    assert smoother["max_velocity"][2] >= bridge_params["wz_max"]
    assert controller["vx_max"] == pytest.approx(0.80)
    assert controller["vx_min"] == pytest.approx(-0.30)
    assert controller["vy_max"] == pytest.approx(0.60)
    assert controller["wz_max"] == pytest.approx(1.00)
    assert smoother["max_velocity"] == pytest.approx([0.80, 0.60, 1.00])
    assert smoother["min_velocity"] == pytest.approx([-0.30, -0.60, -1.00])


def test_nav2_launch_uses_dedicated_params_argument():
    nav2_launch = (PLANNER_ROOT / "launch" / "nav2_navigation.launch.py").read_text(
        encoding="utf-8",
    )
    planner_launch = (PLANNER_ROOT / "launch" / "goal_planner.launch.py").read_text(
        encoding="utf-8",
    )

    assert 'LaunchConfiguration("nav2_params_file")' in nav2_launch
    assert 'DeclareLaunchArgument("nav2_params_file"' in nav2_launch
    assert 'LaunchConfiguration("params_file")' not in nav2_launch
    assert 'DeclareLaunchArgument("params_file"' not in nav2_launch

    assert 'LaunchConfiguration("nav2_params_file")' in planner_launch
    assert '"nav2_params_file": nav2_params_file' in planner_launch
    assert 'LaunchConfiguration("params_file")' not in planner_launch
    assert 'DeclareLaunchArgument("params_file"' not in planner_launch


def test_goal_pose_executor_defaults():
    config = _load_yaml(PLANNER_ROOT / "config" / "goal_pose_executor.yaml")
    params = config["goal_pose_executor"]["ros__parameters"]
    composite_launch = (
        PLANNER_ROOT / "launch" / "goal_planner.launch.py"
    ).read_text(encoding="utf-8")
    executor_source = (
        PLANNER_ROOT / "nav_goal_go2w_planner" / "goal_pose_executor.py"
    ).read_text(encoding="utf-8")

    # Operator-facing defaults: RViz topic, preempt on new click, validation
    # against the served static map with unknown space allowed.
    assert params["goal_pose_topic"] == "/goal_pose"
    assert params["goal_update_strategy"] == "preempt"
    assert params["goal_validation_map_topic"] == "/map"
    assert params["treat_unknown_as_reachable"] is True
    assert params["require_localization"] is True
    assert "TRACKING" in params["dispatch_localization_states"]
    assert "LOST" in params["cancel_localization_states"]
    assert "goal_update_strategy" in composite_launch
    assert "require_localization" in composite_launch
    assert "validate_goal_reachable" in executor_source
    assert "treat_unknown_as_reachable" in executor_source


def test_goal_pose_executor_launch_does_not_remap_internal_navigator_name():
    composite_launch = (
        PLANNER_ROOT / "launch" / "goal_planner.launch.py"
    ).read_text(encoding="utf-8")
    executor_source = (
        PLANNER_ROOT / "nav_goal_go2w_planner" / "goal_pose_executor.py"
    ).read_text(encoding="utf-8")

    assert 'name="goal_pose_executor"' not in composite_launch
    assert 'super().__init__("goal_pose_executor")' in executor_source
    assert 'node_name="goal_pose_executor_navigator"' in executor_source
