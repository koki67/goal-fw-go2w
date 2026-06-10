"""ROS wrapper for the desktop 2D simulator."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Optional

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header, String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from builtin_interfaces.msg import Duration as DurationMsg

from nav_goal_go2w_sim.dynamic_obstacles import (
    DynamicObstacle,
    spawn_random_obstacles,
    stamp_obstacles,
    step_dynamic_obstacles,
)
from nav_goal_go2w_sim.sim_core import (
    Pose2D,
    Twist2D,
    World2D,
    apply_odom_drift,
    check_collision,
    clamp_twist,
    integrate_omni,
    raycast_pointcloud,
    raycast_scan,
    realized_twist,
    world_points_to_drifted_odom,
)
from nav_goal_go2w_sim.world_loader import load_world


class SimNode(Node):
    """Closed-loop 2D simulator publishing scan, odometry, TF, and metrics."""

    def __init__(self) -> None:
        super().__init__("sim_node")

        self._declare_parameters()
        self._world = load_world(self._resolve_world_file(self._param_str("world_file")))
        self._base_grid = self._world.grid.copy()
        self._pose = self._initial_pose()
        self._odom_pose = self._pose
        self._previous_pose = self._pose
        self._current_cmd = Twist2D(0.0, 0.0, 0.0)
        self._last_cmd_wall_time = self._wall_time_seconds()
        self._last_realized_twist = Twist2D(0.0, 0.0, 0.0)
        self._sim_time = 0.0
        self._paused = False
        self._distance_traveled = 0.0
        self._collisions = 0
        self._seen_free_cells: set[tuple[int, int]] = set()
        self._path_msg = NavPath()
        self._last_path_pose = self._pose

        robot_length = self._param_float("robot_length")
        robot_width = self._param_float("robot_width")
        self._robot_footprint = [
            (0.5 * robot_length, 0.5 * robot_width),
            (0.5 * robot_length, -0.5 * robot_width),
            (-0.5 * robot_length, -0.5 * robot_width),
            (-0.5 * robot_length, 0.5 * robot_width),
        ]
        self._vx_max = self._param_float("vx_max")
        self._vy_max = self._param_float("vy_max")
        self._wz_max = self._param_float("wz_max")
        self._cmd_timeout = self._param_float("cmd_timeout")
        self._odom_drift_x_per_m = self._param_float("odom_drift_x_per_m")
        self._odom_drift_yaw_per_m = self._param_float("odom_drift_yaw_per_m")
        self._odom_frame = self._param_str("odom_frame")
        self._base_frame = self._param_str("base_frame")
        self._laser_frame = self._param_str("laser_frame")
        self._world_frame = self._param_str("world_frame")
        self._angle_min = self._param_float("angle_min")
        self._angle_max = self._param_float("angle_max")
        self._angle_increment = self._param_float("angle_increment")
        self._range_min = self._param_float("range_min")
        self._range_max = self._param_float("range_max")
        self._lidar_range_max = self._param_float("pointcloud_window_radius_m")
        self._lidar_num_rings = int(self.get_parameter("lidar_num_rings").value)
        self._lidar_vfov_deg = self._param_float("lidar_vfov_deg")
        self._lidar_az_step_deg = self._param_float("lidar_az_step_deg")
        self._lidar_sensor_height_m = self._param_float("lidar_sensor_height_m")
        self._publish_clock = self._param_bool("publish_clock")
        self._path_min_distance = self._param_float("path_min_distance")
        self._path_max_poses = max(int(self._param_float("path_max_poses")), 1)

        self._tf_pub = TransformBroadcaster(self)
        self._static_tf_pub = StaticTransformBroadcaster(self)
        self._publish_static_tf()

        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE

        self._cmd_sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self._on_cmd_vel,
            10,
        )
        self._pause_sub = self.create_subscription(
            Bool,
            "/sim/pause",
            self._on_pause,
            10,
        )
        self._scan_pub = self.create_publisher(
            LaserScan,
            "/scan",
            qos_profile_sensor_data,
        )
        self._pointcloud_pub = self.create_publisher(
            PointCloud2,
            "/dlio/odom_node/pointcloud/deskewed",
            qos_profile_sensor_data,
        )
        self._odom_pub = self.create_publisher(
            Odometry,
            "/dlio/odom_node/odom",
            10,
        )
        self._path_pub = self.create_publisher(NavPath, "/sim/path", 10)
        self._clock_pub = self.create_publisher(Clock, "/clock", 1)
        self._world_pub = self.create_publisher(OccupancyGrid, "/sim/world", map_qos)
        self._metrics_pub = self.create_publisher(String, "/sim/metrics", 10)
        self._paused_pub = self.create_publisher(Bool, "/sim/paused", 10)

        physics_rate = max(self._param_float("physics_rate_hz"), 1.0)
        scan_rate = max(self._param_float("scan_rate_hz"), 1.0)
        clock_rate = max(self._param_float("clock_rate_hz"), 1.0)
        self._physics_dt = 1.0 / physics_rate
        self.create_timer(self._physics_dt, self._on_physics_timer)
        self.create_timer(1.0 / scan_rate, self._on_scan_timer)
        self.create_timer(1.0, self._on_metrics_timer)
        if self._publish_clock:
            self.create_timer(1.0 / clock_rate, self._on_clock_timer)

        self._dynamic_obstacles: list[DynamicObstacle] = []
        self._dynamic_obstacle_radius = self._param_float("dynamic_obstacle_radius")
        self._dynamic_marker_lifetime_sec = 1
        self._dyn_marker_pub = self.create_publisher(
            MarkerArray,
            "/sim/dynamic_obstacles",
            10,
        )
        num_dynamic = self._param_int("num_dynamic_obstacles")
        if num_dynamic > 0:
            requested = num_dynamic
            self._dynamic_obstacles = spawn_random_obstacles(
                self._world,
                requested,
                radius=self._dynamic_obstacle_radius,
                speed=self._param_float("dynamic_obstacle_speed"),
                seed=self._param_int("dynamic_obstacles_seed"),
                exclusion_pose=self._pose,
                exclusion_radius=self._param_float("dynamic_obstacle_min_spawn_distance"),
            )
            placed = len(self._dynamic_obstacles)
            if placed < requested:
                self.get_logger().warning(
                    "Placed %d/%d dynamic obstacles; not enough free space."
                    % (placed, requested)
                )
            stamp_obstacles(
                self._world.grid,
                self._base_grid,
                self._dynamic_obstacles,
                resolution=self._world.resolution,
                origin=self._world.origin,
            )

        self._path_msg.header.frame_id = self._odom_frame
        self._append_path_pose(self._stamp(), force=True)
        self._publish_world()
        self._publish_pause_state()
        self.get_logger().info(
            (
                "2D sim ready: world=%s size=%dx%d res=%.3f "
                "spawn=(%.2f, %.2f, %.2f)"
            )
            % (
                self._world.name,
                self._world.width,
                self._world.height,
                self._world.resolution,
                self._pose.x,
                self._pose.y,
                self._pose.yaw,
            )
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("world_file", "open_room")
        self.declare_parameter("robot_length", 0.70)
        self.declare_parameter("robot_width", 0.43)
        self.declare_parameter("vx_max", 0.30)
        self.declare_parameter("vy_max", 0.20)
        self.declare_parameter("wz_max", 0.50)
        self.declare_parameter("start_x", 0.0)
        self.declare_parameter("start_y", 0.0)
        self.declare_parameter("start_yaw", 0.0)
        self.declare_parameter("use_world_spawn", True)
        self.declare_parameter("angle_min", -math.pi)
        self.declare_parameter("angle_max", math.pi)
        self.declare_parameter("angle_increment", 0.0087)
        self.declare_parameter("range_min", 0.30)
        self.declare_parameter("range_max", 30.0)
        self.declare_parameter("pointcloud_window_radius_m", 8.0)
        self.declare_parameter("lidar_num_rings", 16)
        self.declare_parameter("lidar_vfov_deg", 30.0)
        self.declare_parameter("lidar_az_step_deg", 0.5)
        self.declare_parameter("lidar_sensor_height_m", 0.3)
        self.declare_parameter("scan_rate_hz", 10.0)
        self.declare_parameter("physics_rate_hz", 50.0)
        self.declare_parameter("clock_rate_hz", 200.0)
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("publish_clock", True)
        self.declare_parameter("path_min_distance", 0.05)
        self.declare_parameter("path_max_poses", 5000)
        self.declare_parameter("odom_drift_x_per_m", 0.0)
        self.declare_parameter("odom_drift_yaw_per_m", 0.0)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("laser_frame", "laser_frame")
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("num_dynamic_obstacles", 0)
        self.declare_parameter("dynamic_obstacle_radius", 0.20)
        self.declare_parameter("dynamic_obstacle_speed", 0.30)
        self.declare_parameter("dynamic_obstacles_seed", 0)
        self.declare_parameter("dynamic_obstacle_min_spawn_distance", 1.5)

    def _initial_pose(self) -> Pose2D:
        if self._param_bool("use_world_spawn"):
            return self._world.spawn
        return Pose2D(
            self._param_float("start_x"),
            self._param_float("start_y"),
            self._param_float("start_yaw"),
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._current_cmd = clamp_twist(
            Twist2D(msg.linear.x, msg.linear.y, msg.angular.z),
            vx_max=self._vx_max,
            vy_max=self._vy_max,
            wz_max=self._wz_max,
        )
        self._last_cmd_wall_time = self._wall_time_seconds()

    def _on_pause(self, msg: Bool) -> None:
        if self._paused == msg.data:
            return
        self._paused = msg.data
        state = "paused" if self._paused else "running"
        self.get_logger().info(f"Simulator {state}.")
        self._publish_pause_state()

    def _on_physics_timer(self) -> None:
        if self._paused:
            return

        self._sim_time += self._physics_dt
        if self._dynamic_obstacles:
            step_dynamic_obstacles(
                self._dynamic_obstacles,
                self._base_grid,
                self._world,
                self._physics_dt,
            )
            stamp_obstacles(
                self._world.grid,
                self._base_grid,
                self._dynamic_obstacles,
                resolution=self._world.resolution,
                origin=self._world.origin,
            )
        cmd = self._active_cmd()
        candidate = integrate_omni(self._pose, cmd, self._physics_dt)

        if check_collision(self._world, candidate, self._robot_footprint):
            self._collisions += 1
            new_pose = self._pose
        else:
            new_pose = candidate
            self._distance_traveled += math.hypot(
                new_pose.x - self._pose.x,
                new_pose.y - self._pose.y,
            )

        self._previous_pose = self._pose
        self._pose = new_pose
        self._last_realized_twist = realized_twist(
            self._previous_pose,
            self._pose,
            self._physics_dt,
        )
        # Odometry integrates the same motion corrupted by drift; the
        # localizer's job is to keep map->odom correcting this error.
        self._odom_pose = integrate_omni(
            self._odom_pose,
            apply_odom_drift(
                self._last_realized_twist,
                self._odom_drift_x_per_m,
                self._odom_drift_yaw_per_m,
            ),
            self._physics_dt,
        )
        stamp = self._stamp()
        self._publish_odom(stamp)
        self._publish_odom_tf(stamp)
        self._publish_path(stamp)

    def _on_scan_timer(self) -> None:
        if self._paused:
            return

        result = raycast_scan(
            self._world,
            self._pose,
            angle_min=self._angle_min,
            angle_max=self._angle_max,
            angle_inc=self._angle_increment,
            range_min=self._range_min,
            range_max=self._range_max,
        )
        self._seen_free_cells.update(result.seen_free_cells)

        msg = LaserScan()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = self._laser_frame
        msg.angle_min = self._angle_min
        msg.angle_max = self._angle_max
        msg.angle_increment = self._angle_increment
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / max(self._param_float("scan_rate_hz"), 1.0)
        msg.range_min = self._range_min
        msg.range_max = self._range_max
        msg.ranges = result.ranges.tolist()
        self._scan_pub.publish(msg)
        self._publish_pointcloud(msg.header.stamp)

        if self._dynamic_obstacles:
            self._publish_dynamic_markers(msg.header.stamp)

    def _on_clock_timer(self) -> None:
        msg = Clock()
        msg.clock = self._stamp()
        self._clock_pub.publish(msg)

    def _on_metrics_timer(self) -> None:
        if self._paused:
            return

        self._publish_world()
        free_total = int((self._base_grid == 0).sum())
        explored_ratio = 0.0 if free_total == 0 else len(self._seen_free_cells) / free_total
        payload = {
            "sim_time": self._sim_time,
            "explored_ratio": min(explored_ratio, 1.0),
            "distance_traveled": self._distance_traveled,
            "collisions": self._collisions,
            "pose_xyyaw": [self._pose.x, self._pose.y, self._pose.yaw],
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self._metrics_pub.publish(msg)

    def _active_cmd(self) -> Twist2D:
        if self._wall_time_seconds() - self._last_cmd_wall_time > self._cmd_timeout:
            return Twist2D(0.0, 0.0, 0.0)
        return self._current_cmd

    def _publish_odom(self, stamp) -> None:
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_frame
        msg.pose.pose.position.x = self._odom_pose.x
        msg.pose.pose.position.y = self._odom_pose.y
        msg.pose.pose.orientation.z = math.sin(self._odom_pose.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self._odom_pose.yaw / 2.0)
        msg.twist.twist.linear.x = self._last_realized_twist.vx
        msg.twist.twist.linear.y = self._last_realized_twist.vy
        msg.twist.twist.angular.z = self._last_realized_twist.wz
        self._odom_pub.publish(msg)

    def _publish_path(self, stamp) -> None:
        distance = math.hypot(
            self._pose.x - self._last_path_pose.x,
            self._pose.y - self._last_path_pose.y,
        )
        if distance < self._path_min_distance:
            return
        self._append_path_pose(stamp)

    def _append_path_pose(self, stamp, *, force: bool = False) -> None:
        if not force and self._path_min_distance > 0.0:
            distance = math.hypot(
                self._pose.x - self._last_path_pose.x,
                self._pose.y - self._last_path_pose.y,
            )
            if distance < self._path_min_distance:
                return

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self._odom_frame
        pose.pose.position.x = self._odom_pose.x
        pose.pose.position.y = self._odom_pose.y
        pose.pose.orientation.z = math.sin(self._odom_pose.yaw / 2.0)
        pose.pose.orientation.w = math.cos(self._odom_pose.yaw / 2.0)

        self._path_msg.header.stamp = stamp
        self._path_msg.header.frame_id = self._odom_frame
        self._path_msg.poses.append(pose)
        if len(self._path_msg.poses) > self._path_max_poses:
            self._path_msg.poses = self._path_msg.poses[-self._path_max_poses :]
        self._last_path_pose = self._pose
        self._path_pub.publish(self._path_msg)

    def _publish_odom_tf(self, stamp) -> None:
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self._odom_frame
        transform.child_frame_id = self._base_frame
        transform.transform.translation.x = self._odom_pose.x
        transform.transform.translation.y = self._odom_pose.y
        transform.transform.rotation.z = math.sin(self._odom_pose.yaw / 2.0)
        transform.transform.rotation.w = math.cos(self._odom_pose.yaw / 2.0)
        self._tf_pub.sendTransform(transform)

    def _publish_static_tf(self) -> None:
        transform = TransformStamped()
        transform.header.stamp = self._stamp()
        transform.header.frame_id = self._base_frame
        transform.child_frame_id = self._laser_frame
        transform.transform.rotation.w = 1.0
        self._static_tf_pub.sendTransform(transform)

    def _publish_world(self) -> None:
        msg = OccupancyGrid()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = self._world_frame
        msg.info.resolution = self._world.resolution
        msg.info.width = self._world.width
        msg.info.height = self._world.height
        msg.info.origin.position.x = self._world.origin[0]
        msg.info.origin.position.y = self._world.origin[1]
        msg.info.origin.orientation.w = 1.0
        msg.data = self._base_grid.astype(np.int8).reshape(-1).tolist()
        self._world_pub.publish(msg)

    def _publish_pointcloud(self, stamp) -> None:
        points = raycast_pointcloud(
            self._world,
            self._pose,
            {
                "range_max": self._lidar_range_max,
                "num_rings": self._lidar_num_rings,
                "vfov_deg": self._lidar_vfov_deg,
                "az_step_deg": self._lidar_az_step_deg,
                "sensor_height_m": self._lidar_sensor_height_m,
            },
        )
        points = world_points_to_drifted_odom(points, self._pose, self._odom_pose)
        msg = point_cloud2.create_cloud_xyz32(
            header=_header(stamp, self._odom_frame),
            points=points.tolist(),
        )
        self._pointcloud_pub.publish(msg)

    def _publish_dynamic_markers(self, stamp) -> None:
        array = MarkerArray()
        for index, ob in enumerate(self._dynamic_obstacles):
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = self._odom_frame
            marker.ns = "dynamic_obstacles"
            marker.id = index
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = float(ob.x)
            marker.pose.position.y = float(ob.y)
            marker.pose.position.z = 0.15
            marker.pose.orientation.w = 1.0
            marker.scale.x = float(2.0 * ob.radius)
            marker.scale.y = float(2.0 * ob.radius)
            marker.scale.z = 0.30
            marker.color.r = 1.0
            marker.color.g = 0.4
            marker.color.b = 0.1
            marker.color.a = 0.85
            marker.lifetime = DurationMsg(sec=self._dynamic_marker_lifetime_sec, nanosec=0)
            array.markers.append(marker)
        self._dyn_marker_pub.publish(array)

    def _publish_pause_state(self) -> None:
        msg = Bool()
        msg.data = self._paused
        self._paused_pub.publish(msg)

    def _resolve_world_file(self, value: str) -> Path:
        candidate = Path(value).expanduser()
        if candidate.is_file():
            return candidate
        if candidate.suffix != ".yaml":
            candidate_name = f"{value}.yaml"
        else:
            candidate_name = value
        share = Path(get_package_share_directory("nav_goal_go2w_sim"))
        resolved = share / "worlds" / candidate_name
        if not resolved.is_file():
            raise FileNotFoundError(f"world file not found: {value}")
        return resolved

    def _stamp(self):
        return Time(seconds=self._sim_time).to_msg()

    def _wall_time_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

    def _param_float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _param_bool(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "on"}
        return bool(value)

    def _param_str(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _param_int(self, name: str) -> int:
        return int(self.get_parameter(name).value)


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[SimNode] = None
    try:
        node = SimNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _header(stamp, frame_id: str) -> Header:
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id
    return header
