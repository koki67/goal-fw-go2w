"""Scan-to-map localizer: registers live LiDAR scans against a pre-built PCD
map and publishes the map->odom TF correction on top of D-LIO odometry.

The default input is D-LIO's deskewed cloud, which is published in the odom
frame — registering it against the map-frame target with the current
map->odom as the initial guess yields the refined map->odom directly.

Registration runs on cloud arrival (throttled to registration_rate_hz in
wall time — a sim-time timer would stall whenever /clock lags wall time),
in its own callback group on a multi-threaded executor; a separate TF timer
(default 50 Hz) re-stamps the latest accepted map->odom, future-dated by
transform_tolerance so Nav2 lookups never extrapolate between registration
updates (same trick AMCL uses).

No TF is published before the first /initialpose: with map->odom missing,
Nav2 costmaps stay inactive, which is exactly the safe behavior we want.
"""
from __future__ import annotations

import threading
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    TransformStamped,
)
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from nav_goal_go2w_localization import cloud_utils
from nav_goal_go2w_localization.localization_core import (
    LocalizationStateMachine,
    LocalizerConfig,
    LocalizerState,
    make_transform,
    quaternion_to_rotation,
    rotation_to_quaternion,
)
from nav_goal_go2w_localization.registration import (
    MapTarget,
    RegistrationConfig,
)

try:  # pcd loading shared with the map package
    from nav_goal_go2w_map import pcd_io
except ImportError:  # pragma: no cover - map package is a hard runtime dep
    pcd_io = None


def transform_msg_to_matrix(transform) -> np.ndarray:
    rotation = quaternion_to_rotation(
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w,
    )
    translation = np.array(
        [transform.translation.x, transform.translation.y, transform.translation.z]
    )
    return make_transform(rotation, translation)


def pose_msg_to_matrix(pose) -> np.ndarray:
    rotation = quaternion_to_rotation(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    translation = np.array([pose.position.x, pose.position.y, pose.position.z])
    return make_transform(rotation, translation)


class LocalizerNode(Node):
    """small_gicp scan-to-map localization with health state machine."""

    def __init__(self) -> None:
        super().__init__("scan_to_map_localizer")

        self.declare_parameter("map_path", "")
        self.declare_parameter(
            "input_cloud_topic", "/dlio/odom_node/pointcloud/deskewed"
        )
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("registration_rate_hz", 2.0)
        self.declare_parameter("tf_publish_rate_hz", 50.0)
        self.declare_parameter("transform_tolerance", 0.3)
        self.declare_parameter("registration_type", "GICP")
        self.declare_parameter("num_threads", 4)
        self.declare_parameter("map_voxel_size", 0.20)
        self.declare_parameter("scan_voxel_size", 0.25)
        self.declare_parameter("min_range", 0.5)
        self.declare_parameter("max_range", 30.0)
        self.declare_parameter("max_correspondence_distance", 1.0)
        self.declare_parameter("converging_max_correspondence_distance", 2.0)
        self.declare_parameter("min_points", 200)
        self.declare_parameter("min_inlier_fraction", 0.6)
        self.declare_parameter("max_mean_error", 0.10)
        self.declare_parameter("max_translation_jump_m", 0.5)
        self.declare_parameter("max_rotation_jump_deg", 10.0)
        self.declare_parameter("jump_confirm_count", 3)
        self.declare_parameter("correction_blend", 1.0)
        self.declare_parameter("constrain_2d", True)
        self.declare_parameter("converge_good_count", 3)
        self.declare_parameter("degraded_after_rejects", 4)
        self.declare_parameter("lost_after_rejects", 12)

        map_path = str(self.get_parameter("map_path").value)
        if not map_path:
            raise ValueError("map_path parameter is required")
        if pcd_io is None:
            raise ImportError("nav_goal_go2w_map is required for PCD loading")

        self._map_frame = str(self.get_parameter("map_frame").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._min_range = float(self.get_parameter("min_range").value)
        self._max_range = float(self.get_parameter("max_range").value)
        self._tracking_distance = float(
            self.get_parameter("max_correspondence_distance").value
        )
        self._converging_distance = float(
            self.get_parameter("converging_max_correspondence_distance").value
        )
        self._min_points = int(self.get_parameter("min_points").value)
        self._transform_tolerance = float(
            self.get_parameter("transform_tolerance").value
        )

        map_points = pcd_io.load_xyz(map_path)
        self._map_target = MapTarget(
            map_points,
            RegistrationConfig(
                registration_type=str(
                    self.get_parameter("registration_type").value
                ),
                map_voxel_size=float(self.get_parameter("map_voxel_size").value),
                scan_voxel_size=float(self.get_parameter("scan_voxel_size").value),
                num_threads=int(self.get_parameter("num_threads").value),
            ),
        )
        self.get_logger().info(
            f"map loaded: {len(map_points)} points -> "
            f"{self._map_target.size} after preprocessing ({map_path})"
        )

        self._machine = LocalizationStateMachine(
            LocalizerConfig(
                min_points=self._min_points,
                min_inlier_fraction=float(
                    self.get_parameter("min_inlier_fraction").value
                ),
                max_mean_error=float(self.get_parameter("max_mean_error").value),
                max_translation_jump_m=float(
                    self.get_parameter("max_translation_jump_m").value
                ),
                max_rotation_jump_deg=float(
                    self.get_parameter("max_rotation_jump_deg").value
                ),
                jump_confirm_count=int(
                    self.get_parameter("jump_confirm_count").value
                ),
                correction_blend=float(
                    self.get_parameter("correction_blend").value
                ),
                constrain_2d=bool(self.get_parameter("constrain_2d").value),
                converge_good_count=int(
                    self.get_parameter("converge_good_count").value
                ),
                degraded_after_rejects=int(
                    self.get_parameter("degraded_after_rejects").value
                ),
                lost_after_rejects=int(
                    self.get_parameter("lost_after_rejects").value
                ),
            )
        )
        self._lock = threading.Lock()
        self._latest_cloud: PointCloud2 | None = None
        self._cloud_is_new = False
        self._last_registration_ms = 0.0
        self._last_outcome_info: dict[str, str] = {}
        self._cycles_since_accept = 0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)

        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._state_pub = self.create_publisher(
            String, "/localization/state", latched
        )
        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/localization/pose", 10
        )
        self._fitness_pub = self.create_publisher(
            Float32, "/localization/fitness", 10
        )
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # Registration is driven by cloud arrival, throttled in WALL time:
        # a sim-time timer stalls whenever /clock runs slower than wall time,
        # and on the robot the cloud rate is the natural cadence anyway. The
        # mutually-exclusive group keeps the heavy GICP work off the TF timer.
        registration_rate = max(
            float(self.get_parameter("registration_rate_hz").value), 0.1
        )
        self._registration_min_period = 1.0 / registration_rate
        self._last_registration_wall = float("-inf")
        self._cloud_sub = self.create_subscription(
            PointCloud2,
            str(self.get_parameter("input_cloud_topic").value),
            self._on_cloud,
            sensor_qos,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self._initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self._on_initialpose, 10
        )

        tf_rate = max(float(self.get_parameter("tf_publish_rate_hz").value), 1.0)
        self._tf_timer = self.create_timer(
            1.0 / tf_rate,
            self._on_tf_timer,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self._publish_state()
        self.get_logger().info(
            "localizer ready; waiting for /initialpose (RViz '2D Pose Estimate')"
        )

    # -- callbacks ----------------------------------------------------------

    def _on_cloud(self, msg: PointCloud2) -> None:
        with self._lock:
            self._latest_cloud = msg
            self._cloud_is_new = True
        now = time.monotonic()
        if now - self._last_registration_wall < self._registration_min_period:
            return
        self._last_registration_wall = now
        self._run_registration()

    def _on_initialpose(self, msg: PoseWithCovarianceStamped) -> None:
        if msg.header.frame_id and msg.header.frame_id != self._map_frame:
            self.get_logger().warn(
                f"/initialpose frame {msg.header.frame_id!r} != {self._map_frame!r}; ignoring"
            )
            return
        T_odom_base = self._lookup_matrix(self._odom_frame, self._base_frame)
        if T_odom_base is None:
            self.get_logger().error(
                "cannot accept /initialpose: no odom->base_link TF yet "
                "(is D-LIO running?)"
            )
            return
        T_map_base = pose_msg_to_matrix(msg.pose.pose)
        with self._lock:
            self._machine.set_initial_pose(T_map_base, T_odom_base)
        self._publish_state()
        position = msg.pose.pose.position
        self.get_logger().info(
            f"initial pose set ({position.x:.2f}, {position.y:.2f}); converging..."
        )

    def _on_tf_timer(self) -> None:
        with self._lock:
            transform = (
                None
                if self._machine.T_map_odom is None
                else self._machine.T_map_odom.copy()
            )
        if transform is None:
            return
        stamped = TransformStamped()
        stamped.header.stamp = (
            self.get_clock().now()
            + Duration(seconds=self._transform_tolerance)
        ).to_msg()
        stamped.header.frame_id = self._map_frame
        stamped.child_frame_id = self._odom_frame
        stamped.transform.translation.x = float(transform[0, 3])
        stamped.transform.translation.y = float(transform[1, 3])
        stamped.transform.translation.z = float(transform[2, 3])
        qx, qy, qz, qw = rotation_to_quaternion(transform[:3, :3])
        stamped.transform.rotation.x = qx
        stamped.transform.rotation.y = qy
        stamped.transform.rotation.z = qz
        stamped.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(stamped)

    def _run_registration(self) -> None:
        with self._lock:
            if self._machine.state == LocalizerState.UNINITIALIZED:
                return
            if not self._cloud_is_new or self._latest_cloud is None:
                return
            cloud_msg = self._latest_cloud
            self._cloud_is_new = False
            init_T = self._machine.T_map_odom.copy()
            converging = self._machine.state == LocalizerState.CONVERGING

        points = self._cloud_to_odom_points(cloud_msg)
        if points is None:
            return
        center = self._registration_center(cloud_msg, points)
        points = cloud_utils.crop_range(
            points, center, self._min_range, self._max_range
        )
        if len(points) < self._min_points:
            self._record_registration_rejection(
                f"too_few_points({len(points)})",
                source_points=len(points),
            )
            return

        distance = (
            self._converging_distance if converging else self._tracking_distance
        )
        started = time.monotonic()
        try:
            outcome = self._map_target.register(points, init_T, distance)
        except Exception as exc:
            elapsed_ms = (time.monotonic() - started) * 1000.0
            self.get_logger().error(
                f"registration failed: {exc}",
                throttle_duration_sec=5.0,
            )
            self._record_registration_rejection(
                f"registration_error({type(exc).__name__})",
                source_points=len(points),
                elapsed_ms=elapsed_ms,
            )
            return
        elapsed_ms = (time.monotonic() - started) * 1000.0

        with self._lock:
            result = self._machine.update(outcome)
            state = self._machine.state
            T_map_odom = (
                None
                if self._machine.T_map_odom is None
                else self._machine.T_map_odom.copy()
            )
            self._last_registration_ms = elapsed_ms
            self._cycles_since_accept = (
                0 if result.accepted else self._cycles_since_accept + 1
            )
            self._last_outcome_info = {
                "reason": result.reason,
                "inlier_fraction": f"{outcome.inlier_fraction:.3f}",
                "mean_error": f"{outcome.mean_error:.4f}",
                "source_points": str(outcome.source_size),
                "registration_ms": f"{elapsed_ms:.1f}",
            }

        self._fitness_pub.publish(Float32(data=float(outcome.mean_error)))
        if result.state_changed:
            self._publish_state()
            log = (
                self.get_logger().info
                if state == LocalizerState.TRACKING
                else self.get_logger().warn
            )
            log(f"localization state -> {state.value} ({result.reason})")
        if result.accepted and T_map_odom is not None:
            self._publish_pose(cloud_msg, T_map_odom, outcome.mean_error)
        self._publish_diagnostics(state)

    # -- helpers -------------------------------------------------------------

    def _record_registration_rejection(
        self,
        reason: str,
        source_points: int,
        elapsed_ms: float = 0.0,
    ) -> None:
        with self._lock:
            result = self._machine.reject(reason)
            state = self._machine.state
            self._last_registration_ms = elapsed_ms
            self._cycles_since_accept += 1
            self._last_outcome_info = {
                "reason": result.reason,
                "source_points": str(source_points),
                "registration_ms": f"{elapsed_ms:.1f}",
            }

        if result.state_changed:
            self._publish_state()
            self.get_logger().warn(
                f"localization state -> {state.value} ({result.reason})"
            )
        self._publish_diagnostics(state)

    def _cloud_to_odom_points(self, msg: PointCloud2) -> np.ndarray | None:
        points = cloud_utils.pointcloud2_to_xyz(msg)
        frame = msg.header.frame_id
        if frame == self._odom_frame:
            return points
        # Fallback path for clouds not already in odom (e.g. /points_raw).
        T_odom_cloud = self._lookup_matrix_stamped_or_latest(
            self._odom_frame, frame, msg.header.stamp
        )
        if T_odom_cloud is None:
            self.get_logger().warn(
                f"no TF {self._odom_frame} <- {frame}; dropping cloud",
                throttle_duration_sec=5.0,
            )
            return None
        rotated = points @ T_odom_cloud[:3, :3].T.astype(np.float32)
        return rotated + T_odom_cloud[:3, 3].astype(np.float32)

    def _registration_center(self, msg: PointCloud2, points: np.ndarray) -> np.ndarray:
        T_odom_base = self._lookup_matrix_stamped_or_latest(
            self._odom_frame, self._base_frame, msg.header.stamp
        )
        if T_odom_base is not None:
            return T_odom_base[:3, 3]
        return points.mean(axis=0) if len(points) else np.zeros(3)

    def _lookup_matrix_stamped_or_latest(
        self, target: str, source: str, stamp
    ) -> np.ndarray | None:
        """TF at the message stamp, falling back to latest.

        Explicit None checks: numpy arrays cannot be chained with `or`.
        """
        matrix = self._lookup_matrix(target, source, Time.from_msg(stamp))
        if matrix is not None:
            return matrix
        return self._lookup_matrix(target, source)

    def _lookup_matrix(
        self, target: str, source: str, when: Time | None = None
    ) -> np.ndarray | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                target,
                source,
                when if when is not None else Time(),
                timeout=Duration(seconds=0.05),
            )
        except Exception:
            return None
        return transform_msg_to_matrix(transform.transform)

    def _publish_state(self) -> None:
        self._state_pub.publish(String(data=self._machine.state.value))

    def _publish_pose(
        self, cloud_msg: PointCloud2, T_map_odom: np.ndarray, mean_error: float
    ) -> None:
        T_odom_base = self._lookup_matrix_stamped_or_latest(
            self._odom_frame, self._base_frame, cloud_msg.header.stamp
        )
        if T_odom_base is None:
            return
        T_map_base = T_map_odom @ T_odom_base
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = cloud_msg.header.stamp
        msg.header.frame_id = self._map_frame
        msg.pose.pose.position.x = float(T_map_base[0, 3])
        msg.pose.pose.position.y = float(T_map_base[1, 3])
        msg.pose.pose.position.z = float(T_map_base[2, 3])
        qx, qy, qz, qw = rotation_to_quaternion(T_map_base[:3, :3])
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        # Coarse covariance scaled from the registration residual: enough for
        # RViz inspection, not a calibrated uncertainty.
        variance = max(float(mean_error), 1.0e-4)
        for index in (0, 7, 14):
            msg.pose.covariance[index] = variance
        msg.pose.covariance[35] = variance
        self._pose_pub.publish(msg)

    def _publish_diagnostics(self, state: LocalizerState) -> None:
        status = DiagnosticStatus()
        status.name = "goal_nav/localizer"
        status.hardware_id = "scan_to_map_localizer"
        if state == LocalizerState.TRACKING:
            status.level = DiagnosticStatus.OK
        elif state in (LocalizerState.CONVERGING, LocalizerState.DEGRADED):
            status.level = DiagnosticStatus.WARN
        else:
            status.level = DiagnosticStatus.ERROR
        status.message = state.value
        values = dict(self._last_outcome_info)
        values["cycles_since_accept"] = str(self._cycles_since_accept)
        status.values = [
            KeyValue(key=key, value=value) for key, value in values.items()
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diag_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizerNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
