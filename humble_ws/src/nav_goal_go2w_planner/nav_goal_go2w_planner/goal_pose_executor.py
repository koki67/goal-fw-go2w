r"""Subscribe to /goal_pose (RViz "2D Nav Goal") and dispatch to Nav2.

State machine:
  IDLE  --(/goal_pose)-->  ACTIVE  --(SUCCEEDED|FAILED|CANCELED)-->  IDLE
                   \-(/goal_pose during ACTIVE)-> preempt (default) or queue

Deltas from the frontier executor this is derived from:
  * goals come from the operator, so `preempt` is the default strategy and
    the duplicate-suppression radius is small (only true double-clicks)
  * dispatch is gated on localization health (/localization/state); an
    active goal is canceled if the localizer reports LOST
  * unknown map cells count as reachable for goal validation by default —
    the pre-built map is allowed to be incomplete and NavFn plans with
    allow_unknown
  * operator feedback on /goal_executor/status (latched String) and
    /goal_markers (MarkerArray)
"""
from __future__ import annotations

import math
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from nav_goal_go2w_planner.goal_policy import (
    Action,
    Decision,
    GoalPolicy,
    GoalPose,
    Outcome,
    frames_match,
    should_cancel_for_timeout,
)
from nav_goal_go2w_planner.goal_validation import (
    Pose2D,
    ValidationGrid,
    validate_goal_reachable,
)


class GoalPoseExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__("goal_pose_executor")

        self.declare_parameter("goal_pose_topic", "/goal_pose")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("min_goal_update_distance", 0.1)
        self.declare_parameter("goal_update_strategy", "preempt")
        self.declare_parameter("goal_timeout_sec", 300.0)
        self.declare_parameter("result_check_rate", 2.0)
        self.declare_parameter("tf_timeout", 0.2)
        self.declare_parameter("goal_validation_map_topic", "/map")
        self.declare_parameter("goal_validation_reachable_cost_threshold", 0)
        self.declare_parameter("goal_validation_seed_search_radius", 3)
        self.declare_parameter("goal_validation_connectivity", 8)
        self.declare_parameter("treat_unknown_as_reachable", True)
        self.declare_parameter("require_localization", True)
        self.declare_parameter("localization_state_topic", "/localization/state")
        self.declare_parameter(
            "dispatch_localization_states", ["TRACKING", "DEGRADED"]
        )
        self.declare_parameter("cancel_localization_states", ["LOST"])

        self._goal_topic = self.get_parameter("goal_pose_topic").value
        self._global_frame = self.get_parameter("global_frame").value
        self._robot_base_frame = self.get_parameter("robot_base_frame").value
        self._min_update = max(0.0, float(self.get_parameter("min_goal_update_distance").value))
        self._goal_update_strategy = str(self.get_parameter("goal_update_strategy").value)
        self._goal_timeout = max(0.0, float(self.get_parameter("goal_timeout_sec").value))
        result_rate = max(0.5, float(self.get_parameter("result_check_rate").value))
        self._tf_timeout = float(self.get_parameter("tf_timeout").value)
        self._goal_validation_map_topic = str(self.get_parameter("goal_validation_map_topic").value).strip()
        self._goal_validation_reachable_cost_threshold = max(
            0, int(self.get_parameter("goal_validation_reachable_cost_threshold").value),
        )
        self._goal_validation_seed_search_radius = max(
            0, int(self.get_parameter("goal_validation_seed_search_radius").value),
        )
        self._goal_validation_connectivity = int(self.get_parameter("goal_validation_connectivity").value)
        self._treat_unknown_as_reachable = bool(
            self.get_parameter("treat_unknown_as_reachable").value
        )
        self._require_localization = bool(
            self.get_parameter("require_localization").value
        )
        self._dispatch_states = {
            str(state)
            for state in self.get_parameter("dispatch_localization_states").value
        }
        self._cancel_states = {
            str(state)
            for state in self.get_parameter("cancel_localization_states").value
        }

        self._policy = GoalPolicy(
            min_update_distance=self._min_update,
            update_strategy=self._goal_update_strategy,
        )
        self._throttle: dict[str, float] = {}

        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._navigator_lock = threading.RLock()
        self._navigator = BasicNavigator(node_name="goal_pose_executor_navigator")
        self._active_started_at: Optional[float] = None
        self._timeout_requested = False
        self._preempt_requested = False
        self._validation_cancel_requested = False
        self._validation_cancel_reason = ""
        self._localization_cancel_requested = False
        self._localization_state = "UNINITIALIZED"
        self._last_result_text = "no goals yet"
        self._last_failed_goal: Optional[GoalPose] = None
        self._latest_validation_map: Optional[OccupancyGrid] = None
        self._validation_map_sub = None
        if self._goal_validation_map_topic:
            validation_qos = QoSProfile(depth=1)
            validation_qos.reliability = ReliabilityPolicy.RELIABLE
            validation_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self._validation_map_sub = self.create_subscription(
                OccupancyGrid,
                self._goal_validation_map_topic,
                self._on_validation_map,
                validation_qos,
            )

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._status_pub = self.create_publisher(
            String, "/goal_executor/status", latched
        )
        self._markers_pub = self.create_publisher(MarkerArray, "/goal_markers", 1)
        self._localization_sub = self.create_subscription(
            String,
            str(self.get_parameter("localization_state_topic").value),
            self._on_localization_state,
            latched,
        )

        self._goal_sub = self.create_subscription(
            PoseStamped, self._goal_topic, self._on_goal, 10,
        )
        self._timer = self.create_timer(1.0 / result_rate, self._on_timer)

        self._publish_status("IDLE (waiting for goals)")
        self.get_logger().info(
            "Goal executor ready: topic=%s frame=%s strategy=%s min_update=%.2f "
            "timeout=%.0fs validation_map=%s require_localization=%s"
            % (
                self._goal_topic, self._global_frame,
                self._policy.update_strategy.value, self._min_update,
                self._goal_timeout, self._goal_validation_map_topic or "disabled",
                self._require_localization,
            )
        )

    def destroy_node(self) -> bool:
        try:
            with self._navigator_lock:
                self._navigator.destroy_node()
        except Exception:
            pass
        return super().destroy_node()

    # --- Inputs --------------------------------------------------------------

    def _on_validation_map(self, msg: OccupancyGrid) -> None:
        self._latest_validation_map = msg

    def _on_localization_state(self, msg: String) -> None:
        previous = self._localization_state
        self._localization_state = msg.data
        if previous != msg.data:
            self.get_logger().info("localization state: %s" % msg.data)

    def _on_goal(self, msg: PoseStamped) -> None:
        frame_id = msg.header.frame_id.strip()
        if not frames_match(frame_id, self._global_frame):
            self._throttled(
                "wrong_frame",
                "Rejecting goal in frame '%s' (expected '%s')." % (frame_id or "<empty>", self._global_frame),
                5.0,
            )
            return
        if self._require_localization and not self._localization_ok():
            self.get_logger().warning(
                "Rejecting goal: localization state is %s (need one of %s). "
                "Set the initial pose in RViz first."
                % (self._localization_state, sorted(self._dispatch_states))
            )
            self._publish_status(
                "REJECTED (localization %s)" % self._localization_state
            )
            return
        candidate = GoalPose(
            frame_id=frame_id,
            x=msg.pose.position.x,
            y=msg.pose.position.y,
            qz=msg.pose.orientation.z,
            qw=msg.pose.orientation.w,
        )
        decision = self._policy.offer(candidate, can_dispatch=self._can_dispatch())
        self._apply(decision)

    def _on_timer(self) -> None:
        if self._policy.has_active():
            self._maybe_cancel_for_localization()
            if not self._localization_cancel_requested:
                self._maybe_cancel_if_goal_invalid()
            if not (self._validation_cancel_requested or self._localization_cancel_requested):
                self._maybe_cancel_for_timeout()
            self._maybe_collect_result()
            return
        decision = self._policy.maybe_dispatch_pending(can_dispatch=self._can_dispatch())
        self._apply(decision)

    # --- Outputs -------------------------------------------------------------

    def _apply(self, decision: Decision) -> None:
        if decision.action == Action.NONE:
            if decision.reason in {"duplicate_active", "duplicate_pending"}:
                self._throttled(decision.reason, "Suppressing near-duplicate goal.", 5.0)
            return
        if decision.action == Action.QUEUE and decision.goal is not None:
            self.get_logger().info(
                "Queued goal: x=%.2f y=%.2f (%s)"
                % (decision.goal.x, decision.goal.y, decision.reason)
            )
            self._publish_status(
                "ACTIVE + pending x=%.2f y=%.2f" % (decision.goal.x, decision.goal.y)
            )
            return
        if decision.action == Action.PREEMPT and decision.goal is not None:
            self._request_preempt(decision.goal)
            return
        if decision.action == Action.DISPATCH and decision.goal is not None:
            self._dispatch(decision.goal)

    def _dispatch(self, goal: GoalPose) -> None:
        msg = self._pose_msg(goal)
        try:
            with self._navigator_lock:
                accepted = self._navigator.goToPose(msg)
        except Exception as exc:
            self.get_logger().error("Nav2 goToPose() raised: %s" % exc)
            self._record_failure(goal, "dispatch_error")
            self._apply(self._policy.complete_active(Outcome.FAILED))
            return
        if not accepted:
            self.get_logger().error(
                "Nav2 rejected goal x=%.2f y=%.2f." % (goal.x, goal.y)
            )
            self._record_failure(goal, "rejected")
            self._apply(self._policy.complete_active(Outcome.FAILED))
            return
        self._active_started_at = time.monotonic()
        self._timeout_requested = False
        self._preempt_requested = False
        self._validation_cancel_requested = False
        self._validation_cancel_reason = ""
        self._localization_cancel_requested = False
        self.get_logger().info("Sent goal to Nav2: x=%.2f y=%.2f" % (goal.x, goal.y))
        self._publish_status("ACTIVE x=%.2f y=%.2f" % (goal.x, goal.y))

    def _request_preempt(self, pending_goal: GoalPose) -> None:
        active = self._policy.active
        if self._preempt_requested:
            self.get_logger().info(
                "Updated pending preempt goal: x=%.2f y=%.2f"
                % (pending_goal.x, pending_goal.y)
            )
            return
        self._preempt_requested = True
        self.get_logger().info(
            "Preempting active goal x=%.2f y=%.2f with x=%.2f y=%.2f."
            % (
                active.x if active else 0.0,
                active.y if active else 0.0,
                pending_goal.x,
                pending_goal.y,
            )
        )
        self._publish_status(
            "PREEMPTING -> x=%.2f y=%.2f" % (pending_goal.x, pending_goal.y)
        )
        try:
            with self._navigator_lock:
                self._navigator.cancelTask()
        except Exception as exc:
            self._preempt_requested = False
            self.get_logger().error("cancelTask() for preempt raised: %s" % exc)

    def _maybe_cancel_for_localization(self) -> None:
        if (
            not self._require_localization
            or self._localization_cancel_requested
            or self._timeout_requested
            or self._preempt_requested
            or self._validation_cancel_requested
        ):
            return
        if self._localization_state not in self._cancel_states:
            return
        active = self._policy.active
        self._localization_cancel_requested = True
        self.get_logger().error(
            "Localization %s while navigating to x=%.2f y=%.2f; canceling goal."
            % (
                self._localization_state,
                active.x if active else 0.0,
                active.y if active else 0.0,
            )
        )
        self._publish_status("CANCELING (localization %s)" % self._localization_state)
        try:
            with self._navigator_lock:
                self._navigator.cancelTask()
        except Exception as exc:
            self._localization_cancel_requested = False
            self.get_logger().error("cancelTask() for localization raised: %s" % exc)

    def _maybe_cancel_if_goal_invalid(self) -> None:
        if (
            not self._goal_validation_map_topic
            or self._validation_cancel_requested
            or self._timeout_requested
            or self._preempt_requested
        ):
            return
        active = self._policy.active
        if active is None:
            return
        validation_map = self._latest_validation_map
        if validation_map is None:
            self._throttled(
                "validation_map_unavailable",
                "Waiting for goal validation map on %s." % self._goal_validation_map_topic,
                5.0,
            )
            return
        map_frame = validation_map.header.frame_id.strip()
        if not frames_match(map_frame, self._global_frame):
            self._throttled(
                "validation_frame_mismatch",
                "Skipping goal validation map in frame '%s' (expected '%s')."
                % (map_frame or "<empty>", self._global_frame),
                5.0,
            )
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                self._global_frame,
                self._robot_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self._tf_timeout),
            )
        except TransformException as exc:
            self._throttled(
                "validation_tf_unavailable",
                "Skipping goal validation; missing TF %s -> %s: %s"
                % (self._global_frame, self._robot_base_frame, exc),
                5.0,
            )
            return

        grid = self._validation_grid(validation_map)
        robot_pose = Pose2D(
            x=float(transform.transform.translation.x),
            y=float(transform.transform.translation.y),
        )
        try:
            result = validate_goal_reachable(
                validation_map.data,
                grid,
                robot_pose,
                Pose2D(active.x, active.y),
                reachable_cost_threshold=self._goal_validation_reachable_cost_threshold,
                seed_search_radius=self._goal_validation_seed_search_radius,
                connectivity=self._goal_validation_connectivity,
                treat_unknown_as_reachable=self._treat_unknown_as_reachable,
            )
        except ValueError as exc:
            self._throttled("validation_grid_invalid", "Skipping goal validation: %s" % exc, 5.0)
            return
        if result.valid:
            return

        self._validation_cancel_requested = True
        self._validation_cancel_reason = result.reason
        self.get_logger().warning(
            "Active goal x=%.2f y=%.2f invalid on %s (%s); canceling."
            % (active.x, active.y, self._goal_validation_map_topic, result.reason)
        )
        try:
            with self._navigator_lock:
                self._navigator.cancelTask()
        except Exception as exc:
            self._validation_cancel_requested = False
            self._validation_cancel_reason = ""
            self.get_logger().error("cancelTask() for goal validation raised: %s" % exc)

    def _maybe_cancel_for_timeout(self) -> None:
        if not should_cancel_for_timeout(
            started_at_monotonic=self._active_started_at,
            now_monotonic=time.monotonic(),
            timeout_sec=self._goal_timeout,
            already_requested=self._timeout_requested,
        ):
            return
        self._timeout_requested = True
        active = self._policy.active
        self.get_logger().error(
            "Timeout (%.0fs) on goal x=%.2f y=%.2f; canceling."
            % (self._goal_timeout, active.x if active else 0.0, active.y if active else 0.0)
        )
        try:
            with self._navigator_lock:
                self._navigator.cancelTask()
        except Exception as exc:
            self.get_logger().error("cancelTask() raised: %s" % exc)

    def _maybe_collect_result(self) -> None:
        try:
            with self._navigator_lock:
                if not self._navigator.isTaskComplete():
                    return
                result = self._navigator.getResult()
        except Exception as exc:
            self.get_logger().error("Nav2 result query failed: %s" % exc)
            result = TaskResult.UNKNOWN

        active = self._policy.active
        was_timeout = self._timeout_requested
        was_preempt = self._preempt_requested
        was_validation = self._validation_cancel_requested
        was_localization = self._localization_cancel_requested
        validation_reason = self._validation_cancel_reason
        self._timeout_requested = False
        self._preempt_requested = False
        self._validation_cancel_requested = False
        self._validation_cancel_reason = ""
        self._localization_cancel_requested = False
        self._active_started_at = None

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(
                "Goal succeeded: x=%.2f y=%.2f"
                % (active.x if active else 0.0, active.y if active else 0.0)
            )
            outcome = Outcome.SUCCEEDED
            self._last_result_text = "succeeded"
            self._last_failed_goal = None
        elif result == TaskResult.CANCELED:
            log = self.get_logger().error if was_timeout else self.get_logger().warning
            if was_timeout:
                reason = "canceled (timeout)"
            elif was_preempt:
                reason = "canceled (preempted)"
            elif was_validation:
                reason = "canceled (validation: %s)" % (validation_reason or "invalid")
            elif was_localization:
                reason = "canceled (localization lost)"
            else:
                reason = "canceled"
            log(
                "Goal %s: x=%.2f y=%.2f"
                % (
                    reason,
                    active.x if active else 0.0,
                    active.y if active else 0.0,
                )
            )
            outcome = Outcome.CANCELED
            self._last_result_text = reason
            if not was_preempt and active is not None:
                self._last_failed_goal = active
        else:
            self.get_logger().error(
                "Goal failed: x=%.2f y=%.2f"
                % (active.x if active else 0.0, active.y if active else 0.0)
            )
            outcome = Outcome.FAILED
            self._last_result_text = "failed"
            self._last_failed_goal = active

        self._apply(self._policy.complete_active(outcome))
        if not self._policy.has_active():
            self._publish_status("IDLE (last: %s)" % self._last_result_text)

    # --- Helpers -------------------------------------------------------------

    def _localization_ok(self) -> bool:
        return self._localization_state in self._dispatch_states

    def _can_dispatch(self) -> bool:
        if self._require_localization and not self._localization_ok():
            return False
        return (not self._policy.has_active()) and self._has_required_tf(log=False)

    def _has_required_tf(self, log: bool) -> bool:
        try:
            self._tf_buffer.lookup_transform(
                self._global_frame, self._robot_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self._tf_timeout),
            )
        except TransformException as exc:
            if log:
                self._throttled(
                    "tf_unavailable",
                    "Waiting for TF %s -> %s: %s" % (self._global_frame, self._robot_base_frame, exc),
                    5.0,
                )
            return False
        return True

    def _record_failure(self, goal: GoalPose, reason: str) -> None:
        self._last_result_text = reason
        self._last_failed_goal = goal

    def _validation_grid(self, msg: OccupancyGrid) -> ValidationGrid:
        q = msg.info.origin.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return ValidationGrid(
            width=int(msg.info.width),
            height=int(msg.info.height),
            resolution=float(msg.info.resolution),
            origin_x=float(msg.info.origin.position.x),
            origin_y=float(msg.info.origin.position.y),
            origin_yaw=yaw,
        )

    def _pose_msg(self, goal: GoalPose) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = goal.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = goal.x
        msg.pose.position.y = goal.y
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = goal.qz
        msg.pose.orientation.w = goal.qw
        return msg

    def _publish_status(self, text: str) -> None:
        self._status_pub.publish(String(data=text))
        self._publish_markers(text)

    def _publish_markers(self, status_text: str) -> None:
        markers = MarkerArray()
        wipe = Marker()
        wipe.action = Marker.DELETEALL
        markers.markers.append(wipe)

        def _arrow(goal: GoalPose, marker_id: int, rgba) -> Marker:
            marker = Marker()
            marker.header.frame_id = self._global_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "goal_executor"
            marker.id = marker_id
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = goal.x
            marker.pose.position.y = goal.y
            marker.pose.position.z = 0.1
            marker.pose.orientation.z = goal.qz
            marker.pose.orientation.w = goal.qw
            marker.scale.x, marker.scale.y, marker.scale.z = 0.6, 0.12, 0.12
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
            return marker

        if self._policy.active is not None:
            markers.markers.append(
                _arrow(self._policy.active, 1, (0.1, 0.9, 0.1, 0.9))
            )
        if self._policy.pending is not None:
            markers.markers.append(
                _arrow(self._policy.pending, 2, (0.9, 0.9, 0.1, 0.9))
            )
        if self._last_failed_goal is not None:
            markers.markers.append(
                _arrow(self._last_failed_goal, 3, (0.9, 0.1, 0.1, 0.9))
            )

        anchor = (
            self._policy.active
            or self._policy.pending
            or self._last_failed_goal
        )
        if anchor is not None:
            text = Marker()
            text.header.frame_id = self._global_frame
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = "goal_executor"
            text.id = 4
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = anchor.x
            text.pose.position.y = anchor.y
            text.pose.position.z = 0.8
            text.scale.z = 0.3
            text.color.r = text.color.g = text.color.b = 1.0
            text.color.a = 0.9
            text.text = status_text
            markers.markers.append(text)

        self._markers_pub.publish(markers)

    def _throttled(self, key: str, message: str, period_s: float) -> None:
        now = time.monotonic()
        last = self._throttle.get(key, 0.0)
        if (now - last) >= period_s:
            self._throttle[key] = now
            self.get_logger().warning(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalPoseExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    main()
