"""Normalize Nav2 MPPI trajectory markers into thin line-strip markers.

Nav2 MPPI publishes sampled rollout visualization as a MarkerArray on
`/trajectories`. This helper keeps that data visual-only, but republishes it in
a consistent line style that is easier to inspect in RViz and replay bags.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class MppiTrajectoryLinesNode(Node):
    """Convert MPPI trajectory markers to line markers for RViz."""

    def __init__(self) -> None:
        super().__init__("mppi_trajectory_lines")

        self.declare_parameter("input_topic", "/trajectories")
        self.declare_parameter("output_topic", "/mppi_trajectory_lines")
        self.declare_parameter("line_width", 0.015)
        self.declare_parameter("alpha", 0.35)
        self.declare_parameter("color_rgb", [80.0, 180.0, 255.0])
        self.declare_parameter("z_offset", 0.04)
        self.declare_parameter("optimal_line_width", 0.04)
        self.declare_parameter("optimal_alpha", 0.95)
        self.declare_parameter("optimal_color_rgb", [255.0, 220.0, 40.0])
        self.declare_parameter("optimal_z_offset", 0.12)
        self.declare_parameter("lifetime_sec", 0.5)
        self.declare_parameter("max_markers", 400)
        self.declare_parameter("point_stride", 1)
        self.declare_parameter("pose_markers_per_trajectory", 10)
        self.declare_parameter("preserve_marker_color", False)
        self.declare_parameter("candidate_marker_namespace", "Candidate Trajectories")
        self.declare_parameter("optimal_marker_namespace", "Optimal Trajectory")

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._candidate_style = _LineStyle(
            namespace="mppi_candidate_trajectory_lines",
            line_width=max(float(self.get_parameter("line_width").value), 0.001),
            alpha=_clamp(float(self.get_parameter("alpha").value), 0.0, 1.0),
            color_rgb=_rgb_param(self.get_parameter("color_rgb").value),
            z_offset=float(self.get_parameter("z_offset").value),
        )
        self._optimal_style = _LineStyle(
            namespace="mppi_optimal_trajectory",
            line_width=max(float(self.get_parameter("optimal_line_width").value), 0.001),
            alpha=_clamp(float(self.get_parameter("optimal_alpha").value), 0.0, 1.0),
            color_rgb=_rgb_param(self.get_parameter("optimal_color_rgb").value),
            z_offset=float(self.get_parameter("optimal_z_offset").value),
        )
        self._lifetime_sec = max(float(self.get_parameter("lifetime_sec").value), 0.0)
        self._max_markers = max(int(self.get_parameter("max_markers").value), 0)
        self._point_stride = max(int(self.get_parameter("point_stride").value), 1)
        self._pose_markers_per_trajectory = max(
            int(self.get_parameter("pose_markers_per_trajectory").value),
            0,
        )
        self._preserve_marker_color = bool(self.get_parameter("preserve_marker_color").value)
        self._candidate_marker_namespace = str(
            self.get_parameter("candidate_marker_namespace").value
        )
        self._optimal_marker_namespace = str(
            self.get_parameter("optimal_marker_namespace").value
        )

        self._pub = self.create_publisher(MarkerArray, self._output_topic, 10)
        self._sub = self.create_subscription(
            MarkerArray, self._input_topic, self._on_markers, 10,
        )

        self.get_logger().info(
            "MPPI trajectory line visualizer ready: %s -> %s candidate_width=%.3f "
            "optimal_width=%.3f"
            % (
                self._input_topic,
                self._output_topic,
                self._candidate_style.line_width,
                self._optimal_style.line_width,
            )
        )

    def _on_markers(self, msg: MarkerArray) -> None:
        output = MarkerArray()

        candidate_pose_markers: list[Marker] = []
        optimal_pose_markers: list[Marker] = []
        count = 0
        for marker in msg.markers:
            style = self._style_for_marker(marker)
            if _is_pose_sample_marker(marker):
                if style is self._optimal_style:
                    optimal_pose_markers.append(marker)
                else:
                    candidate_pose_markers.append(marker)
                continue

            line_marker = self._to_line_marker(marker, count, style)
            if line_marker is None:
                continue
            if self._max_markers and count >= self._max_markers:
                break
            output.markers.append(line_marker)
            count += 1

        for line_marker in self._candidate_pose_markers_to_lines(
            candidate_pose_markers, count
        ):
            if self._max_markers and count >= self._max_markers:
                break
            output.markers.append(line_marker)
            count += 1

        optimal_line = self._optimal_pose_markers_to_line(optimal_pose_markers)
        if optimal_line is not None and (not self._max_markers or count < self._max_markers):
            output.markers.append(optimal_line)

        self._pub.publish(output)

    def _candidate_pose_markers_to_lines(
        self, markers: list[Marker], first_id: int
    ) -> list[Marker]:
        if self._pose_markers_per_trajectory < 2:
            return []

        sorted_markers = sorted(markers, key=lambda marker: marker.id)
        output: list[Marker] = []
        for start in range(0, len(sorted_markers), self._pose_markers_per_trajectory):
            chunk = sorted_markers[
                start:start + self._pose_markers_per_trajectory:self._point_stride
            ]
            if len(chunk) < 2:
                continue

            line = Marker()
            line.header = chunk[0].header
            line.ns = self._candidate_style.namespace
            line.id = first_id + len(output)
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.type = Marker.LINE_STRIP
            line.scale.x = self._candidate_style.line_width
            line.points = [
                _pose_marker_point(marker, self._candidate_style.z_offset)
                for marker in chunk
            ]
            line.frame_locked = chunk[0].frame_locked
            _set_lifetime(line, self._lifetime_sec)

            if self._preserve_marker_color and chunk[0].color.a > 0.0:
                line.color = chunk[0].color
                line.color.a = min(chunk[0].color.a, self._candidate_style.alpha)
            else:
                self._set_default_color(line, self._candidate_style)

            output.append(line)

        return output

    def _optimal_pose_markers_to_line(self, markers: list[Marker]) -> Marker | None:
        sorted_markers = sorted(markers, key=lambda marker: marker.id)
        sampled_markers = sorted_markers[::self._point_stride]
        if len(sampled_markers) < 2:
            return None

        line = Marker()
        line.header = sampled_markers[0].header
        line.ns = self._optimal_style.namespace
        line.id = 0
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.type = Marker.LINE_STRIP
        line.scale.x = self._optimal_style.line_width
        line.points = [
            _pose_marker_point(marker, self._optimal_style.z_offset)
            for marker in sampled_markers
        ]
        line.frame_locked = sampled_markers[0].frame_locked
        _set_lifetime(line, self._lifetime_sec)
        self._set_default_color(line, self._optimal_style)
        return line

    def _to_line_marker(
        self, marker: Marker, marker_id: int, style: "_LineStyle"
    ) -> Marker | None:
        if marker.action in (Marker.DELETE, Marker.DELETEALL):
            return None
        if len(marker.points) < 2:
            return None

        out = Marker()
        out.header = marker.header
        out.ns = style.namespace
        out.id = marker_id
        out.action = Marker.ADD
        out.pose = deepcopy(marker.pose)
        out.pose.position.z += style.z_offset
        out.type = Marker.LINE_LIST if marker.type == Marker.LINE_LIST else Marker.LINE_STRIP
        out.scale.x = style.line_width
        out.points = list(marker.points)[::self._point_stride]
        _set_lifetime(out, self._lifetime_sec)
        out.frame_locked = marker.frame_locked

        if len(out.points) < 2:
            return None

        if self._preserve_marker_color and marker.color.a > 0.0:
            out.color = marker.color
            out.color.a = min(marker.color.a, style.alpha)
        else:
            self._set_default_color(out, style)

        return out

    def _style_for_marker(self, marker: Marker) -> "_LineStyle":
        if marker.ns == self._optimal_marker_namespace:
            return self._optimal_style
        return self._candidate_style

    def _set_default_color(self, marker: Marker, style: "_LineStyle") -> None:
        marker.color.r = style.color_rgb[0]
        marker.color.g = style.color_rgb[1]
        marker.color.b = style.color_rgb[2]
        marker.color.a = style.alpha


@dataclass(frozen=True)
class _LineStyle:
    namespace: str
    line_width: float
    alpha: float
    color_rgb: tuple[float, float, float]
    z_offset: float


def _is_pose_sample_marker(marker: Marker) -> bool:
    if marker.action in (Marker.DELETE, Marker.DELETEALL):
        return False
    return (
        marker.type in (Marker.SPHERE, Marker.CUBE, Marker.CYLINDER)
        and len(marker.points) == 0
    )


def _pose_marker_point(marker: Marker, z_offset: float) -> Point:
    point = Point()
    point.x = marker.pose.position.x
    point.y = marker.pose.position.y
    point.z = marker.pose.position.z + z_offset
    return point


def _set_lifetime(marker: Marker, lifetime_sec: float) -> None:
    seconds = int(lifetime_sec)
    marker.lifetime.sec = seconds
    marker.lifetime.nanosec = int((lifetime_sec - seconds) * 1_000_000_000)


def _rgb_param(value) -> tuple[float, float, float]:
    vals = list(value)
    if len(vals) != 3:
        return (80.0 / 255.0, 180.0 / 255.0, 255.0 / 255.0)
    return tuple(_normalize_color(float(v)) for v in vals)


def _normalize_color(value: float) -> float:
    if value > 1.0:
        value = value / 255.0
    return _clamp(value, 0.0, 1.0)


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MppiTrajectoryLinesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
