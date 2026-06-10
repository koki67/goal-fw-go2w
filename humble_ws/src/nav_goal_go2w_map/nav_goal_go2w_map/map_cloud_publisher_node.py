"""Publish the pre-built map point cloud for RViz and diagnostics.

Loads a PCD once at startup and publishes it on /map_cloud as a latched
(TRANSIENT_LOCAL) PointCloud2 in the map frame. A slow republish timer is a
belt-and-suspenders for late joiners on lossy Wi-Fi links where the durable
sample can be dropped.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from nav_goal_go2w_map import pcd_io


class MapCloudPublisherNode(Node):
    """One-shot latched publisher for the pre-built map cloud."""

    def __init__(self) -> None:
        super().__init__("map_cloud_publisher")

        self.declare_parameter("cloud_path", "")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("topic", "/map_cloud")
        self.declare_parameter("republish_period_s", 10.0)

        cloud_path = str(self.get_parameter("cloud_path").value)
        if not cloud_path:
            raise ValueError("cloud_path parameter is required")
        self._frame_id = str(self.get_parameter("frame_id").value)
        topic = str(self.get_parameter("topic").value)
        republish_period = float(self.get_parameter("republish_period_s").value)

        points = pcd_io.load_xyz(cloud_path)
        self._points = points
        self._publisher = self.create_publisher(
            PointCloud2,
            topic,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )
        self._publish()
        if republish_period > 0.0:
            self._timer = self.create_timer(republish_period, self._publish)

        self.get_logger().info(
            f"map cloud ready: {len(points)} points from {cloud_path} "
            f"on {topic} (frame {self._frame_id})"
        )

    def _publish(self) -> None:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._frame_id
        self._publisher.publish(
            point_cloud2.create_cloud_xyz32(header, self._points)
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapCloudPublisherNode()
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
