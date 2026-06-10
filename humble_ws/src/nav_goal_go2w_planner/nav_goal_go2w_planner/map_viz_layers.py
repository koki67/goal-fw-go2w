"""Publish a visualization-only global costmap with unknown cells hidden."""
from __future__ import annotations

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class MapVizLayersNode(Node):
    """Filter unknown cells out of the global costmap for RViz only."""

    def __init__(self) -> None:
        super().__init__("map_viz_layers")

        self.declare_parameter("input_topic", "/global_costmap/costmap")
        self.declare_parameter("output_topic", "/global_costmap/costmap_known_only")
        self.declare_parameter("unknown_replacement_value", 0)

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._unknown_replacement_value = int(self.get_parameter("unknown_replacement_value").value)

        costmap_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._pub = self.create_publisher(OccupancyGrid, self._output_topic, costmap_qos)
        self._sub = self.create_subscription(OccupancyGrid, self._input_topic, self._on_costmap, costmap_qos)

        self.get_logger().info(
            "Global costmap RViz filter ready: input=%s output=%s unknown->%d"
            % (self._input_topic, self._output_topic, self._unknown_replacement_value)
        )

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        filtered = OccupancyGrid()
        filtered.header = msg.header
        filtered.info = msg.info
        filtered.data = [
            self._unknown_replacement_value if int(value) < 0 else int(value)
            for value in msg.data
        ]
        self._pub.publish(filtered)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapVizLayersNode()
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
