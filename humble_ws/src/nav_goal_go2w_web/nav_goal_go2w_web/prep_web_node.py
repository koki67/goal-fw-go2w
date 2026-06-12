"""Publish a bounded live mapping preview and expose browser map finalization."""
from __future__ import annotations

import os
import signal
import threading
import time
from pathlib import Path

import rclpy
from direct_lidar_inertial_odometry.srv import SavePCD
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger

from nav_goal_go2w_map.finish_map_core import finish_map
from nav_goal_go2w_web.prep_grid_core import downsample_voxel, pointcloud2_to_xyz, project_points

LATCHED_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)


class PrepWebNode(Node):
    def __init__(self) -> None:
        super().__init__("prep_web_node")
        for name, default in (("cloud_topic", "/dlio/map_node/map"), ("output", ""),
                              ("save_leaf_size", 0.05), ("resolution", 0.10),
                              ("z_min", -0.25), ("z_max", 1.75), ("publish_period_s", 2.0),
                              ("preview_leaf_size", 0.15), ("preview_max_points", 150_000),
                              ("shutdown_after_done_s", 3.0)):
            self.declare_parameter(name, default)
        self._latest_msg = None
        self._cloud_lock = threading.Lock()
        self._finish_lock = threading.Lock()
        self._grid_pub = self.create_publisher(OccupancyGrid, "/web/prep_grid", LATCHED_QOS)
        self._cloud_pub = self.create_publisher(PointCloud2, "/web/prep_cloud", LATCHED_QOS)
        self._status_pub = self.create_publisher(String, "/web/prep_status", LATCHED_QOS)
        self._save_group = ReentrantCallbackGroup()
        self._service_group = MutuallyExclusiveCallbackGroup()
        self._save_client = self.create_client(SavePCD, "/save_pcd", callback_group=self._save_group)
        self.create_subscription(PointCloud2, str(self.get_parameter("cloud_topic").value), self._cloud_cb, qos_profile_sensor_data)
        self.create_service(Trigger, "/web/finish_map", self._finish_cb, callback_group=self._service_group)
        self.create_timer(float(self.get_parameter("publish_period_s").value), self._publish_grid)
        self._status("IDLE")

    def _status(self, text: str) -> None:
        self._status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def _cloud_cb(self, msg: PointCloud2) -> None:
        # D-LIO republishes the whole cumulative map on every keyframe, so only
        # keep the newest message here; decoding happens once per preview
        # interval in _publish_grid, not per publication.
        with self._cloud_lock:
            self._latest_msg = msg

    def _publish_grid(self) -> None:
        with self._cloud_lock:
            msg = self._latest_msg
            self._latest_msg = None
        if msg is None:
            return
        points = pointcloud2_to_xyz(msg)
        frame = msg.header.frame_id or "odom"
        stamp = self.get_clock().now().to_msg()
        grid = project_points(points, resolution=float(self.get_parameter("resolution").value),
            z_min=float(self.get_parameter("z_min").value), z_max=float(self.get_parameter("z_max").value))
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = stamp
        grid_msg.header.frame_id = frame
        grid_msg.info.map_load_time = stamp
        grid_msg.info.resolution = grid.resolution
        grid_msg.info.width = grid.width
        grid_msg.info.height = grid.height
        grid_msg.info.origin.position.x = grid.origin_x
        grid_msg.info.origin.position.y = grid.origin_y
        grid_msg.info.origin.orientation.w = 1.0
        grid_msg.data = grid.data.tolist()
        self._grid_pub.publish(grid_msg)
        preview = downsample_voxel(points, leaf=float(self.get_parameter("preview_leaf_size").value),
            max_points=int(self.get_parameter("preview_max_points").value))
        self._cloud_pub.publish(point_cloud2.create_cloud_xyz32(Header(stamp=stamp, frame_id=frame), preview))

    def _save(self, raw_dir: Path, leaf: float) -> None:
        if not self._save_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("map save service is unavailable: /save_pcd")
        request = SavePCD.Request(leaf_size=leaf, save_path=str(raw_dir))
        future = self._save_client.call_async(request)
        while not future.done():
            time.sleep(0.05)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError("D-LIO reported a failed save")

    def _finish_cb(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if not self._finish_lock.acquire(blocking=False):
            response.success = False
            response.message = "map finalization is already running"
            return response
        try:
            output = str(self.get_parameter("output").value)
            result = finish_map(output, float(self.get_parameter("save_leaf_size").value), self._save, self._status)
            response.success = True
            response.message = str(result)
            self._schedule_collect_shutdown()
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        finally:
            self._finish_lock.release()
        return response

    def _schedule_collect_shutdown(self) -> None:
        delay = float(self.get_parameter("shutdown_after_done_s").value)
        if delay <= 0.0:
            return

        def _fire() -> None:
            # The delay lets the DONE status and the service response reach the
            # browser through rosbridge before rosbridge itself goes down.
            time.sleep(delay)
            self.get_logger().info("Finalization complete; stopping the collection launch.")
            # The collection launch shares this node's process group, so a
            # group-wide SIGINT is the same orderly shutdown as Ctrl-C in the
            # tmux collect window (the Enter path's finish_prepare_map.sh).
            os.killpg(os.getpgrp(), signal.SIGINT)

        threading.Thread(target=_fire, daemon=True).start()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PrepWebNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
