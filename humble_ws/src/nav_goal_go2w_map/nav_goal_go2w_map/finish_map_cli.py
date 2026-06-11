"""Save the active D-LIO map and atomically prepare navigation artifacts."""
from __future__ import annotations

import argparse
from pathlib import Path

import rclpy
from direct_lidar_inertial_odometry.srv import SavePCD
from rclpy.node import Node

from nav_goal_go2w_map.finish_map_core import finish_map


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="finish_map", description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--save-leaf-size", type=float, default=0.05)
    parser.add_argument("--save-service", default="/save_pcd")
    return parser


def _save_with_service(node: Node, service_name: str, raw_dir: Path, leaf: float) -> None:
    client = node.create_client(SavePCD, service_name)
    if not client.wait_for_service(timeout_sec=5.0):
        raise RuntimeError(f"map save service is unavailable: {service_name}")
    request = SavePCD.Request()
    request.leaf_size = leaf
    request.save_path = str(raw_dir)
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
    if response is None or not response.success:
        raise RuntimeError("D-LIO reported a failed save")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    rclpy.init()
    node = Node("finish_map_cli")
    try:
        result = finish_map(args.output, args.save_leaf_size,
            lambda raw_dir, leaf: _save_with_service(node, args.save_service, raw_dir, leaf),
            lambda status: node.get_logger().info(status))
        print(f"Prepared map written to: {result}")
        print(f"Raw D-LIO cloud retained at: {result}/raw/dlio_map.pcd")
        return 0
    except Exception as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
