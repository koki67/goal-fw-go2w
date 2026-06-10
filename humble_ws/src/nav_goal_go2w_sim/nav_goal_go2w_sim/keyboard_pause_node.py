"""Terminal keyboard control for the desktop simulator."""

from __future__ import annotations

import select
import sys
import termios
import tty
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class KeyboardPauseNode(Node):
    """Toggle /sim/pause from a terminal key without blocking ROS spin."""

    def __init__(self) -> None:
        super().__init__("sim_keyboard_pause")
        self._paused = False
        self._old_termios: Optional[list] = None
        self._pub = self.create_publisher(Bool, "/sim/pause", 10)
        self._status_sub = self.create_subscription(
            Bool,
            "/sim/paused",
            self._on_paused_status,
            10,
        )

        if sys.stdin.isatty():
            self._old_termios = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            self.create_timer(0.05, self._poll_keyboard)
            self.get_logger().info("Press 'p' or space in this terminal to pause/resume the simulator.")
        else:
            self.get_logger().warn("Keyboard pause disabled because stdin is not a TTY.")

    def destroy_node(self) -> bool:
        self._restore_terminal()
        return super().destroy_node()

    def _poll_keyboard(self) -> None:
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not readable:
            return

        char = sys.stdin.read(1)
        if char not in {"p", "P", " "}:
            return

        self._paused = not self._paused
        msg = Bool()
        msg.data = self._paused
        self._pub.publish(msg)
        state = "paused" if self._paused else "running"
        self.get_logger().info(f"Requested simulator {state}.")

    def _on_paused_status(self, msg: Bool) -> None:
        self._paused = msg.data

    def _restore_terminal(self) -> None:
        if self._old_termios is None:
            return
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_termios)
        self._old_termios = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[KeyboardPauseNode] = None
    try:
        node = KeyboardPauseNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
