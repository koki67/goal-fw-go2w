from __future__ import annotations

import ast
from pathlib import Path


SRC_ROOT = Path(__file__).resolve().parents[2]
LOGGER_METHODS = {"debug", "info", "warning", "warn", "error", "fatal"}


def _package_python_files() -> list[Path]:
    """Every custom nav_goal_go2w_* python module in the workspace."""
    files: list[Path] = []
    for package_dir in sorted(SRC_ROOT.glob("nav_goal_go2w_*")):
        module_dir = package_dir / package_dir.name
        if module_dir.is_dir():
            files.extend(sorted(module_dir.glob("*.py")))
    assert files, "no nav_goal_go2w_* packages found"
    return files


def test_rclpy_logger_calls_are_humble_compatible():
    # Humble's rclpy loggers do not support printf-style varargs.
    offenders: list[str] = []

    for path in _package_python_files():
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            if not isinstance(node.func, ast.Attribute):
                continue
            if node.func.attr not in LOGGER_METHODS:
                continue
            if len(node.args) <= 1:
                continue

            offenders.append(f"{path.relative_to(SRC_ROOT)}:{node.lineno}")

    assert offenders == []
