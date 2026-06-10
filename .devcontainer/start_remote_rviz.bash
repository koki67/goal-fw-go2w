#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
IFACE="${1:-enp97s0}"

source /opt/ros/humble/setup.bash
if [ -f "${REPO_ROOT}/humble_ws/install/setup.bash" ]; then
    source "${REPO_ROOT}/humble_ws/install/setup.bash"
fi
source "${SCRIPT_DIR}/setup_remote_viz.bash" "${IFACE}"

exec rviz2 -d "${REPO_ROOT}/humble_ws/src/nav_goal_go2w_bringup/config/goal_remote.rviz"
