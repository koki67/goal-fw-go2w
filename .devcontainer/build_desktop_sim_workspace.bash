#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="${REPO_ROOT}/humble_ws"

source /opt/ros/humble/setup.bash

# With symlink-install, generated build/install trees can keep file lists from
# another branch. Clean only the desktop-sim dependency packages that often
# change across the active feature branches.
rm -rf \
    "${WS_DIR}/build/go2w_description" \
    "${WS_DIR}/install/go2w_description" \
    "${WS_DIR}/build/go2w_slam_toolbox_bringup" \
    "${WS_DIR}/install/go2w_slam_toolbox_bringup" \
    "${WS_DIR}/build/nav_goal_go2w_map" \
    "${WS_DIR}/install/nav_goal_go2w_map" \
    "${WS_DIR}/build/nav_goal_go2w_localization" \
    "${WS_DIR}/install/nav_goal_go2w_localization" \
    "${WS_DIR}/build/nav_goal_go2w_planner" \
    "${WS_DIR}/install/nav_goal_go2w_planner" \
    "${WS_DIR}/build/nav_goal_go2w_sim" \
    "${WS_DIR}/install/nav_goal_go2w_sim"

cd "${WS_DIR}"
colcon build --symlink-install --packages-up-to nav_goal_go2w_sim "$@"
