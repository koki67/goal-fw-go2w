#!/usr/bin/env bash
# Run the closed-loop desktop simulation of the goal navigation stack.
#
# Usage:
#   .devcontainer/run_desktop_sim.bash [--world NAME_OR_PATH] [--map DIR]
#       [--rviz] [--allow-running-sim] [launch_arg:=value ...]
#
# Examples:
#   .devcontainer/run_desktop_sim.bash --world open_room --rviz
#   .devcontainer/run_desktop_sim.bash --world doorway --rviz odom_drift_yaw_per_m:=0.02
#   .devcontainer/run_desktop_sim.bash --world open_room --rviz sim_localization:=false
#
# The map directory defaults to maps/sim_<world> at the repo root; generate it
# once with:
#   ros2 run nav_goal_go2w_sim gen_sim_map --world <world> --output maps/sim_<world>
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

WORLD="open_room"
MAP_DIR=""
USE_RVIZ="false"
ALLOW_RUNNING_SIM="${ALLOW_RUNNING_SIM:-false}"

while [ "$#" -gt 0 ]; do
    case "$1" in
        --world)
            WORLD="$2"
            shift 2
            ;;
        --map)
            MAP_DIR="$2"
            shift 2
            ;;
        --rviz)
            USE_RVIZ="true"
            shift
            ;;
        --allow-running-sim)
            ALLOW_RUNNING_SIM="true"
            shift
            ;;
        --help|-h)
            sed -n '2,16p' "$0"
            exit 0
            ;;
        --)
            shift
            break
            ;;
        *)
            break
            ;;
    esac
done

source /opt/ros/humble/setup.bash

if [ ! -f "${REPO_ROOT}/humble_ws/install/setup.bash" ]; then
    echo "Built workspace not found: ${REPO_ROOT}/humble_ws/install/setup.bash" >&2
    echo "Build it first from the devcontainer:" >&2
    echo "  ${SCRIPT_DIR}/build_desktop_sim_workspace.bash" >&2
    exit 2
fi
source "${REPO_ROOT}/humble_ws/install/setup.bash"

if [ -z "${MAP_DIR}" ]; then
    MAP_DIR="${REPO_ROOT}/maps/sim_${WORLD}"
fi
if [ ! -f "${MAP_DIR}/grid.yaml" ]; then
    echo "Map directory ${MAP_DIR} is missing grid.yaml. Generate it first:" >&2
    echo "  ros2 run nav_goal_go2w_sim gen_sim_map --world ${WORLD} --output ${MAP_DIR}" >&2
    exit 2
fi

SIM_NODE_EXE="${REPO_ROOT}/humble_ws/install/nav_goal_go2w_sim/lib/nav_goal_go2w_sim/sim_node"
if [ "${ALLOW_RUNNING_SIM}" != "true" ] && [ -x "${SIM_NODE_EXE}" ]; then
    RUNNING_SIM="$(pgrep -af "${SIM_NODE_EXE}" || true)"
    if [ -n "${RUNNING_SIM}" ]; then
        echo "A desktop simulator is already running:" >&2
        printf '%s\n' "${RUNNING_SIM}" >&2
        echo "Stop the previous run first; duplicate simulators publish competing /clock and TF data." >&2
        exit 2
    fi
fi

exec ros2 launch nav_goal_go2w_sim sim_bringup.launch.py \
    "world_file:=${WORLD}" \
    "map:=${MAP_DIR}" \
    "use_rviz:=${USE_RVIZ}" \
    "$@"
