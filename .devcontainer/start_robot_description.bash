#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
USE_SIM_TIME="${1:-true}"
URDF_FILE="${REPO_ROOT}/humble_ws/src/go2w_description/urdf/go2w_description.urdf"

if [ ! -f "${URDF_FILE}" ]; then
    echo "Robot visual URDF not found: ${URDF_FILE}" >&2
    exit 2
fi

if [ -f /workspace/humble_ws/install/setup.bash ]; then
    source /workspace/humble_ws/install/setup.bash
fi
if [ -f "${REPO_ROOT}/humble_ws/install/setup.bash" ]; then
    source "${REPO_ROOT}/humble_ws/install/setup.bash"
fi

ROBOT_DESCRIPTION="$(<"${URDF_FILE}")"

exec ros2 run robot_state_publisher robot_state_publisher \
    --ros-args \
    -p "use_sim_time:=${USE_SIM_TIME}" \
    -p "robot_description:=${ROBOT_DESCRIPTION}"
