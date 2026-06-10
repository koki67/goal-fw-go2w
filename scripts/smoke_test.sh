#!/bin/bash
# Smoke test for the goal navigation stack.
#
# Default (sim mode, no hardware or bag needed):
#   bash scripts/smoke_test.sh
# Brings up the closed-loop simulator with the committed sim_open_room map,
# publishes /initialpose, waits for localization TRACKING, publishes a
# /goal_pose, and asserts that /cmd_vel starts publishing.
#
# Bag mode (robot data):
#   bash scripts/smoke_test.sh --bag /path/to/bag --map /external/maps/<name>
# Brings up the real bringup (bridge dry-run) against a replayed bag instead
# of the simulator; same initialpose/goal/cmd_vel assertions.
#
# Run inside the container (after docker/run.sh) so the workspace overlay is
# sourced and dependencies are available.
set -e

MODE="sim"
BAG_PATH=""
MAP_DIR="/external/maps/sim_open_room"

while [ "$#" -gt 0 ]; do
    case "$1" in
        --bag)
            MODE="bag"
            BAG_PATH="$2"
            shift 2
            ;;
        --map)
            MAP_DIR="$2"
            shift 2
            ;;
        -h|--help)
            sed -n '2,16p' "$0"
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            exit 2
            ;;
    esac
done

if ! command -v ros2 >/dev/null 2>&1; then
    echo "ros2 not in PATH. Run this inside the docker container (docker/run.sh)." >&2
    exit 1
fi

source /opt/ros/humble/setup.bash
[ -f /workspace/humble_ws/install/setup.bash ] && source /workspace/humble_ws/install/setup.bash

if [ ! -f "$MAP_DIR/grid.yaml" ]; then
    echo "Map directory $MAP_DIR is missing grid.yaml." >&2
    echo "For sim mode: ros2 run nav_goal_go2w_sim gen_sim_map --world open_room --output $MAP_DIR" >&2
    exit 1
fi

LOG_DIR=/tmp/nav_goal_smoke_$$
mkdir -p "$LOG_DIR"
BAG_PID=""

if [ "$MODE" = "sim" ]; then
    echo "[smoke] Launching closed-loop sim (map: $MAP_DIR), logs in $LOG_DIR ..."
    ros2 launch nav_goal_go2w_sim sim_bringup.launch.py \
        map:="$MAP_DIR" \
        use_rviz:=false \
        odom_drift_yaw_per_m:=0.01 > "$LOG_DIR/bringup.log" 2>&1 &
    BRINGUP_PID=$!
    SIM_TIME_FLAG="--use-sim-time"
else
    if [ ! -e "$BAG_PATH" ]; then
        echo "Bag not found at: $BAG_PATH" >&2
        exit 1
    fi
    echo "[smoke] Launching bringup (bridge dry-run, map: $MAP_DIR), logs in $LOG_DIR ..."
    ros2 launch nav_goal_go2w_bringup bringup.launch.py \
        map:="$MAP_DIR" \
        use_rviz:=false \
        bridge_dry_run:=true \
        use_sim_time:=true > "$LOG_DIR/bringup.log" 2>&1 &
    BRINGUP_PID=$!
    SIM_TIME_FLAG="--use-sim-time"
fi

cleanup() {
    echo "[smoke] cleaning up..."
    kill -INT "$BRINGUP_PID" 2>/dev/null || true
    [ -n "$BAG_PID" ] && kill -INT "$BAG_PID" 2>/dev/null || true
    wait "$BRINGUP_PID" 2>/dev/null || true
    [ -n "$BAG_PID" ] && wait "$BAG_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[smoke] Waiting 12s for nodes to come up..."
sleep 12

if [ "$MODE" = "bag" ]; then
    echo "[smoke] Replaying bag..."
    ros2 bag play "$BAG_PATH" --clock > "$LOG_DIR/bag_play.log" 2>&1 &
    BAG_PID=$!
    sleep 5
fi

echo "[smoke] Publishing /initialpose..."
ros2 topic pub --once $SIM_TIME_FLAG /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    '{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}' \
    > "$LOG_DIR/initialpose.log" 2>&1

echo "[smoke] Waiting for localization TRACKING (up to 30s)..."
TRACKING_OK=false
for _ in $(seq 1 30); do
    STATE=$(timeout 2 ros2 topic echo --once /localization/state 2>/dev/null | grep -oP "data: '\K[A-Z]+" || true)
    if [ "$STATE" = "TRACKING" ]; then
        echo "[smoke] localization TRACKING"
        TRACKING_OK=true
        break
    fi
    sleep 1
done
if [ "$TRACKING_OK" != "true" ]; then
    echo "FAIL: localization never reached TRACKING (last: ${STATE:-none}). See $LOG_DIR/bringup.log." >&2
    exit 1
fi

echo "[smoke] Verifying map->odom TF..."
if ! timeout 5 ros2 run tf2_ros tf2_echo map odom > "$LOG_DIR/tf_echo.log" 2>&1; then
    grep -q "Translation" "$LOG_DIR/tf_echo.log" || {
        echo "FAIL: no map->odom TF. See $LOG_DIR/tf_echo.log." >&2
        exit 1
    }
fi

echo "[smoke] Publishing /goal_pose..."
ros2 topic pub --once $SIM_TIME_FLAG /goal_pose geometry_msgs/msg/PoseStamped \
    '{header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}' \
    > "$LOG_DIR/goal.log" 2>&1

echo "[smoke] Waiting for /cmd_vel (up to 60s)..."
CMD_OK=false
for _ in $(seq 1 60); do
    if timeout 1 ros2 topic echo --once /cmd_vel >/dev/null 2>&1; then
        echo "[smoke] saw /cmd_vel"
        CMD_OK=true
        break
    fi
    sleep 1
done

if [ "$CMD_OK" != "true" ]; then
    echo "FAIL: /cmd_vel never published. See $LOG_DIR/bringup.log." >&2
    exit 1
fi

echo "[smoke] PASS: initialpose -> TRACKING -> goal -> /cmd_vel"
