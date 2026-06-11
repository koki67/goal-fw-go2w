#!/bin/bash
# Start Go2W + Hesai D-LIO map collection and an interactive finish window.
set -euo pipefail

SESSION="go2w_prepare_map"
ATTACH=true
DRY_RUN=false
OUTPUT=""
USE_RVIZ="false"
DLIO_OUTPUT="screen"
SAVE_LEAF_SIZE="0.05"
WEB_UI="false"
WEB_UI_PORT="8080"
ROSBRIDGE_PORT="9090"

usage() {
    cat <<'USAGE'
Usage:
  scripts/prepare_map_tmux.sh [options] output:=/external/maps/<name> [arg:=value ...]

Options:
  --session NAME   tmux session name (default: go2w_prepare_map)
  --no-attach      create the session without attaching
  --dry-run        print the tmux commands without checking the ROS graph
  -h, --help       show this help

Arguments:
  output:=DIR               required prepared map directory; must not exist
  use_rviz:=false           launch D-LIO's live RViz mapping view
  dlio_output:=screen       D-LIO output target: screen or log
  save_leaf_size:=0.05      D-LIO save-time voxel leaf size in meters
  web_ui:=false             start browser UI on ports 8080 and 9090
  web_ui_port:=8080         browser HTTP port
  rosbridge_port:=9090      rosbridge WebSocket port

Prerequisite:
  The separately deployed go2w_teleop_gamepad system must already be running
  and visible as /go2w_teleop_gamepad_node on the same ROS graph.
USAGE
}

shell_join() {
    local token
    printf "%q" "$1"
    shift
    for token in "$@"; do
        printf " %q" "$token"
    done
}

window_shell() {
    local launch_cmd="$1"
    printf "source /opt/ros/humble/setup.bash && "
    printf "[ ! -f /workspace/humble_ws/install/setup.bash ] || source /workspace/humble_ws/install/setup.bash && "
    printf "%s; " "$launch_cmd"
    printf 'status=$?; echo; echo "[prepare_map_tmux] command exited with status $status"; exec bash'
}

node_exists() {
    local node="$1"
    printf '%s\n' "$ROS_NODES" | grep -qx "$node"
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --session)
            if [ "$#" -lt 2 ]; then
                echo "--session requires a name." >&2
                exit 2
            fi
            SESSION="$2"
            shift 2
            ;;
        --no-attach)
            ATTACH=false
            shift
            ;;
        --dry-run)
            DRY_RUN=true
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        --)
            shift
            break
            ;;
        -*)
            echo "Unknown option: $1" >&2
            usage >&2
            exit 2
            ;;
        *)
            break
            ;;
    esac
done

for arg in "$@"; do
    if [[ "$arg" != *:=* ]]; then
        echo "Arguments must use ROS syntax name:=value: $arg" >&2
        exit 2
    fi

    name="${arg%%:=*}"
    value="${arg#*:=}"
    case "$name" in
        output)
            OUTPUT="$value"
            ;;
        use_rviz)
            USE_RVIZ="$value"
            ;;
        dlio_output)
            DLIO_OUTPUT="$value"
            ;;
        save_leaf_size)
            SAVE_LEAF_SIZE="$value"
            ;;
        web_ui)
            WEB_UI="$value"
            ;;
        web_ui_port)
            WEB_UI_PORT="$value"
            ;;
        rosbridge_port)
            ROSBRIDGE_PORT="$value"
            ;;
        *)
            echo "Unknown argument: $name" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if [ -z "$OUTPUT" ]; then
    echo "output:= is required (e.g. output:=/external/maps/office)." >&2
    usage >&2
    exit 2
fi

if [[ "$OUTPUT" != /* ]]; then
    echo "output:= must be an absolute path: $OUTPUT" >&2
    exit 2
fi

if [[ "$OUTPUT" == *\"* ]]; then
    echo 'output:= cannot contain a double quote.' >&2
    exit 2
fi

if ! [[ "$SAVE_LEAF_SIZE" =~ ^[0-9]+([.][0-9]+)?$ ]] ||
    [[ "$SAVE_LEAF_SIZE" =~ ^0+([.]0+)?$ ]]; then
    echo "save_leaf_size:= must be a positive decimal number: $SAVE_LEAF_SIZE" >&2
    exit 2
fi

if [ "$WEB_UI" != "true" ] && [ "$WEB_UI" != "false" ]; then
    echo "web_ui:= must be true or false: $WEB_UI" >&2
    exit 2
fi

if [ -e "$OUTPUT" ]; then
    echo "Output already exists; refusing to overwrite: $OUTPUT" >&2
    exit 1
fi

if [ "$DRY_RUN" = false ]; then
    if tmux has-session -t "$SESSION" 2>/dev/null; then
        echo "tmux session '$SESSION' already exists. Attach with: tmux attach -t $SESSION" >&2
        exit 1
    fi

    if ! ROS_NODES="$(ros2 node list --no-daemon --spin-time 3)"; then
        echo "Could not inspect the ROS graph; refusing to start collection." >&2
        exit 1
    fi

    if ! node_exists "/go2w_teleop_gamepad_node"; then
        echo "Required external teleop node is not visible: /go2w_teleop_gamepad_node" >&2
        echo "Start go2w_teleop_gamepad on the same ROS domain, then retry." >&2
        exit 1
    fi

    CONFLICTING_NODES=(
        /velocity_bridge
        /controller_server
        /bt_navigator
        /goal_pose_executor
        /hesai_node
        /go2w_imu_publisher
        /dlio_odom_node
        /dlio_map_node
    )
    for node in "${CONFLICTING_NODES[@]}"; do
        if node_exists "$node"; then
            echo "Conflicting node is already running; refusing to start collection: $node" >&2
            exit 1
        fi
    done
fi

COLLECT_CMD=(
    ros2 launch nav_goal_go2w_bringup prepare_map.launch.py
    "use_rviz:=${USE_RVIZ}"
    "dlio_output:=${DLIO_OUTPUT}"
)
if [ "$WEB_UI" = "true" ]; then
    COLLECT_CMD+=("web_ui:=true" "output:=$OUTPUT" "save_leaf_size:=$SAVE_LEAF_SIZE" "web_ui_port:=$WEB_UI_PORT" "rosbridge_port:=$ROSBRIDGE_PORT")
fi
FINISH_CMD=(
    bash /external/scripts/finish_prepare_map.sh
    --output "$OUTPUT"
    --session "$SESSION"
    --save-leaf-size "$SAVE_LEAF_SIZE"
)

COLLECT_LAUNCH="$(shell_join "${COLLECT_CMD[@]}")"
FINISH_LAUNCH="$(shell_join "${FINISH_CMD[@]}")"
COLLECT_WINDOW_CMD="$(window_shell "$COLLECT_LAUNCH")"
FINISH_WINDOW_CMD="$(window_shell "$FINISH_LAUNCH")"

if [ "$DRY_RUN" = true ]; then
    echo "tmux new-session -d -s $(printf "%q" "$SESSION") -n collect $(printf "%q" "bash -lc $COLLECT_WINDOW_CMD")"
    echo "tmux new-window -t $(printf "%q" "$SESSION") -n finish $(printf "%q" "bash -lc $FINISH_WINDOW_CMD")"
    if [ "$ATTACH" = true ]; then
        echo "tmux attach -t $(printf "%q" "$SESSION")"
    fi
    exit 0
fi

tmux new-session -d -s "$SESSION" -n collect "bash -lc $(printf "%q" "$COLLECT_WINDOW_CMD")"
tmux new-window -t "$SESSION" -n finish "bash -lc $(printf "%q" "$FINISH_WINDOW_CMD")"

if [ "$ATTACH" = true ]; then
    tmux attach -t "$SESSION"
else
    echo "Created tmux session '$SESSION'. Attach with: tmux attach -t $SESSION"
fi
