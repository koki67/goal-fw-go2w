#!/bin/bash
# Start the Go2W goal navigation bringup in tmux with D-LIO isolated in its
# own window and a health window echoing localization state.
set -euo pipefail

SESSION="go2w"
ATTACH=true
DRY_RUN=false

VALID_ARGS=(
    map
    use_sim_time
    use_rviz
    bridge_dry_run
    vx_max
    vy_max
    wz_max
    goal_update_strategy
    require_localization
    registration_rate
    localization_params_file
    dlio_output
    enable_map_viz_layers
    record_results
    record_bag_dir
    record_bag_prefix
    record_storage
)

usage() {
    cat <<'USAGE'
Usage:
  scripts/bringup_tmux.sh [options] map:=/external/maps/<name> [launch_arg:=value ...]

Options:
  --session NAME   tmux session name (default: go2w)
  --no-attach      create the session without attaching
  --dry-run        print the tmux commands without running them
  -h, --help       show this help

Examples:
  scripts/bringup_tmux.sh map:=/external/maps/office
  scripts/bringup_tmux.sh map:=/external/maps/office bridge_dry_run:=false vx_max:=0.2
  scripts/bringup_tmux.sh --session go2w_debug map:=/external/maps/office use_rviz:=true

The script always runs main bringup with with_dlio:=false because D-LIO is
started in the separate dlio tmux window. After attach: set the initial pose
with RViz "2D Pose Estimate", wait for TRACKING in the health window, then
click "2D Nav Goal".
USAGE
}

is_valid_arg() {
    local candidate="$1"
    local valid
    for valid in "${VALID_ARGS[@]}"; do
        if [ "$candidate" = "$valid" ]; then
            return 0
        fi
    done
    return 1
}

valid_args_csv() {
    local IFS=","
    printf "%s" "${VALID_ARGS[*]}"
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
    printf 'status=$?; echo; echo "[bringup_tmux] command exited with status $status"; exec bash'
}

BRINGUP_ARGS=()
MAP_ARG=""
DLIO_USE_SIM_TIME="false"
DLIO_OUTPUT="screen"

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
        echo "Launch arguments must use ROS syntax name:=value: $arg" >&2
        exit 2
    fi

    name="${arg%%:=*}"
    value="${arg#*:=}"

    if [ -z "$name" ]; then
        echo "Launch argument name cannot be empty: $arg" >&2
        exit 2
    fi

    if [ "$name" = "with_dlio" ]; then
        echo "with_dlio is managed by this script and is always passed as with_dlio:=false." >&2
        exit 2
    fi

    if ! is_valid_arg "$name"; then
        echo "Unknown launch argument: $name" >&2
        echo "Valid arguments are: $(valid_args_csv)" >&2
        exit 2
    fi

    BRINGUP_ARGS+=("$arg")

    if [ "$name" = "map" ]; then
        MAP_ARG="$value"
    elif [ "$name" = "use_sim_time" ]; then
        DLIO_USE_SIM_TIME="$value"
    elif [ "$name" = "dlio_output" ]; then
        DLIO_OUTPUT="$value"
    fi
done

if [ -z "$MAP_ARG" ]; then
    echo "map:= is required (e.g. map:=/external/maps/office)." >&2
    usage >&2
    exit 2
fi

if [ "$DRY_RUN" = false ] && tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "tmux session '$SESSION' already exists. Attach with: tmux attach -t $SESSION" >&2
    exit 1
fi

DLIO_CMD=(
    ros2 launch direct_lidar_inertial_odometry dlio.launch.py
    "use_sim_time:=${DLIO_USE_SIM_TIME}"
    "dlio_output:=${DLIO_OUTPUT}"
)

BRINGUP_CMD=(
    ros2 launch nav_goal_go2w_bringup bringup.launch.py
    with_dlio:=false
    "${BRINGUP_ARGS[@]}"
)

HEALTH_CMD=(
    ros2 topic echo /localization/state
)

DLIO_LAUNCH="$(shell_join "${DLIO_CMD[@]}")"
BRINGUP_LAUNCH="$(shell_join "${BRINGUP_CMD[@]}")"
HEALTH_LAUNCH="$(shell_join "${HEALTH_CMD[@]}")"
DLIO_WINDOW_CMD="$(window_shell "$DLIO_LAUNCH")"
BRINGUP_WINDOW_CMD="$(window_shell "$BRINGUP_LAUNCH")"
HEALTH_WINDOW_CMD="$(window_shell "$HEALTH_LAUNCH")"

if [ "$DRY_RUN" = true ]; then
    echo "tmux new-session -d -s $(printf "%q" "$SESSION") -n dlio $(printf "%q" "bash -lc $DLIO_WINDOW_CMD")"
    echo "tmux new-window -t $(printf "%q" "$SESSION") -n bringup $(printf "%q" "bash -lc $BRINGUP_WINDOW_CMD")"
    echo "tmux new-window -t $(printf "%q" "$SESSION") -n health $(printf "%q" "bash -lc $HEALTH_WINDOW_CMD")"
    if [ "$ATTACH" = true ]; then
        echo "tmux attach -t $(printf "%q" "$SESSION")"
    fi
    exit 0
fi

tmux new-session -d -s "$SESSION" -n dlio "bash -lc $(printf "%q" "$DLIO_WINDOW_CMD")"
tmux new-window -t "$SESSION" -n bringup "bash -lc $(printf "%q" "$BRINGUP_WINDOW_CMD")"
tmux new-window -t "$SESSION" -n health "bash -lc $(printf "%q" "$HEALTH_WINDOW_CMD")"

if [ "$ATTACH" = true ]; then
    tmux attach -t "$SESSION"
else
    echo "Created tmux session '$SESSION'. Attach with: tmux attach -t $SESSION"
fi
