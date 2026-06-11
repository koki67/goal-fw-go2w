#!/bin/bash
# Save the active D-LIO map, prepare navigation artifacts, and publish them
# atomically at the requested output directory.
set -euo pipefail

OUTPUT=""
SESSION=""
SAVE_LEAF_SIZE="0.05"
SAVE_SERVICE="/dlio_map_node/save_pcd"

usage() {
    cat <<'USAGE'
Usage:
  scripts/finish_prepare_map.sh --output DIR --session NAME [--save-leaf-size METERS]

This helper is normally started by scripts/prepare_map_tmux.sh. Wait until
the environment has been covered, then press Enter to save and convert the
active D-LIO map.
USAGE
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --output)
            if [ "$#" -lt 2 ]; then
                echo "--output requires a value." >&2
                exit 2
            fi
            OUTPUT="${2:-}"
            shift 2
            ;;
        --session)
            if [ "$#" -lt 2 ]; then
                echo "--session requires a value." >&2
                exit 2
            fi
            SESSION="${2:-}"
            shift 2
            ;;
        --save-leaf-size)
            if [ "$#" -lt 2 ]; then
                echo "--save-leaf-size requires a value." >&2
                exit 2
            fi
            SAVE_LEAF_SIZE="${2:-}"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if [ -z "$OUTPUT" ] || [ -z "$SESSION" ]; then
    echo "--output and --session are required." >&2
    usage >&2
    exit 2
fi

if [[ "$OUTPUT" != /* ]]; then
    echo "--output must be an absolute path: $OUTPUT" >&2
    exit 2
fi

if [[ "$OUTPUT" == *\"* ]]; then
    echo "--output cannot contain a double quote." >&2
    exit 2
fi

if ! [[ "$SAVE_LEAF_SIZE" =~ ^[0-9]+([.][0-9]+)?$ ]] ||
    [[ "$SAVE_LEAF_SIZE" =~ ^0+([.]0+)?$ ]]; then
    echo "--save-leaf-size must be a positive decimal number: $SAVE_LEAF_SIZE" >&2
    exit 2
fi

if [ -e "$OUTPUT" ]; then
    echo "Output already exists; refusing to overwrite: $OUTPUT" >&2
    exit 1
fi

echo "Waiting for D-LIO map save service: $SAVE_SERVICE"
until ros2 service list 2>/dev/null | grep -qx "$SAVE_SERVICE"; do
    sleep 1
done

cat <<EOF

Map collection is ready.

Drive the Go2W with the separately managed gamepad system. Cover the complete
environment slowly and revisit loop closures where practical.

Output: $OUTPUT

Press Enter when collection is complete. This will save D-LIO, stop sensor
collection, convert the raw cloud, and retain it as raw/dlio_map.pcd.
EOF
read -r

if [ -e "$OUTPUT" ]; then
    echo "Output appeared during collection; refusing to overwrite: $OUTPUT" >&2
    exit 1
fi

OUTPUT_PARENT="$(dirname "$OUTPUT")"
OUTPUT_NAME="$(basename "$OUTPUT")"
mkdir -p "$OUTPUT_PARENT"
STAGING_ROOT="$(mktemp -d "$OUTPUT_PARENT/.${OUTPUT_NAME}.prepare-map.XXXXXX")"
STAGING_DIR="$STAGING_ROOT/$OUTPUT_NAME"
RAW_DIR="$STAGING_DIR/raw"
RAW_PCD="$RAW_DIR/dlio_map.pcd"
mkdir -p "$RAW_DIR"

echo "Saving D-LIO map to staging directory: $RAW_DIR"
SAVE_RESPONSE="$(
    ros2 service call \
        "$SAVE_SERVICE" \
        direct_lidar_inertial_odometry/srv/SavePCD \
        "{leaf_size: $SAVE_LEAF_SIZE, save_path: \"$RAW_DIR\"}"
)"
printf '%s\n' "$SAVE_RESPONSE"

if ! printf '%s\n' "$SAVE_RESPONSE" |
    grep -Eq 'success[=:][[:space:]]*(True|true)'; then
    echo "D-LIO reported a failed save. Staging data retained at: $STAGING_DIR" >&2
    exit 1
fi

if [ ! -s "$RAW_PCD" ]; then
    echo "D-LIO reported success but did not create a non-empty $RAW_PCD." >&2
    echo "Staging data retained at: $STAGING_DIR" >&2
    exit 1
fi

echo "Stopping the collection launch."
tmux send-keys -t "${SESSION}:collect" C-c
sleep 2

echo "Preparing navigation map artifacts."
if ! ros2 run nav_goal_go2w_map prepare_map \
    --input "$RAW_PCD" \
    --output "$STAGING_DIR"; then
    echo "Map conversion failed. Raw data retained at: $RAW_PCD" >&2
    exit 1
fi

if [ -e "$OUTPUT" ]; then
    echo "Output appeared during conversion; refusing to overwrite: $OUTPUT" >&2
    echo "Prepared staging data retained at: $STAGING_DIR" >&2
    exit 1
fi

mv "$STAGING_DIR" "$OUTPUT"
rmdir "$STAGING_ROOT"

cat <<EOF

Prepared map written to: $OUTPUT
Raw D-LIO cloud retained at: $OUTPUT/raw/dlio_map.pcd

Stop the external teleop system before using bridge_dry_run:=false. It must
not publish /api/sport/request concurrently with the navigation velocity bridge.

Continue with goal navigation:
  bash /external/scripts/bringup_tmux.sh map:=$OUTPUT
EOF
