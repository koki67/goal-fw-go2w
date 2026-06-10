#!/bin/bash
# Open an extra shell inside an already-running nav-goal-go2w container.
# Use docker/run.sh first to start the primary container.
set -e

CONTAINER="${NAV_GOAL_CONTAINER_NAME:-nav-goal-go2w}"

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "Container '$CONTAINER' is not running. Start it with docker/run.sh first." >&2
    exit 1
fi

exec docker exec -it "$CONTAINER" bash
