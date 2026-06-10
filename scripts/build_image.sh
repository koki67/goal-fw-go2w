#!/bin/bash
# Build the nav-goal-go2w docker image (ARM64 robot target).
# On an x86 dev host this requires QEMU + buildx for cross-build; on the
# Jetson itself a plain `docker build` is sufficient.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

CUDA="false"
while [ "$#" -gt 0 ]; do
    case "$1" in
        --cuda)
            CUDA="true"
            shift
            ;;
        -h|--help)
            echo "Usage: bash scripts/build_image.sh [--cuda]"
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            exit 2
            ;;
    esac
done

if [ "$CUDA" = "true" ]; then
    IMAGE="${NAV_GOAL_IMAGE:-${NAV_GOAL_CUDA_IMAGE:-nav-goal-go2w:cuda}}"
    DOCKERFILE="$REPO_ROOT/docker/Dockerfile.cuda"
else
    IMAGE="${NAV_GOAL_IMAGE:-nav-goal-go2w:latest}"
    DOCKERFILE="$REPO_ROOT/docker/Dockerfile"
fi
PLATFORM="${NAV_GOAL_PLATFORM:-linux/arm64}"

cd "$REPO_ROOT"

if docker buildx version >/dev/null 2>&1; then
    if [ "$PLATFORM" = "linux/arm64" ] && [ "$(uname -m)" != "aarch64" ] &&
       [ ! -e /proc/sys/fs/binfmt_misc/qemu-aarch64 ]; then
        cat >&2 <<EOF
ARM64 cross-build requires QEMU/binfmt on this host.
Install it with:
  docker run --privileged --rm tonistiigi/binfmt --install arm64

Or build on the Go2W/aarch64 target directly. For an amd64 Dockerfile smoke
build on this host, run:
  NAV_GOAL_PLATFORM=linux/amd64 bash scripts/build_image.sh
EOF
        exit 1
    fi

    echo "Building $IMAGE for $PLATFORM with buildx..."
    docker buildx build \
        --platform "$PLATFORM" \
        --load \
        -f "$DOCKERFILE" \
        -t "$IMAGE" \
        .
else
    echo "Building $IMAGE natively..."
    docker build \
        -f "$DOCKERFILE" \
        -t "$IMAGE" \
        .
fi
