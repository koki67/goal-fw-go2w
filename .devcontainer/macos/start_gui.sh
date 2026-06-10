#!/usr/bin/env bash
# Start the in-container GUI stack used by the macOS devcontainer profile.
# Renders ROS GUIs (RViz, rqt, ...) into an Xvfb framebuffer with Mesa llvmpipe
# software OpenGL, then exposes the desktop to the Mac browser via noVNC.
#
# Idempotent: re-running this script while the stack is already up is a no-op.
# Safe to call from postStartCommand.

set -eu

DISPLAY_NUM="${DISPLAY_NUM:-1}"
SCREEN_GEOMETRY="${SCREEN_GEOMETRY:-1600x1000x24}"
VNC_PORT="${VNC_PORT:-5900}"
NOVNC_PORT="${NOVNC_PORT:-6080}"
LOG_DIR="${LOG_DIR:-/tmp/macos-gui}"

mkdir -p "${LOG_DIR}"

start_bg() {
    local name="$1"
    shift
    if pgrep -f "$1" >/dev/null 2>&1; then
        return 0
    fi
    nohup "$@" >"${LOG_DIR}/${name}.log" 2>&1 &
    disown
}

start_bg xvfb \
    Xvfb ":${DISPLAY_NUM}" -screen 0 "${SCREEN_GEOMETRY}" -ac +extension GLX +render -noreset

# Wait for Xvfb to be ready before launching anything that needs DISPLAY.
for _ in $(seq 1 30); do
    if DISPLAY=":${DISPLAY_NUM}" xdpyinfo >/dev/null 2>&1; then
        break
    fi
    sleep 0.1
done

DISPLAY=":${DISPLAY_NUM}" start_bg fluxbox fluxbox

start_bg x11vnc \
    x11vnc -display ":${DISPLAY_NUM}" -forever -shared -nopw -rfbport "${VNC_PORT}" -quiet -bg -o "${LOG_DIR}/x11vnc.log"

NOVNC_WEB_ROOT="${NOVNC_WEB_ROOT:-/opt/novnc}"
start_bg websockify \
    websockify --web="${NOVNC_WEB_ROOT}" "${NOVNC_PORT}" "localhost:${VNC_PORT}"

cat <<EOF
[macos-gui] Xvfb on :${DISPLAY_NUM}, x11vnc on :${VNC_PORT}, noVNC on http://localhost:${NOVNC_PORT}/vnc.html
[macos-gui] Logs: ${LOG_DIR}
EOF
