#!/usr/bin/env bash
# Expose the existing Gazebo desktop through noVNC on this same simulation host.
set -Eeuo pipefail
export DISPLAY="${DISPLAY:-:0}"
LISTEN_ADDRESS="${LISTEN_ADDRESS:-127.0.0.1}"
WEB_PORT="${WEB_PORT:-6080}"
VNC_PORT="${VNC_PORT:-5900}"
PID_DIR="${XDG_RUNTIME_DIR:-/tmp}/vla-gazebo-viewer"
mkdir -p "$PID_DIR"

for bin in x11vnc websockify; do
  command -v "$bin" >/dev/null || {
    echo "Missing $bin. Install with: sudo apt-get install x11vnc novnc"
    exit 1
  }
done
[[ -S "/tmp/.X11-unix/X${DISPLAY#:}" ]] || {
  echo "No X11 display at $DISPLAY. Start the Gazebo desktop/GUI first."
  exit 1
}
[[ -n "${VNC_PASSWORD:-}" ]] || {
  echo "Set VNC_PASSWORD before starting (at least 12 characters)."
  exit 2
}
[[ "${#VNC_PASSWORD}" -ge 12 ]] || {
  echo "VNC_PASSWORD must contain at least 12 characters."
  exit 2
}
if [[ -f "$PID_DIR/x11vnc.pid" ]] && kill -0 "$(cat "$PID_DIR/x11vnc.pid")" 2>/dev/null; then
  echo "Viewer already running. Use tools/stop_gazebo_web_viewer.sh first."
  exit 1
fi

x11vnc -storepasswd "$VNC_PASSWORD" "$PID_DIR/passwd" >/dev/null
chmod 600 "$PID_DIR/passwd"
x11vnc -display "$DISPLAY" -rfbauth "$PID_DIR/passwd" -forever -shared \
  -rfbport "$VNC_PORT" -listen "$LISTEN_ADDRESS" -noxdamage -repeat \
  -o "$PID_DIR/x11vnc.log" &
echo $! > "$PID_DIR/x11vnc.pid"

websockify --web=/usr/share/novnc --heartbeat=30 \
  "$LISTEN_ADDRESS:$WEB_PORT" "127.0.0.1:$VNC_PORT" \
  > "$PID_DIR/websockify.log" 2>&1 &
echo $! > "$PID_DIR/websockify.pid"

sleep 2
if ! kill -0 "$(cat "$PID_DIR/x11vnc.pid")" 2>/dev/null || \
   ! kill -0 "$(cat "$PID_DIR/websockify.pid")" 2>/dev/null; then
  echo "Viewer failed to start; inspect $PID_DIR/*.log"
  exit 1
fi

echo "Gazebo browser viewer started."
echo "On this host: http://127.0.0.1:$WEB_PORT/vnc.html?autoconnect=true"
if [[ "$LISTEN_ADDRESS" == "0.0.0.0" ]]; then
  IP=$(hostname -I | awk '{print $1}')
  echo "On LAN: http://$IP:$WEB_PORT/vnc.html?autoconnect=true"
  echo "Only use on a trusted LAN; VNC password is required."
else
  echo "Bound to localhost only. For trusted LAN access, set LISTEN_ADDRESS=0.0.0.0."
fi
echo "VNC password is required in the browser. Never share it in chat or commit it."
