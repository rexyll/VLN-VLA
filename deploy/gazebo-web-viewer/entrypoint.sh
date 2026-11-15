#!/usr/bin/env bash
# Public browser viewer for the real Gazebo Harmonic GUI.
set -Eeuo pipefail

: "${VNC_PASSWORD:?Set a strong VNC_PASSWORD in the hosting service's secret environment variables.}"
if [[ "${#VNC_PASSWORD}" -lt 12 ]]; then
  echo "VNC_PASSWORD must contain at least 12 characters." >&2
  exit 2
fi

export VLA_HOME=/opt/VLN-VLA
export DISPLAY=:0
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1
export GZ_SIM_RENDER_ENGINE=ogre
export PW_AUTO_DRINK="${PW_AUTO_DRINK:-1}"
export PW_DRINK_FIRST="${PW_DRINK_FIRST:-45}"
export PW_DRINK_GAP_MIN="${PW_DRINK_GAP_MIN:-120}"
export PW_DRINK_GAP_MAX="${PW_DRINK_GAP_MAX:-180}"

mkdir -p "$VLA_HOME/vla_ws/logs" /tmp/.X11-unix
Xvfb :0 -screen 0 "${SCREEN_RESOLUTION:-1600x900x24}" -ac +extension GLX +render -noreset \
  > "$VLA_HOME/vla_ws/logs/xvfb.log" 2>&1 &
XVFB_PID=$!
for _ in $(seq 1 30); do
  [[ -S /tmp/.X11-unix/X0 ]] && break
  sleep 1
done
[[ -S /tmp/.X11-unix/X0 ]] || { echo "Xvfb failed to start"; exit 1; }

openbox > "$VLA_HOME/vla_ws/logs/openbox.log" 2>&1 &
x11vnc -storepasswd "$VNC_PASSWORD" /tmp/vnc.pass >/dev/null
x11vnc -display :0 -rfbauth /tmp/vnc.pass -forever -shared -rfbport 5900 \
  -noxdamage -repeat > "$VLA_HOME/vla_ws/logs/x11vnc.log" 2>&1 &
websockify --web=/usr/share/novnc 6080 localhost:5900 \
  > "$VLA_HOME/vla_ws/logs/novnc.log" 2>&1 &
VNC_PID=$!

cleanup() {
  set +e
  bash "$VLA_HOME/tools/killall_demo.sh"
  kill "$VNC_PID" "$XVFB_PID" 2>/dev/null
}
trap cleanup EXIT TERM INT

source /opt/ros/jazzy/setup.bash
source "$VLA_HOME/vla_ws/install/setup.bash"

echo "Starting Gazebo work_area with GUI..."
bash "$VLA_HOME/tools/start_scene.sh" --gui
echo "Starting DrinkTask and follower stack..."
NAV2=1 bash "$VLA_HOME/tools/start_drink_demo.sh"

echo "Gazebo browser viewer is ready on port 6080."
echo "Open /vnc.html?autoconnect=true&resize=remote and enter the VNC password."

# Keep the service alive while monitoring the simulator and its GUI.
while kill -0 "$XVFB_PID" 2>/dev/null; do
  if ! pgrep -af 'gz sim' >/dev/null; then
    echo "Gazebo process exited; stopping viewer service." >&2
    exit 1
  fi
  sleep 10
done
