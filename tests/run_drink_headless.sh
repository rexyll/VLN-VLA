#!/usr/bin/env bash
# run_drink_headless.sh [rounds=1] [tag] [cooldown_seconds=0]
#
# Start a persistent Xvfb display, the complete Gazebo work_area world,
# ROS-Gazebo bridge and DrinkTask demo nodes, then run the real observer-based
# DrinkTask acceptance batch. Xvfb must outlive gz sim: start_scene.sh launches
# Gazebo in the background, so wrapping that script in xvfb-run would terminate
# the X server as soon as the launcher shell exits.
#
# Prerequisites: ROS 2 Jazzy, Gazebo Harmonic, ros_gz_bridge, Nav2, colcon-built
# vla_follow workspace, Xvfb and xdpyinfo. This script does not install deps.
#
# This is a real simulation runner, not a mock or a substitute for the
# acoustic-ASR path: person_walk emits text:我要喝水 through voice_node's
# explicit text-input route, then the normal /robot/command -> drink_task path.

set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
export VLA_HOME="$ROOT"
export DISPLAY="${VLA_HEADLESS_DISPLAY:-:99}"
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1

LOG_DIR="$ROOT/vla_ws/logs"
mkdir -p "$LOG_DIR"

N="${1:-1}"
TAG="${2:-headless_$(date -u +%Y%m%dT%H%M%SZ)}"
COOLDOWN="${3:-0}"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo "ROS 2 Jazzy setup not found: /opt/ros/jazzy/setup.bash" >&2
    exit 2
fi
if [[ ! -f "$ROOT/vla_ws/install/setup.bash" ]]; then
    echo "Workspace overlay missing; build vla_ws before running this test." >&2
    exit 2
fi
for command in Xvfb xdpyinfo gz ros2 colcon; do
    if ! command -v "$command" >/dev/null 2>&1; then
        echo "Required command not found: $command" >&2
        exit 2
    fi
done

source /opt/ros/jazzy/setup.bash
source "$ROOT/vla_ws/install/setup.bash"

XVFB_PID=""
cleanup() {
    status=$?
    trap - EXIT
    if (( status != 0 )); then
        echo "DrinkTask headless run failed (exit=$status); recent logs:" >&2
        for file in gzsim bridge pw voice drink percept vision mux follower nav2; do
            if [[ -s "$LOG_DIR/$file.log" ]]; then
                echo "----- $file.log -----" >&2
                tail -n 100 "$LOG_DIR/$file.log" >&2 || true
            fi
        done
    fi
    bash "$ROOT/tools/killall_demo.sh" || true
    if [[ -n "$XVFB_PID" ]]; then
        kill "$XVFB_PID" 2>/dev/null || true
        wait "$XVFB_PID" 2>/dev/null || true
    fi
    exit "$status"
}
trap cleanup EXIT INT TERM

# Keep the display alive for the entire Gazebo server lifetime. Do not use
# xvfb-run here: start_scene.sh deliberately backgrounds gz sim and returns.
if xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
    echo "Refusing to reuse an already-active display: $DISPLAY" >&2
    exit 2
fi
Xvfb "$DISPLAY" -screen 0 1280x1024x24 -nolisten tcp -ac \
    > "$LOG_DIR/xvfb.log" 2>&1 &
XVFB_PID=$!
display_ready=0
for _ in $(seq 1 30); do
    if xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
        display_ready=1
        break
    fi
    if ! kill -0 "$XVFB_PID" 2>/dev/null; then
        cat "$LOG_DIR/xvfb.log" >&2 || true
        exit 1
    fi
    sleep 1
done
if (( display_ready != 1 )); then
    echo "Xvfb did not become ready on $DISPLAY" >&2
    cat "$LOG_DIR/xvfb.log" >&2 || true
    exit 1
fi

echo "[1/5] Starting complete work_area Gazebo world and ROS-Gazebo bridge"
bash "$ROOT/tools/start_scene.sh"

wait_gz_topic() {
    local topic="$1"
    local timeout_s="$2"
    for _ in $(seq 1 "$timeout_s"); do
        if gz topic -l 2>/dev/null | grep -Fxq "$topic"; then
            echo "Gazebo topic ready: $topic"
            return 0
        fi
        sleep 1
    done
    echo "Timed out waiting for Gazebo topic: $topic" >&2
    tail -n 160 "$LOG_DIR/gzsim.log" >&2 || true
    tail -n 100 "$LOG_DIR/bridge.log" >&2 || true
    return 1
}

wait_ros_topic() {
    local topic="$1"
    local timeout_s="$2"
    for _ in $(seq 1 "$timeout_s"); do
        if ros2 topic list 2>/dev/null | grep -Fxq "$topic"; then
            echo "ROS topic ready: $topic"
            return 0
        fi
        sleep 1
    done
    echo "Timed out waiting for ROS topic: $topic" >&2
    return 1
}

wait_gz_topic "/world/follow_world/dynamic_pose/info" 120
wait_gz_topic "/drink/carry/ack" 120
wait_ros_topic "/clock" 60
wait_ros_topic "/drink_cam/image" 120

echo "[2/5] Starting full DrinkTask node set (Nav2 enabled; random auto-drink off)"
PW_AUTO_DRINK=0 NAV2=1 bash "$ROOT/tools/start_drink_demo.sh"

echo "[3/5] Waiting for required ROS nodes and Nav2 action server"
required_nodes=(/person_walk /voice_node /drink_task /follower /cmd_mux)
for node in "${required_nodes[@]}"; do
    ready=0
    for _ in $(seq 1 90); do
        if ros2 node list 2>/dev/null | grep -Fxq "$node"; then
            ready=1
            break
        fi
        sleep 1
    done
    if (( ready != 1 )); then
        echo "Required ROS node did not appear: $node" >&2
        ros2 node list >&2 || true
        exit 1
    fi
done
action_ready=0
for _ in $(seq 1 120); do
    if ros2 action list 2>/dev/null | grep -Fxq "/navigate_to_pose"; then
        action_ready=1
        break
    fi
    sleep 1
done
if (( action_ready != 1 )); then
    echo "Nav2 /navigate_to_pose action server did not become ready" >&2
    ros2 action list >&2 || true
    exit 1
fi

echo "[4/5] Running complete observer acceptance through drink_repeat.sh"
echo "Rounds=$N tag=$TAG cooldown=$COOLDOWN"
bash "$ROOT/tests/drink_repeat.sh" "$N" "$TAG" "$COOLDOWN"

echo "[5/5] DrinkTask headless batch passed"
echo "Evidence: $ROOT/tests/out/drink_repeat_$TAG.json"
