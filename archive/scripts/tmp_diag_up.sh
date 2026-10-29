#!/bin/bash
# tmp_diag_up.sh —— 诊断专用最小启动集:
#   gz sim + depth_percept + person_vision
# 不起 person_walk / follower, 使人与机器人的位姿完全由脚本控制。
set +u
export DISPLAY=${DISPLAY:-:0}
export QT_SCALE_FACTOR=1 QT_AUTO_SCREEN_SCALE_FACTOR=0 GDK_SCALE=1
source /opt/ros/jazzy/setup.bash
source /home/earl/workspace/VLA/vla_ws/install/setup.bash
cd /home/earl/workspace/VLA || exit 1

pkill -f 'gz si[m]'
pkill -f 'ros2 launc[h]'
pkill -f 'vla_follow/(follower|person_walk|depth_percept|person_vision|voice_node)' 2>/dev/null
sleep 4

WORLD=$(ros2 pkg prefix vla_follow)/share/vla_follow/worlds/work_area.sdf
setsid nohup gz sim -r -v 2 --render-engine ogre "$WORLD" \
  > /home/earl/workspace/VLA/diag_sim.log 2>&1 < /dev/null &
echo "[DIAG] gz sim 启动"
sleep 22

setsid nohup ros2 run vla_follow depth_percept \
  > /home/earl/workspace/VLA/diag_dp.log 2>&1 < /dev/null &
echo "[DIAG] depth_percept 启动"
sleep 3
setsid nohup ros2 run vla_follow person_vision \
  > /home/earl/workspace/VLA/diag_vis.log 2>&1 < /dev/null &
echo "[DIAG] person_vision 启动"
sleep 4
echo "[DIAG] 就绪"
