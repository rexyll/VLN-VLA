#!/bin/bash
# tmp_show.sh —— 启动带 GUI 的 Gazebo 演示（供人工观察轨迹）
set +u
export DISPLAY=${DISPLAY:-:0}
# HiDPI 下 ogre1 的 3D 视口只渲染在窗口左上角一小块(Qt 按 2x 缩放,
# 视口尺寸没跟着缩), 强制 1.0 让视口铺满
export QT_SCALE_FACTOR=1 QT_AUTO_SCREEN_SCALE_FACTOR=0 GDK_SCALE=1
source /opt/ros/jazzy/setup.bash
source /home/earl/workspace/VLA/vla_ws/install/setup.bash
cd /home/earl/workspace/VLA || exit 1

pkill -f 'gz si[m]'
pkill -f 'vla_follo[w]/lib/vla_follow'
pkill -f 'ros2 launc[h]'
pkill -f 'tmp_traj_vi[z]'
# P45: 原来**不清理 parameter_bridge**, 每次重启泄漏一个 —— 实测积压了
# 40+ 个僵尸 bridge, 同时重复桥接 /robot/cmd_vel 与 /robot/odom,
# 速度指令和测速一起被污染。必须一起清。
pkill -f 'ros_gz_bridge/parameter_bridg[e]'
pkill -f 'parameter_bridge --ros[-]args'
sleep 4

setsid nohup ros2 launch vla_follow demo.launch.py auto_voice:=false \
  > /home/earl/workspace/VLA/demo_show.log 2>&1 < /dev/null &
echo "[SHOW] launch 已启动"
