#!/bin/bash
# P60b 启动脚本 —— 带 GUI 的完整演示(便于观察路径可视化)
set +u
export DISPLAY=:0
export QT_SCALE_FACTOR=1 QT_AUTO_SCREEN_SCALE_FACTOR=0 GDK_SCALE=1
source /opt/ros/jazzy/setup.bash
source /home/earl/workspace/VLA/vla_ws/install/setup.bash
cd /home/earl/workspace/VLA || exit 1
rm -f /home/earl/workspace/VLA/p60_run.log
setsid nohup ros2 launch vla_follow demo.launch.py auto_voice:=false \
  > /home/earl/workspace/VLA/p60_run.log 2>&1 < /dev/null &
echo LAUNCHED
