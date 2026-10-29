#!/bin/bash
# tmp_restart.sh —— 一键重置演示环境 (detached 运行)
# 用法: tmp_restart.sh [keep_nodes]
#   keep_nodes=1 → 只保留 gz sim + parameter_bridge (纯物理/指令调试)
export VLA_HOME=/home/earl/workspace/VLA
export DISPLAY=:0
LOG=$VLA_HOME/logs/demo2.log

pkill -f 'ros2 launch vla_follow' 2>/dev/null
sleep 1
pkill -f 'vla_follow/lib/vla_follow' 2>/dev/null
pkill -f 'parameter_bridge' 2>/dev/null
pkill -f 'gz sim' 2>/dev/null
pkill -f 'gz-sim' 2>/dev/null
sleep 2
pkill -9 -f 'gz sim' 2>/dev/null
pkill -9 -f 'vla_follow/lib/vla_follow' 2>/dev/null
sleep 1

source /opt/ros/jazzy/setup.bash
source $VLA_HOME/vla_ws/install/setup.bash
cd $VLA_HOME
setsid nohup ros2 launch vla_follow demo.launch.py auto_voice:=false \
    > $LOG 2>&1 < /dev/null &
echo "LAUNCHED"
sleep 20

if [ "$1" = "keep_nodes" ]; then
    pkill -f 'vla_follow/lib/vla_follow/person_walk' 2>/dev/null
    pkill -f 'vla_follow/lib/vla_follow/person_vision' 2>/dev/null
    pkill -f 'vla_follow/lib/vla_follow/depth_percept' 2>/dev/null
    pkill -f 'vla_follow/lib/vla_follow/voice_node' 2>/dev/null
    pkill -f 'vla_follow/lib/vla_follow/follower' 2>/dev/null
    sleep 2
fi

echo "GZ=$(pgrep -c -f 'gz sim') BRIDGE=$(pgrep -c -f parameter_bridge) REST=$(pgrep -c -f 'vla_follow/lib')"
echo DONE
