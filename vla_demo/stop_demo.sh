#!/bin/bash
# stop_demo.sh —— 停止演示
export VLA_HOME="${VLA_HOME:-/home/earl/workspace/VLA}"
pkill -f 'ros2 launch vla_follow' 2>/dev/null
pkill -f 'vla_follow/lib/vla_follow' 2>/dev/null
pkill -f parameter_bridge 2>/dev/null
sleep 1
pkill -f 'gz sim' 2>/dev/null
sleep 2
pkill -9 -f 'gz sim' 2>/dev/null
echo ">>> DEMO 已停止"
