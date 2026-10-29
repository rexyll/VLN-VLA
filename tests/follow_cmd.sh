#!/bin/bash
# tests/follow_cmd.sh —— 下发"跟随"指令(命令行方式, DDS 发现完整)
#
# 位置: tests/
export VLA_HOME="${VLA_HOME:-/home/earl/workspace/VLA}"
source /opt/ros/jazzy/setup.bash
source "$VLA_HOME/vla_ws/install/setup.bash"
ros2 topic pub --once /robot/command std_msgs/msg/String \
  '{data: "{\"action\":\"FOLLOW\",\"distance\":1.0,\"text\":\"follow\"}"}'
