#!/bin/bash
# tmp_verify.sh —— 完整闭环验证: 启动全套节点 → 发 FOLLOW 指令 → 采集日志
export VLA_HOME=/home/earl/workspace/VLA
export DISPLAY=:0
LOG=$VLA_HOME/logs/demo2.log

pkill -f 'ros2 launch vla_follow' 2>/dev/null
sleep 1
pkill -f 'vla_follow/lib/vla_follow' 2>/dev/null
pkill -f 'parameter_bridge' 2>/dev/null
pkill -f 'gz sim' 2>/dev/null
sleep 3
pkill -9 -f 'gz sim' 2>/dev/null
pkill -9 -f 'vla_follow/lib/vla_follow' 2>/dev/null
sleep 1
rm -rf $VLA_HOME/vla_demo/snapshots/*

source /opt/ros/jazzy/setup.bash
source $VLA_HOME/vla_ws/install/setup.bash
cd $VLA_HOME
setsid nohup ros2 launch vla_follow demo.launch.py auto_voice:=false \
    > $LOG 2>&1 < /dev/null &
echo '>>> 等待仿真就绪'
sleep 26

echo '>>> 发送跟随指令'
ros2 topic pub --once /robot/command std_msgs/msg/String \
  "{data: '{\"action\": \"FOLLOW\", \"distance\": 1.0, \"text\": \"请跟随我\"}'}" \
  >/dev/null 2>&1
sleep 45
echo '>>> 采集完成'
