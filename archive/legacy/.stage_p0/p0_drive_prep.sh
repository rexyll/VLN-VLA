#!/bin/bash
# p0_drive_prep2.sh —— 底盘物理直测准备(排除人推机器人这个混淆因素)
pkill -f 'vla_follow/lib/vla_follow/follow[e]r'
pkill -f 'vla_follow/lib/vla_follow/cmd_mu[x]'
pkill -f 'vla_follow/lib/vla_follow/person_wal[k]'
pkill -f 'nav2_act[i]ve'
sleep 2
echo "CMD_VEL_WRITERS_STOPPED"
source /opt/ros/jazzy/setup.bash
source /home/earl/workspace/VLA/vla_ws/install/setup.bash
timeout 8 ros2 topic info /robot/cmd_vel 2>&1 | tr '\n' ' '
echo
echo "--- person 位姿 ---"
timeout 10 gz model -m person -p 2>/dev/null | grep -E "x:|y:|z:" | head -3
echo "--- robot 位姿 ---"
timeout 10 gz model -m robot -p 2>/dev/null | grep -E "x:|y:|z:" | head -3
