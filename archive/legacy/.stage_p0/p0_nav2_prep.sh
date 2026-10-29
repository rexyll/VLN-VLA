#!/bin/bash
# p0_nav2_prep.sh —— 为 Nav2 独立验证做准备
# 关键: pkill -f 的模式不能出现在执行它的命令行里(否则会杀掉自己),
#       所以这段必须放在独立脚本文件中执行。
pkill -f 'vla_follow/lib/vla_follow/follower'
pkill -f 'nav2_active'
sleep 1
if pgrep -f 'vla_follow/lib/vla_follow/follower' >/dev/null; then
    echo FOLLOWER_STILL_ALIVE
else
    echo FOLLOWER_STOPPED
fi

source /opt/ros/jazzy/setup.bash
source /home/earl/workspace/VLA/vla_ws/install/setup.bash
setsid nohup ros2 topic pub -r 10 /nav2_active std_msgs/msg/Bool '{data: true}' \
    > /tmp/nav2act.log 2>&1 < /dev/null 9>&- &
sleep 2
echo PUB_STARTED
