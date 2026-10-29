#!/bin/bash
# tmp_start.sh —— 后台启动 demo, 日志到 /home/earl/workspace/VLA/logs/demo2.log
export VLA_HOME=/home/earl/workspace/VLA
export DISPLAY=:0
source /opt/ros/jazzy/setup.bash
source $VLA_HOME/vla_ws/install/setup.bash
cd $VLA_HOME
setsid nohup ros2 launch vla_follow demo.launch.py "${@:-auto_voice:=false}" \
    > $VLA_HOME/logs/demo2.log 2>&1 < /dev/null &
echo "LAUNCHED $!"
