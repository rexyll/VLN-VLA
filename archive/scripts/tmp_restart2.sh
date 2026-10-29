#!/bin/bash
# tmp_restart2.sh —— 受控重启: 启动后立即移走"人"并停掉所有业务节点,
# 只保留 gz sim + parameter_bridge, 用于纯物理/指令符号验证
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
sleep 14

# 立刻停掉业务节点, 把人挪到远角, 机器人归位到原点
pkill -f 'vla_follow/lib/vla_follow/person_walk' 2>/dev/null
pkill -f 'vla_follow/lib/vla_follow/person_vision' 2>/dev/null
pkill -f 'vla_follow/lib/vla_follow/depth_percept' 2>/dev/null
pkill -f 'vla_follow/lib/vla_follow/voice_node' 2>/dev/null
pkill -f 'vla_follow/lib/vla_follow/follower' 2>/dev/null
sleep 2
gz service -s /world/follow_world/set_pose --reqtype gz.msgs.Pose \
   --reptype gz.msgs.Boolean --timeout 500 \
   --req 'name: "person", position: {x: 4.5, y: 4.5, z: 0.02}, orientation: {w: 1}' >/dev/null 2>&1
gz service -s /world/follow_world/set_pose --reqtype gz.msgs.Pose \
   --reptype gz.msgs.Boolean --timeout 500 \
   --req 'name: "robot", position: {x: 0, y: 0, z: 0.0}, orientation: {w: 1}' >/dev/null 2>&1
sleep 3
echo "REST=$(pgrep -c -f 'vla_follow/lib') GZ=$(pgrep -c -f 'gz sim')"
echo READY
