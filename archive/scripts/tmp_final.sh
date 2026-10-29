#!/bin/bash
set +u
export DISPLAY=:0
source /opt/ros/jazzy/setup.bash
source /home/earl/workspace/VLA/vla_ws/install/setup.bash
cd /home/earl/workspace/VLA || exit 1
say() { echo "[FINAL $(date +%H:%M:%S)] $*"; }

pkill -f 'tmp_traj_vi[z]'
pkill -f 'tmp_circl[e]'
sleep 1
gz service -s /world/follow_world/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'name: "robot", position: {x: -3.2, y: -0.8, z: 0.15}, orientation: {w: 1.0}' >/dev/null
gz service -s /world/follow_world/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'name: "person", position: {x: 0.0, y: -1.8, z: 0.02}, orientation: {w: 1.0}' >/dev/null
say "复位完成"

setsid nohup python3 tmp_circle.py --r 2.0 --w 0.18 --cx 0 --cy 0.2 --theta0 -1.5708 --dur 900 \
  > circle2.log 2>&1 < /dev/null &
sleep 1
setsid nohup python3 tmp_traj_viz.py --world follow_world --out /home/earl/workspace/VLA/traj_log.csv \
  > traj_viz5.log 2>&1 < /dev/null &
sleep 4
ros2 topic pub --once /robot/command std_msgs/msg/String \
  "{data: '{\"action\":\"FOLLOW\",\"distance\":1.0,\"text\":\"请跟随我\"}'}" >/dev/null 2>&1
say "FOLLOW 已下发, 开始 100s 观察"
sleep 100
DISPLAY=:0 python3 tmp_shot.py shots/final1.png >/dev/null 2>&1
say "t=100s $(tail -1 traj_viz5.log)"
sleep 25
DISPLAY=:0 python3 tmp_shot.py shots/final2.png >/dev/null 2>&1
say "t=125s $(tail -1 traj_viz5.log)"
say "场景保持运行"
