#!/bin/bash
# tmp_reg4.sh —— 跟随主流程回归 + 振荡量化 + 截图取证
set +u
export DISPLAY=:0
source /opt/ros/jazzy/setup.bash
source /home/earl/workspace/VLA/vla_ws/install/setup.bash
cd /home/earl/workspace/VLA || exit 1
say() { echo "[REG4 $(date +%H:%M:%S)] $*"; }

pkill -f 'person_wal[k]'
pkill -f 'tmp_circl[e]'
sleep 1
mkdir -p shots

put() {
  gz service -s /world/follow_world/set_pose --reqtype gz.msgs.Pose \
    --reptype gz.msgs.Boolean --timeout 3000 \
    --req "name: \"$1\", position: {x: $2, y: $3, z: $4}, orientation: {w: 1.0}" \
    >/dev/null 2>&1
}

# 人沿 r=2m 圆周匀速走, 机器人从圆心附近起步
put robot 0.0 -0.6 0.15
put person 0.0 -2.20 0.02
sleep 1
say "复位完成 robot=(0,-0.6) person=(0,-2.2)"

setsid nohup python3 tmp_circle.py --r 2.0 --w 0.16 --cx 0 --cy 0.0 \
  --theta0 -1.5708 --dur 200 > circle_reg4.log 2>&1 < /dev/null &
sleep 1
say "开始圆周行走"

ros2 topic pub --once /robot/command std_msgs/msg/String \
  "{data: '{\"action\":\"FOLLOW\",\"distance\":1.0,\"text\":\"请跟随我\"}'}" \
  >/dev/null 2>&1
say "FOLLOW 已下发"

sleep 55
DISPLAY=:0 python3 tmp_shot.py shots/reg4_a.png >/dev/null 2>&1
say "t=55s 截图 reg4_a.png"
sleep 55
DISPLAY=:0 python3 tmp_shot.py shots/reg4_b.png >/dev/null 2>&1
say "t=110s 截图 reg4_b.png"

pkill -f 'tmp_circl[e]'
python3 tmp_logstat.py demo_show.log
say "回归结束"
