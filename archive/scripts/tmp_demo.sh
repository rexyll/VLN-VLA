#!/bin/bash
# tmp_demo.sh —— 一次完整的可视化演示：复位 → 开轨迹 → 语音跟随 → 定时截图
set +u
export DISPLAY=:0
source /opt/ros/jazzy/setup.bash
source /home/earl/workspace/VLA/vla_ws/install/setup.bash
cd /home/earl/workspace/VLA || exit 1

say() { echo "[DEMO $(date +%H:%M:%S)] $*"; }

# ---- 1. 复位到场地西南角，让人在斜前方 3.6m ----
gz service -s /world/follow_world/set_pose --reqtype gz.msgs.Pose \
  --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'name: "robot", position: {x: -4.0, y: -4.0, z: 0.15}, orientation: {w: 1.0}' >/dev/null
gz service -s /world/follow_world/set_pose --reqtype gz.msgs.Pose \
  --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'name: "person", position: {x: -0.5, y: -3.0, z: 0.2}, orientation: {w: 1.0}' >/dev/null
say "已复位 robot=(-4,-4,0°) person=(-0.5,-3)"
sleep 1

# ---- 2. 重启轨迹可视化（清空旧 marker，从新位置开始画） ----
pkill -f 'tmp_traj_vi[z]'
sleep 1
setsid nohup python3 tmp_traj_viz.py --world follow_world \
  --out /home/earl/workspace/VLA/traj_log.csv \
  > /home/earl/workspace/VLA/traj_viz.log 2>&1 < /dev/null &
sleep 3
say "轨迹可视化: $(head -2 traj_viz.log | tr '\n' ' ')"

# ---- 3. 语音指令：跟随 ----
ros2 topic pub --once /robot/command std_msgs/msg/String \
  "{data: '{\"action\":\"FOLLOW\",\"distance\":1.0,\"text\":\"请跟随我\"}'}" >/dev/null 2>&1
say "已下发 FOLLOW 指令"

# ---- 4. 每 12s 截图一次 ----
for i in 1 2 3 4 5 6 7; do
  sleep 12
  DISPLAY=:0 python3 tmp_shot.py shots/t$(printf %02d $((i*12))).png >> shot.log 2>&1
  say "截图 t=$((i*12))s: $(tail -1 traj_viz.log)"
done

say "演示结束"
