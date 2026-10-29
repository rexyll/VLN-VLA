#!/bin/bash
# tmp_reg.sh —— 完整回归: 对正/跟踪 → 四相机切换 → 丢失后 GOTO/SCAN
set +u
source /opt/ros/jazzy/setup.bash
source /home/earl/workspace/VLA/vla_ws/install/setup.bash
cd /home/earl/workspace/VLA || exit 1

LOG=/home/earl/workspace/VLA/reg.log
put() {  # put <模型> <x> <y> <yaw度>
  local deg=${4:-0}
  local rad
  rad=$(python3 -c "import math;print(math.radians($deg/2))")
  gz service -s /world/follow_world/set_pose --reqtype gz.msgs.Pose \
    --reptype gz.msgs.Boolean --timeout 3000 \
    --req "name: \"$1\", position: {x: $2, y: $3, z: ${5:-0.2}}, orientation: {z: $rad, w: $(python3 -c "import math;print(math.cos(math.radians($deg/2)))")}" \
    >/dev/null 2>&1
  sleep 0.6
}

mark() { echo "---- $(date +%H:%M:%S) :: $* ----" >> "$LOG"; }

# ---------- 启动 ----------
pkill -f 'gz si[m]'; pkill -f 'vla_follo[w]/lib/vla_follow'; pkill -f 'ros2 launc[h]'
sleep 3
: > "$LOG"
setsid nohup ros2 launch vla_follow demo.launch.py auto_voice:=false \
  > "$LOG" 2>&1 < /dev/null &
echo "启动中, 等 30s..."
sleep 30

# ---------- 阶段A: 对正 + 跟踪行进中目标 ----------
put robot -4.0 -4.0 0 0.15
put person -2.0 -4.0 0 0.2
sleep 2
mark "阶段A 发 FOLLOW (人正前方 2m, person_walk 会随机走)"
ros2 topic pub --once /robot/command std_msgs/msg/String \
  "{data: '{\"action\":\"FOLLOW\",\"distance\":1.0,\"text\":\"请跟随我\"}'}" >/dev/null 2>&1
sleep 48

# ---------- 阶段B: 受控四方位切换 ----------
pkill -f 'person_wal[k]'
sleep 1
for spec in "左侧:-4.0:-2.4:0" "后方:-5.6:-4.0:0" "右侧:-4.0:-5.6:0" "前方:-2.4:-4.0:0"; do
  name=${spec%%:*}; rest=${spec#*:}
  x=$(echo "$rest" | cut -d: -f1); y=$(echo "$rest" | cut -d: -f2)
  put robot -4.0 -4.0 0 0.15
  put person "$x" "$y" 0 0.2
  mark "阶段B 人置于$name ($x,$y) 相对机器人"
  sleep 9
done

# ---------- 阶段C: 目标消失 → GOTO → SCAN ----------
put robot -4.0 -4.0 0 0.15
put person 5.0 5.0 0 0.2
mark "阶段C 人移出视野(5.0,5.0), 观察 丢失→搜索→扫描"
sleep 30

pkill -f 'gz si[m]'; pkill -f 'vla_follo[w]/lib/vla_follow'; pkill -f 'ros2 launc[h]'
echo "=== 回归结束, 日志: $LOG ==="
grep -c FOLLOWER "$LOG"
