#!/bin/bash
# tmp_reg3.sh —— 专测"目标丢失 → GOTO → SCAN"链路(P23 修复前不可达)
set +u
export DISPLAY=${DISPLAY:-:0}
source /opt/ros/jazzy/setup.bash
source /home/earl/workspace/VLA/vla_ws/install/setup.bash
cd /home/earl/workspace/VLA || exit 1

LOG=/home/earl/workspace/VLA/reg3.log
MARK=/home/earl/workspace/VLA/reg3_mark.log
mark() { echo "[$(date +%H:%M:%S)] $*" >> "$MARK"; }

put() {
  local deg=${4:-0} z=${5:-0.2} rad w r
  read -r rad w < <(python3 -c \
    "import math;a=math.radians($deg);print(a/2,math.cos(a/2))")
  r=$(gz service -s /world/follow_world/set_pose --reqtype gz.msgs.Pose \
      --reptype gz.msgs.Boolean --timeout 3000 \
      --req "name: \"$1\", position: {x: $2, y: $3, z: $z}, orientation: {z: $rad, w: $w}" \
      2>/dev/null | tr -d '\n')
  mark "set_pose $1=($2,$3) → ${r:-NO_RESP}"
  sleep 0.6
}

pkill -f 'gz si[m]'; pkill -f 'vla_follo[w]/lib/vla_follow'; pkill -f 'ros2 launc[h]'
sleep 5
: > "$LOG"; : > "$MARK"
setsid nohup ros2 launch vla_follow demo.launch.py auto_voice:=false \
  > "$LOG" 2>&1 < /dev/null &
echo "启动中, 等 26s..."
sleep 26
mark "自检: $(timeout 15 python3 tmp_probe.py)"

put robot -4.0 -4.0 0 0.15
put person -2.4 -4.0 0 0.2
sleep 2
mark "=== 发 FOLLOW, 人正前 1.6m ==="
ros2 topic pub --once /robot/command std_msgs/msg/String \
  "{data: '{\"action\":\"FOLLOW\",\"distance\":1.0,\"text\":\"请跟随我\"}'}" \
  >/dev/null 2>&1
sleep 14

mark "=== 人移到 (4.5,4.5), 应触发 丢失→GOTO→SCAN ==="
put person 4.5 4.5 0 0.2
sleep 40

mark "=== 结束 ==="
pkill -f 'gz si[m]'; pkill -f 'vla_follo[w]/lib/vla_follow'; pkill -f 'ros2 launc[h]'
echo DONE
