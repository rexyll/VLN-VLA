#!/bin/bash
# tmp_reg2.sh —— 完整回归 v2
#   阶段A 对正+跟踪动态目标 | 阶段B 受控四方位 | 阶段C 目标消失→搜索→扫描
# 与 v1 的差别: 启动后先自检位姿流, 不通就直接中止(避免拿僵尸节点的日志
# 得出结论); 阶段标记写到独立文件, 不再被 launch 的 stdout 覆盖。
set +u
# gz sim 默认带 GUI, SSH 会话里 DISPLAY 为空会让 Qt 初始化失败并直接退出
# (表现为 "process has finished cleanly"), 必须先指到本机桌面会话。
export DISPLAY=${DISPLAY:-:0}
source /opt/ros/jazzy/setup.bash
source /home/earl/workspace/VLA/vla_ws/install/setup.bash
cd /home/earl/workspace/VLA || exit 1

LOG=/home/earl/workspace/VLA/reg2.log
MARK=/home/earl/workspace/VLA/reg2_mark.log
mark() { echo "[$(date +%H:%M:%S)] $*" >> "$MARK"; }

put() {  # put <模型> <x> <y> <deg> <z>
  local deg=${4:-0} z=${5:-0.2}
  local rad w
  read -r rad w < <(python3 -c \
    "import math;a=math.radians($deg);print(a/2,math.cos(a/2))")
  local r
  r=$(gz service -s /world/follow_world/set_pose --reqtype gz.msgs.Pose \
      --reptype gz.msgs.Boolean --timeout 3000 \
      --req "name: \"$1\", position: {x: $2, y: $3, z: $z}, \
orientation: {z: $rad, w: $w}" 2>/dev/null | tr -d '\n')
  mark "  set_pose $1=($2,$3,yaw=${deg}°) → ${r:-NO_RESP}"
  sleep 0.6
}

# ---------- 清场 ----------
pkill -f 'gz si[m]'; pkill -f 'vla_follo[w]/lib/vla_follow'; pkill -f 'ros2 launc[h]'
sleep 5
left=$(pgrep -cf 'gz si[m]')
mark "=== 清场完成 (残留 gz 进程 $left) ==="

# ---------- 启动 + 自检 ----------
: > "$LOG"; : > "$MARK"
setsid nohup ros2 launch vla_follow demo.launch.py auto_voice:=false \
  > "$LOG" 2>&1 < /dev/null &
echo "启动中, 等 26s..."
sleep 26
mark "=== 启动后自检 ==="
mark "$(timeout 15 python3 tmp_probe.py)"
if ! grep -q 'POSE_OK' "$MARK"; then
  mark "!!! 位姿流不通, 中止(不采信无效日志)"
  pkill -f 'gz si[m]'; pkill -f 'vla_follo[w]/lib/vla_follow'; pkill -f 'ros2 launc[h]'
  echo "ABORT"; exit 1
fi

# ---------- 阶段A ----------
put robot -4.0 -4.0 0 0.15
put person -2.0 -4.0 0 0.2
sleep 2
mark "=== 阶段A: FOLLOW (人正前 2m, person_walk 随机走) ==="
ros2 topic pub --once /robot/command std_msgs/msg/String \
  "{data: '{\"action\":\"FOLLOW\",\"distance\":1.0,\"text\":\"请跟随我\"}'}" \
  >/dev/null 2>&1
sleep 45

# ---------- 阶段B ----------
pkill -f 'person_wal[k]'
sleep 1
mark "=== 阶段B: 受控四方位 ==="
for spec in "左:-4.0:-2.4:0" "后:-5.6:-4.0:0" "右:-4.0:-5.6:0" "前:-2.4:-4.0:0"; do
  nm=${spec%%:*}; rest=${spec#*:}
  x=${rest%%:*}; rest2=${rest#*:}; y=${rest2%%:*}
  put robot -4.0 -4.0 0 0.15
  put person "$x" "$y" 0 0.2
  mark "--- B: 人置于[$nm] ($x,$y), 相对机器人 yaw=0 ---"
  sleep 9
done

# ---------- 阶段C ----------
put robot -4.0 -4.0 0 0.15
put person 5.0 5.0 0 0.2
mark "=== 阶段C: 人移到(5.0,5.0) 视野外, 观察 丢失→搜索→扫描 ==="
sleep 30

pkill -f 'gz si[m]'; pkill -f 'vla_follo[w]/lib/vla_follow'; pkill -f 'ros2 launc[h]'
mark "=== 结束 ==="
echo "DONE"
