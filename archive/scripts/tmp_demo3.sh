#!/bin/bash
# tmp_demo3.sh —— 跟随效果演示（目标沿圆周平滑行走，机器人跟随）
set +u
export DISPLAY=:0
source /opt/ros/jazzy/setup.bash
source /home/earl/workspace/VLA/vla_ws/install/setup.bash
cd /home/earl/workspace/VLA || exit 1

say() { echo "[DEMO $(date +%H:%M:%S)] $*"; }

# ---- 1. 重启仿真（让 follower 读到新的 approach_speed） ----
pkill -f 'gz si[m]'
pkill -f 'vla_follo[w]/lib/vla_follow'
pkill -f 'ros2 launc[h]'
pkill -f 'tmp_traj_vi[z]'
pkill -f 'tmp_circl[e]'
sleep 4
setsid nohup ros2 launch vla_follow demo.launch.py auto_voice:=false \
  > /home/earl/workspace/VLA/demo_show.log 2>&1 < /dev/null &
say "launch 启动, 等待 32s"
sleep 32

# ---- 2. 接管"人": 停掉随机行走, 改圆周运动 ----
pkill -f 'person_wal[k]'
sleep 1
gz service -s /world/follow_world/set_pose --reqtype gz.msgs.Pose \
  --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'name: "robot", position: {x: -3.2, y: -0.8, z: 0.15}, orientation: {w: 1.0}' >/dev/null
gz service -s /world/follow_world/set_pose --reqtype gz.msgs.Pose \
  --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'name: "person", position: {x: 0.0, y: -1.8, z: 0.02}, orientation: {w: 1.0}' >/dev/null
say "已复位: robot=(-3.2,-0.8) person=(0,-1.8)"

setsid nohup python3 tmp_circle.py --r 2.0 --w 0.18 --cx 0 --cy 0.2 \
  --theta0 -1.5708 --dur 75 \
  > /home/earl/workspace/VLA/circle.log 2>&1 < /dev/null &
sleep 2

# ---- 3. 轨迹可视化 ----
setsid nohup python3 tmp_traj_viz.py --world follow_world \
  --out /home/earl/workspace/VLA/traj_log.csv \
  > /home/earl/workspace/VLA/traj_viz4.log 2>&1 < /dev/null &
sleep 4
say "轨迹: $(sed -n '1p' traj_viz4.log)"

# ---- 4. 跟随指令 ----
ros2 topic pub --once /robot/command std_msgs/msg/String \
  "{data: '{\"action\":\"FOLLOW\",\"distance\":1.0,\"text\":\"请跟随我\"}'}" >/dev/null 2>&1
say "FOLLOW 已下发"
sleep 3
DISPLAY=:0 python3 tmp_shot.py shots/e00.png >/dev/null 2>&1

# ---- 5. 定时截图 ----
for i in 1 2 3 4 5; do
  sleep 15
  if [ "$(pgrep -cf 'gz si[m]')" = "0" ]; then say "!! gz 已退出, 中止"; break; fi
  DISPLAY=:0 python3 tmp_shot.py shots/e$(printf %02d $((i*15))).png >/dev/null 2>&1
  say "t=$((i*15))s $(tail -1 traj_viz4.log)"
done

# ---- 6. 统计 ----
python3 - <<'PY'
import csv, math
rows = list(csv.DictReader(open('/home/earl/workspace/VLA/traj_log.csv', encoding='utf-8')))
def path(keys):
    return sum(math.hypot(float(b[keys[0]]) - float(a[keys[0]]),
                          float(b[keys[1]]) - float(a[keys[1]]))
               for a, b in zip(rows, rows[1:]))
print('[DEMO] 采样 %d 点, 时长 %.1fs' % (len(rows), float(rows[-1]['t']) - float(rows[0]['t'])))
print('[DEMO] 机器人行程 %.2fm, 目标行程 %.2fm' % (path(('rx', 'ry')), path(('px', 'py'))))
d = [math.hypot(float(r['rx']) - float(r['px']), float(r['ry']) - float(r['py'])) for r in rows]
tail = d[20:]
print('[DEMO] 稳态距离 min=%.2f 中位=%.2f max=%.2f' % (min(tail), sorted(tail)[len(tail)//2], max(tail)))
err = [abs(v - 1.0) for v in tail]
print('[DEMO] |距离-1.0m| 平均=%.2fm, 落在 ±0.3m 内的比例=%.0f%%'
      % (sum(err) / len(err), 100.0 * sum(1 for e in err if e <= 0.3) / len(err)))
PY
say "演示结束"
