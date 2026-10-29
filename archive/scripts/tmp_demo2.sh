#!/bin/bash
# tmp_demo2.sh —— 可视化演示：复位 → 清轨迹 → FOLLOW → 定时截图 → 轨迹统计
set +u
export DISPLAY=:0
source /opt/ros/jazzy/setup.bash
source /home/earl/workspace/VLA/vla_ws/install/setup.bash
cd /home/earl/workspace/VLA || exit 1

say() { echo "[DEMO $(date +%H:%M:%S)] $*"; }

# ---- 1. 复位 ----
gz service -s /world/follow_world/set_pose --reqtype gz.msgs.Pose \
  --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'name: "robot", position: {x: -4.4, y: -4.3, z: 0.15}, orientation: {w: 1.0}' >/dev/null
gz service -s /world/follow_world/set_pose --reqtype gz.msgs.Pose \
  --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'name: "person", position: {x: -1.6, y: -2.6, z: 0.2}, orientation: {w: 1.0}' >/dev/null
say "复位 robot=(-4.4,-4.3) person=(-1.6,-2.6)"
sleep 2

# ---- 2. 重启轨迹可视化（清空旧 marker，从起点重新画） ----
pkill -f 'tmp_traj_vi[z]'
sleep 1
setsid nohup python3 tmp_traj_viz.py --world follow_world \
  --out /home/earl/workspace/VLA/traj_log.csv \
  > /home/earl/workspace/VLA/traj_viz3.log 2>&1 < /dev/null &
sleep 4
say "轨迹节点: $(sed -n '1p' traj_viz3.log)"

# ---- 3. 重新下发 FOLLOW（人已在斜前方 3.5m） ----
ros2 topic pub --once /robot/command std_msgs/msg/String \
  "{data: '{\"action\":\"FOLLOW\",\"distance\":1.0,\"text\":\"请跟随我\"}'}" >/dev/null 2>&1
say "FOLLOW 已下发"

DISPLAY=:0 python3 tmp_shot.py shots/d00.png >/dev/null 2>&1
say "起点截图完成"

# ---- 4. 每 15s 截图 ----
for i in 1 2 3 4 5 6; do
  sleep 15
  DISPLAY=:0 python3 tmp_shot.py shots/d$(printf %02d $((i*15))).png >/dev/null 2>&1
  say "t=$((i*15))s $(tail -1 traj_viz3.log)"
done

# ---- 5. 轨迹统计 ----
python3 - <<'PY'
import csv, math
rows = list(csv.DictReader(open('/home/earl/workspace/VLA/traj_log.csv', encoding='utf-8')))
def path(keys):
    tot = 0.0
    for a, b in zip(rows, rows[1:]):
        tot += math.hypot(float(b[keys[0]]) - float(a[keys[0]]),
                          float(b[keys[1]]) - float(a[keys[1]]))
    return tot
print('[DEMO] 采样 %d 点, 时长 %.1fs' % (len(rows), float(rows[-1]['t']) - float(rows[0]['t'])))
print('[DEMO] 机器人行程 %.2fm, 目标行程 %.2fm' % (path(('rx', 'ry')), path(('px', 'py'))))
d = [math.hypot(float(r['rx']) - float(r['px']), float(r['ry']) - float(r['py'])) for r in rows]
d2 = d[5:]
print('[DEMO] 距离 min=%.2f 中位=%.2f max=%.2f' % (min(d2), sorted(d2)[len(d2)//2], max(d2)))
print('[DEMO] 末态 机器人=(%.2f,%.2f) 目标=(%.2f,%.2f) 距离=%.2f' %
      (float(rows[-1]['rx']), float(rows[-1]['ry']),
       float(rows[-1]['px']), float(rows[-1]['py']), d[-1]))
PY
say "演示结束"
