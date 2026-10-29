# -*- coding: utf-8 -*-
"""判别 deploy/work_area.sdf (轴 0 0 -1) 的方向约定:
   1) 机器人前进方向 vs 车头方向
   2) 机器人偏航率 dyaw/dt vs 目标方位角 bearing 的符号关系
      bearing>0 = 目标在车头左侧(逆时针方向) -> 要转到目标, dyaw/dt 必须 >0
      => 若 dyaw/dt 与 bearing 同号, 说明 deploy 版 "转向也正确"
         若异号, 说明 deploy 版 "转向是反的"
   3) 人(person)的移动方向 vs 自身朝向 pyaw
      v>0 应沿自身 pyaw 前进 -> dot(dpos, (cos pyaw, sin pyaw)) > 0
"""
import csv
import math

path = 'deploy/traj_log.csv'
rows = []
with open(path, newline='', encoding='utf-8', errors='replace') as f:
    for r in csv.DictReader(f):
        try:
            rows.append({k: float(v) for k, v in r.items() if v not in (None, '')})
        except (ValueError, TypeError):
            continue

print('总行数', len(rows))
if not rows:
    raise SystemExit('no data')


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


# ---------- 1) 机器人前进方向 ----------
fwd_dot = []
for i in range(1, len(rows)):
    dx = rows[i]['rx'] - rows[i - 1]['rx']
    dy = rows[i]['ry'] - rows[i - 1]['ry']
    n = math.hypot(dx, dy)
    if n < 1e-4:
        continue
    yaw = rows[i]['ryaw']
    fwd_dot.append((dx * math.cos(yaw) + dy * math.sin(yaw)) / n)
print('\n[机器人] 速度方向·车头方向: 均值 %.3f  中位 %.3f  '
      '同向帧 %d/%d (%.1f%%)'
      % (sum(fwd_dot) / len(fwd_dot),
         sorted(fwd_dot)[len(fwd_dot) // 2],
         sum(1 for x in fwd_dot if x > 0), len(fwd_dot),
         100.0 * sum(1 for x in fwd_dot if x > 0) / len(fwd_dot)))

# ---------- 2) 偏航率 vs bearing 符号 ----------
sames = []
sames_cnt = 0
opps_cnt = 0
total = 0
for i in range(1, len(rows)):
    dt = rows[i]['t'] - rows[i - 1]['t']
    if dt <= 0 or dt > 1.0:
        continue
    dyaw = wrap(rows[i]['ryaw'] - rows[i - 1]['ryaw'])
    if abs(dyaw) < 1e-3:
        continue
    brg = wrap(math.atan2(rows[i]['py'] - rows[i]['ry'],
                          rows[i]['px'] - rows[i]['rx']) - rows[i]['ryaw'])
    if abs(brg) < math.radians(20) or abs(brg) > math.radians(160):
        continue          # 只看"目标在侧方"的样本, 这时转向需求明确
    w = dyaw / dt
    total += 1
    if (w > 0) == (brg > 0):
        sames_cnt += 1
    else:
        opps_cnt += 1
    sames.append((brg, w))
print('\n[机器人转向] 目标在侧方(20°~160°)样本 %d 组: '
      '同号(转向正确) %d 组 (%.1f%%), 异号(转向反向) %d 组 (%.1f%%)'
      % (total, sames_cnt, 100.0 * sames_cnt / max(total, 1),
         opps_cnt, 100.0 * opps_cnt / max(total, 1)))

# ---------- 3) 人前进方向 ----------
p_dot = []
for i in range(1, len(rows)):
    dx = rows[i]['px'] - rows[i - 1]['px']
    dy = rows[i]['py'] - rows[i - 1]['py']
    n = math.hypot(dx, dy)
    if n < 1e-4:
        continue
    pyaw = rows[i]['pyaw']
    p_dot.append((dx * math.cos(pyaw) + dy * math.sin(pyaw)) / n)
if p_dot:
    print('\n[人] 移动方向·自身朝向: 均值 %.3f  中位 %.3f  '
          '同向帧 %d/%d (%.1f%%)'
          % (sum(p_dot) / len(p_dot),
             sorted(p_dot)[len(p_dot) // 2],
             sum(1 for x in p_dot if x > 0), len(p_dot),
             100.0 * sum(1 for x in p_dot if x > 0) / len(p_dot)))

# ---------- 4) 人机距离 ----------
d = [math.hypot(r['px'] - r['rx'], r['py'] - r['ry']) for r in rows]
d.sort()
print('\n[人机距离] 首 %.2fm  末 %.2fm  中位 %.2fm  最小 %.2fm  最大 %.2fm'
      % (math.hypot(rows[0]['px'] - rows[0]['rx'], rows[0]['py'] - rows[0]['ry']),
         math.hypot(rows[-1]['px'] - rows[-1]['rx'], rows[-1]['py'] - rows[-1]['ry']),
         d[len(d) // 2], d[0], d[-1]))
