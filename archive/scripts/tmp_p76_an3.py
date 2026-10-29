# -*- coding: utf-8 -*-
"""人走动时机器人跟不跟得上 + 扫描片段的次数与时长"""
import json
import math
import sys

P = sys.argv[1] if len(sys.argv) > 1 else r'C:\Users\Administrator\diag_t17.jsonl'
rows = []
for ln in open(P, encoding='utf-8'):
    ln = ln.strip()
    if ln:
        try:
            rows.append(json.loads(ln))
        except Exception:
            pass

for r in rows:
    th = r['yaw'] + math.radians(r['brg'])
    r['px'] = r['x'] + r['dist'] * math.cos(th)
    r['py'] = r['y'] + r['dist'] * math.sin(th)

# ---------- 1) 人的速度(由重建位置差分) ----------
for i in range(1, len(rows) - 1):
    dt = rows[i + 1]['t'] - rows[i - 1]['t']
    if dt <= 0.01:
        rows[i]['pspd'] = 0.0
        continue
    d = math.hypot(rows[i + 1]['px'] - rows[i - 1]['px'],
                   rows[i + 1]['py'] - rows[i - 1]['py'])
    rows[i]['pspd'] = d / dt
rows[0]['pspd'] = rows[-1]['pspd'] = 0.0

print('=== 1) 人速 vs 机器人指令速度 ===')
print('  人速中位 %.2f m/s, 均 %.2f' %
      (sorted(r['pspd'] for r in rows)[len(rows) // 2],
       sum(r['pspd'] for r in rows) / len(rows)))
for lo, hi in [(0.0, 0.05), (0.05, 0.15), (0.15, 0.25), (0.25, 0.6), (0.6, 9)]:
    seg = [r for r in rows if lo <= r['pspd'] < hi]
    if len(seg) < 3:
        continue
    v = sum(r['v'] for r in seg) / len(seg)
    print('  人速 %.2f~%.2f  n=%3d  机器人均v=%+.2f  (差 %+.2f)  人距离均%.2f'
          % (lo, hi, len(seg), v, v - (lo + hi) / 2,
             sum(r['dist'] for r in seg) / len(seg)))

# ---------- 2) 扫描片段 ----------
print('\n=== 2) 原地扫描片段 ===')
eps = []
i = 0
while i < len(rows):
    if '扫描' in rows[i].get('note', ''):
        j = i
        while j + 1 < len(rows) and '扫描' in rows[j + 1].get('note', ''):
            j += 1
        eps.append((i, j))
        i = j + 1
    else:
        i += 1
tot = 0.0
for a, b in eps:
    dur = rows[b]['t'] - rows[a]['t'] + 0.1
    tot += dur
    print('  t=%6.1f..%6.1f  %.1fs  位置(%.2f,%.2f)  人距%.2f 人方位%+.0f  mode=%s'
          % (rows[a]['t'], rows[b]['t'], dur, rows[a]['x'], rows[a]['y'],
             rows[a]['dist'], rows[a]['brg'], rows[a]['mode']))
print('  共 %d 段, 总 %.1fs (占 %.1f%%)'
      % (len(eps), tot, 100.0 * tot / (rows[-1]['t'] - rows[0]['t'])))

# ---------- 3) 丢失事件(视觉距离突然跳变/mode 切 ALIGN) ----------
print('\n=== 3) 长静止段(>1.5s 完全不动) ===')
eps = []
i = 0
while i < len(rows):
    if rows[i]['v'] <= 0.02:
        j = i
        while j + 1 < len(rows) and rows[j + 1]['v'] <= 0.02:
            j += 1
        if rows[j]['t'] - rows[i]['t'] >= 1.5:
            eps.append((i, j))
        i = j + 1
    else:
        i += 1
for a, b in eps:
    dm = math.hypot(rows[b]['x'] - rows[a]['x'], rows[b]['y'] - rows[a]['y'])
    print('  t=%6.1f..%6.1f %4.1fs 净移%.2f 人距%.2f->%.2f 人方位%+.0f mode=%s note=%s'
          % (rows[a]['t'], rows[b]['t'], rows[b]['t'] - rows[a]['t'], dm,
             rows[a]['dist'], rows[b]['dist'], rows[a]['brg'],
             rows[a]['mode'], (rows[a].get('note') or '-')[:28]))
