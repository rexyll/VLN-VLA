# -*- coding: utf-8 -*-
"""按"人到立柱的距离"交叉验证: 障碍旁的徘徊到底是不是避障急停造成的"""
import json
import math
import sys
from collections import Counter

P = sys.argv[1] if len(sys.argv) > 1 else r'C:\Users\Administrator\diag_t17.jsonl'

# SDF 里的精确几何 (x, y, kind, a, b, yaw)
OBS = [
    ('ob1', 3.5, 2.0, 'box', 0.8, 0.8, 0.0),
    ('ob2', -3.0, 3.2, 'box', 1.2, 0.6, 0.5),
    ('ob3', 0.5, -3.2, 'cyl', 0.4, 0.0, 0.0),
    ('ob4', -4.2, -2.5, 'cyl', 0.5, 0.0, 0.0),
    ('ob5', 4.2, -3.0, 'box', 0.6, 0.6, 0.0),
    ('ob6', -1.5, -4.5, 'box', 1.0, 0.5, -0.4),
    ('ob7', 2.2, 4.6, 'cyl', 0.35, 0.0, 0.0),
]
WALL = 6.0


def gap(x, y):
    """点到最近障碍表面的距离(负数表示已进入)"""
    best = (WALL - abs(x), 'wall')
    for nm, ox, oy, k, a, b, yw in OBS:
        if k == 'cyl':
            d = math.hypot(x - ox, y - oy) - a
        else:
            dx, dy = x - ox, y - oy
            c, s = math.cos(-yw), math.sin(-yw)
            lx, ly = dx * c - dy * s, dx * s + dy * c
            ex = max(abs(lx) - a / 2.0, 0.0)
            ey = max(abs(ly) - b / 2.0, 0.0)
            inside_x = a / 2.0 - abs(lx)
            inside_y = b / 2.0 - abs(ly)
            d = math.hypot(ex, ey) if (ex > 0 or ey > 0) else \
                -min(inside_x, inside_y)
        if d < best[0]:
            best = (d, nm)
    if WALL - abs(y) < best[0]:
        best = (WALL - abs(y), 'wall')
    return best


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
    r['pgap'], r['pobj'] = gap(r['px'], r['py'])
    r['rgap'], r['robj'] = gap(r['x'], r['y'])

print('样本 %d  t=%.1f..%.1f' % (len(rows), rows[0]['t'], rows[-1]['t']))

print('\n=== A) 人到最近障碍的距离  vs 机器人停住比例 ===')
bins = [(-9, 0.15), (0.15, 0.35), (0.35, 0.6), (0.6, 1.0), (1.0, 1.5),
        (1.5, 2.5), (2.5, 99)]
for lo, hi in bins:
    seg = [r for r in rows if lo <= r['pgap'] < hi]
    if not seg:
        continue
    zn = sum(1 for r in seg if r['v'] <= 0.02)
    sc = sum(1 for r in seg if '扫描' in r.get('note', ''))
    al = sum(1 for r in seg if r['mode'] == 'ALIGN')
    print('  pgap %5.2f~%5.2f n=%3d  停%3.0f%%  扫描%3.0f%%  对正%3.0f%%  均v=%.2f'
          % (lo, hi, len(seg), 100.0 * zn / len(seg), 100.0 * sc / len(seg),
             100.0 * al / len(seg), sum(r['v'] for r in seg) / len(seg)))

print('\n=== B) 人贴近障碍(<0.6m)时, 机器人最终停在什么原因 ===')
seg = [r for r in rows if r['pgap'] < 0.6 and r['v'] <= 0.02]
c = Counter()
for r in seg:
    n = r.get('note', '').strip()
    if not n:
        n = '无note(距离已到位 dist=%.1f)' % r['dist']
    c[n.split(' 前')[0].split(' 左前')[0].split(' 侧')[0][:20]] += 1
for k, v in c.most_common(10):
    print('  %3d  %s' % (v, k))

print('\n=== C) 人贴近障碍(<0.6m)时, 机器人在做什么(mode) ===')
c = Counter(r['mode'] for r in rows if r['pgap'] < 0.6)
tot = sum(c.values())
for k, v in c.most_common():
    print('  %3d  %4.0f%%  %s' % (v, 100.0 * v / max(1, tot), k))

print('\n=== D) 机器人自身贴近障碍(<0.35m)时的行为 ===')
seg = [r for r in rows if r['rgap'] < 0.35]
print('  样本 %d' % len(seg))
if seg:
    c = Counter((r.get('note', '').strip().split(' 前')[0].split(' 侧')[0][:18]
                 or '无note') for r in seg)
    for k, v in c.most_common(8):
        print('  %3d  %s' % (v, k))

print('\n=== E) 全部 note 出现次数(前 15) ===')
c = Counter((r.get('note', '').strip().split(' 前')[0].split(' 左前')[0]
             .split(' 侧')[0][:24] or '无note') for r in rows)
for k, v in c.most_common(15):
    print('  %3d  %s' % (v, k))
