# -*- coding: utf-8 -*-
"""分析 /follower/diag —— 定位"障碍物旁徘徊不前"的根因"""
import json
import math
import sys
from collections import Counter

P = sys.argv[1] if len(sys.argv) > 1 else r'C:\Users\Administrator\diag_t17.jsonl'
rows = []
for ln in open(P, encoding='utf-8'):
    ln = ln.strip()
    if not ln:
        continue
    try:
        rows.append(json.loads(ln))
    except Exception:
        pass
print('样本 %d, t=%.1f..%.1f s' % (len(rows), rows[0]['t'], rows[-1]['t']))

DT = 0.1


def net_move(i, win):
    j = min(len(rows) - 1, i + int(win / DT))
    return math.hypot(rows[j]['x'] - rows[i]['x'], rows[j]['y'] - rows[i]['y']), j


# ---------- 1) 全程统计 ----------
mv = [r for r in rows if r['v'] > 0.02]
st = [r for r in rows if r['v'] <= 0.02]
print('\n=== 1) 运动/静止占比 ===')
print('  运动 v>0.02 : %d (%.1f%%)' % (len(mv), 100.0 * len(mv) / len(rows)))
print('  静止 v<=0.02: %d (%.1f%%)' % (len(st), 100.0 * len(st) / len(rows)))

# ---------- 2) 静止的原因分布 ----------
print('\n=== 2) 静止(v<=0.02)时 note 分布 ===')
c = Counter()
for r in st:
    n = r.get('note', '').strip()
    if not n:
        n = '(无 note) dist=%.1f brg=%.0f' % (r['dist'], r['brg'])
    key = n.split(' 前')[0].split(' 左前')[0].split(' 侧')[0][:22]
    c[key] += 1
for k, v in c.most_common(12):
    print('  %4d  %5.1f%%  %s' % (v, 100.0 * v / len(st), k))

# ---------- 3) 徘徊片段 ----------
print('\n=== 3) 徘徊片段(2s 净位移<0.15m 且 |w|>0.3) ===')
win = 2.0
eps = []
i = 0
while i < len(rows):
    dm, j = net_move(i, win)
    seg = rows[i:j + 1]
    wamp = sum(abs(r['w']) for r in seg) * DT
    turn_sign = 0
    last = 0.0
    for r in seg:
        if abs(r['w']) > 0.3 and (last == 0.0 or r['w'] * last < 0):
            turn_sign += 1
            last = r['w']
    if dm < 0.15 and wamp > 0.8:
        eps.append((i, j, dm, wamp, turn_sign))
        i = j + 1
    else:
        i += 1
print('  片段数 %d, 总时长 %.1fs (占 %.1f%%)'
      % (len(eps), sum((b - a) * DT for a, b, *_ in eps),
         100.0 * sum((b - a) * DT for a, b, *_ in eps) / (len(rows) * DT)))
cn = Counter()
for a, b, dm, wamp, ts in eps:
    seg = rows[a:b + 1]
    n = Counter(x.get('note', '').strip() for x in seg).most_common(1)[0][0]
    cn[(n.split(' 前')[0].split(' 左前')[0].split(' 侧')[0][:20])] += 1
print('  主因分布:')
for k, v in cn.most_common(10):
    print('    %3d  %s' % (v, k if k else '(无 note)'))

print('\n  逐段明细(前 15 段):')
for a, b, dm, wamp, ts in eps[:15]:
    seg = rows[a:b + 1]
    notes = Counter(x.get('note', '').strip() for x in seg).most_common(2)
    print('   t=%6.1f..%6.1f 净移%.3f 转动%.2f 换向%d 位置(%.2f,%.2f) '
          '人距%.2f 人方位%+.0f dfl=%.2f dfr=%.2f dpath=%.2f mode=%s'
          % (seg[0]['t'], seg[-1]['t'], dm, wamp, ts, seg[0]['x'], seg[0]['y'],
             seg[0]['dist'], seg[0]['brg'],
             min(x['dfl'] for x in seg), min(x['dfr'] for x in seg),
             min(x['dp'] for x in seg), seg[0]['mode']))
    for n, k in notes:
        print('        %dx %s' % (k, n[:70] if n else '(无)'))

# ---------- 4) 人靠近障碍时机器人的行为 ----------
print('\n=== 4) 机器人离障碍最近距离(dpath) 与 指令速度 的关系 ===')
bins = [(0, 0.4), (0.4, 0.6), (0.6, 0.8), (0.8, 1.2), (1.2, 1.6), (1.6, 99)]
for lo, hi in bins:
    seg = [r for r in rows if lo <= r['dp'] < hi]
    if not seg:
        continue
    zero = sum(1 for r in seg if r['v'] <= 0.02)
    print('  dpath %.1f~%.1f : n=%3d  停住%.0f%%  平均v=%.3f'
          % (lo, hi, len(seg), 100.0 * zero / len(seg),
             sum(r['v'] for r in seg) / len(seg)))

# ---------- 5) 关键: 障碍在侧前方时 d_head 是否被侧方距离拖累 ----------
print('\n=== 5) 急停/限速触发时的几何 ===')
stop_hit = [r for r in rows if r['note'].startswith('全向急停')]
lim_hit = [r for r in rows if r['note'].startswith('全向限速')]
wing = [r for r in rows if r['note'].startswith('侧翼')]
print('  全向急停 %d 拍, 全向限速 %d 拍, 侧翼限速 %d 拍' % (len(stop_hit), len(lim_hit), len(wing)))
if stop_hit:
    print('  急停时 dpath 中位 %.2f, dfront 中位 %.2f, dfl 中位 %.2f, dfr 中位 %.2f'
          % (sorted(r['dp'] for r in stop_hit)[len(stop_hit) // 2],
             sorted(r['df'] for r in stop_hit)[len(stop_hit) // 2],
             sorted(r['dfl'] for r in stop_hit)[len(stop_hit) // 2],
             sorted(r['dfr'] for r in stop_hit)[len(stop_hit) // 2]))
