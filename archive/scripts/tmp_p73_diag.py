import re
import sys

PATH = '/home/earl/workspace/VLA/vla_ws/logs/demo.log'
LIM = re.compile(r'全向限速 前([\d.]+) 左前([\d.]+) 右前([\d.]+) v<=([\d.]+)')
DST = re.compile(r'距离=([\d.]+)m')
BRG = re.compile(r'目标方位=([+-]\d+)')
CMD = re.compile(r'指令v=([+-][\d.]+)')

rows = []
cnt = {'限速': 0, '急停': 0, '侧翼': 0, '真值贴身': 0, '总行': 0}
for line in open(PATH, encoding='utf-8', errors='replace'):
    if 'FOLLOWER [' not in line:
        continue
    cnt['总行'] += 1
    d = DST.search(line)
    if not d:
        continue
    dist = float(d.group(1))
    b = BRG.search(line)
    brg = int(b.group(1)) if b else None
    c = CMD.search(line)
    cmd = float(c.group(1)) if c else None
    m = LIM.search(line)
    lim = None
    if m:
        cnt['限速'] += 1
        lim = (float(m.group(1)), float(m.group(2)), float(m.group(3)),
               float(m.group(4)))
    if '全向急停' in line:
        cnt['急停'] += 1
    if '侧翼限速' in line:
        cnt['侧翼'] += 1
    if '真值贴身' in line:
        cnt['真值贴身'] += 1
    rows.append((dist, brg, cmd, lim, line))

print('总 FOLLOWER 行 %d' % cnt['总行'])
for k in ('限速', '急停', '侧翼', '真值贴身'):
    print('  %-6s %d 行 (%.1f%%)' % (k, cnt[k], 100.0 * cnt[k] / max(1, cnt['总行'])))

# 只看限速发生时的"前向深度 vs 视觉距离"
lim_rows = [r for r in rows if r[3] is not None]
if lim_rows:
    gaps = [r[3][0] - (r[0] - 0.30) for r in lim_rows]
    gaps.sort()
    print('\n限速行 %d 条: d_f 与 (视觉距离-0.30) 之差' % len(lim_rows))
    print('  中位 %+.3f   p10 %+.3f  p90 %+.3f   |差|>0.45 占比 %.1f%%'
          % (gaps[len(gaps) // 2], gaps[int(.1 * len(gaps))],
             gaps[int(.9 * len(gaps))],
             100.0 * sum(1 for g in gaps if abs(g) > 0.45) / len(gaps)))
    print('\n  视觉距离分桶 -> 平均 d_f / 平均 v<= / 条数')
    buck = {}
    for r in lim_rows:
        k = round(min(r[0], 2.5) * 2) / 2.0
        buck.setdefault(k, []).append(r[3])
    for k in sorted(buck):
        v = buck[k]
        print('   d=%.1f  n=%4d  d_f均 %.2f  v<=均 %.2f'
              % (k, len(v), sum(x[0] for x in v) / len(v),
                 sum(x[3] for x in v) / len(v)))

# 全量: 视觉距离 vs 指令v
print('\n全量 视觉距离分桶 -> 平均指令v / 平均|brg|')
buck = {}
for dist, brg, cmd, lim, _ in rows:
    if cmd is None:
        continue
    k = round(min(dist, 2.5) * 2) / 2.0
    buck.setdefault(k, []).append((cmd, abs(brg) if brg is not None else -1))
for k in sorted(buck):
    v = buck[k]
    print('   d=%.1f  n=%4d  cmd均 %+.3f  |brg|均 %.0f°'
          % (k, len(v), sum(x[0] for x in v) / len(v),
             sum(x[1] for x in v) / len(v)))
