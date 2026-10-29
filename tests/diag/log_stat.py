#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_logstat.py —— 从 follower 日志统计跟随质量与转向振荡

用法: python3 tmp_logstat.py demo_show.log

输出:
  1) 距离跟踪: 目标 1.00m 的均值/标准差/带内占比
  2) 对正带: |目标方位| <= 15° 的占比(aim_band_deg)
  3) 转向振荡: w 符号翻转次数/秒, 以及"绕左/绕右"翻转次数
  4) 安全层介入统计: 急停 / 限速 / 后退脱困 / 感知中断
"""
import re
import sys

PATH = sys.argv[1] if len(sys.argv) > 1 else 'demo_show.log'

RE_T = re.compile(r'\[(\d+\.\d+)\]')
RE_D = re.compile(r'距离=([\d.]+)m')
RE_B = re.compile(r'目标方位=([+-]?\d+)°')
RE_W = re.compile(r'\bw=([+-]?[\d.]+)')
RE_MODE = re.compile(r'\[(跟踪|对正)\|')

rows = []
side_seq = []
cnt = {'急停': 0, '限速': 0, '脱困': 0, '中断': 0, '绕左': 0, '绕右': 0}

for line in open(PATH, encoding='utf-8', errors='replace'):
    if 'FOLLOWER' not in line:
        continue
    for k, pat in (('急停', '全向急停'), ('限速', '全向限速'),
                   ('脱困', '后退脱困'), ('中断', '感知中断'),
                   ('绕左', '绕左'), ('绕右', '绕右')):
        if pat in line:
            cnt[k] += 1
    mt = RE_T.search(line)
    if not mt:
        continue
    t = float(mt.group(1))
    for side in ('绕左', '绕右'):
        if side in line:
            side_seq.append((t, side))
    md = RE_D.search(line)
    if not md:
        continue
    d = float(md.group(1))
    mb = RE_B.search(line)
    b = float(mb.group(1)) if mb else None
    mw = RE_W.search(line)
    w = float(mw.group(1)) if mw else None
    mode = RE_MODE.search(line)
    rows.append((t, d, b, w, mode.group(1) if mode else '?'))

if not rows:
    print('!! 未解析到 FOLLOWER 状态行')
    sys.exit(1)

t0, t1 = rows[0][0], rows[-1][0]
span = max(t1 - t0, 1e-6)
ds = [r[1] for r in rows]
bs = [abs(r[2]) for r in rows if r[2] is not None]
n = len(ds)
mean_d = sum(ds) / n
sd = (sum((x - mean_d) ** 2 for x in ds) / n) ** 0.5
in_band = sum(1 for x in ds if abs(x - 1.0) <= 0.15) / n
track = sum(1 for r in rows if r[4] == '跟踪') / n

print('=' * 74)
print(f'日志 {PATH}  {n} 行  {t0:.0f}~{t1:.0f} ({span:.0f}s)')
print(f'  距离跟踪   均值 {mean_d:.3f}m  标准差 {sd:.3f}  1.00±0.15 占比 '
      f'{in_band:.1%}')
print(f'  对正带     平均|方位| {sum(bs) / len(bs):.1f}°  '
      f'≤15° 占比 {sum(1 for x in bs if x <= 15) / len(bs):.1%}')
print(f'  跟踪态占比 {track:.1%}   对正态占比 {1 - track:.1%}')

# w 符号翻转(排除 |w|<0.05 的死区, 否则噪声会把翻转数灌满)
flip, last = 0, None
for r in rows:
    w = r[3]
    if w is None or abs(w) < 0.05:
        continue
    s = 1 if w > 0 else -1
    if last is not None and s != last:
        flip += 1
    last = s
print(f'  转向振荡   w 符号翻转 {flip} 次  ({flip / span:.2f} 次/秒)')

sf = 0
for i in range(1, len(side_seq)):
    if side_seq[i][1] != side_seq[i - 1][1]:
        sf += 1
print(f'  绕向切换   绕左/绕右 切换 {sf} 次  ({len(side_seq)} 条绕向日志)')

print('  安全层介入 ' + ' '.join(f'{k}={v}' for k, v in cnt.items()))
print('=' * 74)
