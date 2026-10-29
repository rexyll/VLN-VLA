#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_p76_scn.py —— 造一条"人绕立柱走、并在柱旁站定"的回放轨迹

用途: 复现用户描述的"目标移动到障碍物附近, 机器人跟过去后在柱子旁徘徊不前"。
人的出生位姿 (1, 0, yaw=0)。立柱 ob3 在 (0.5, -3.2), 半径 0.4。

路线: (1,0) → 南行到 (1,-2.2) → 绕到东侧 (1.6,-3.6) → 折向西南到 (0.5,-4.3)
      → 站定 45s
最后人距柱面 0.60m, 且从机器人的来向看, 柱子正好挡在人与机器人之间。
用单轮运动学积分校验终点, 再写成 PW_REPLAY 需要的 jsonl。
"""
import json
import math

X0, Y0, YAW0 = 1.0, 0.0, 0.0
DT = 0.05


def seg(out, t, v, w, dur):
    n = int(round(dur / DT))
    for i in range(n):
        out.append((round(t + i * DT, 4), v, w))
    return t + n * DT


def route():
    cmds = []
    t = 0.0
    t = seg(cmds, t, 0.0, 0.0, 3.0)             # 等待
    t = seg(cmds, t, 0.0, -0.60, 2.62)          # 原地转到朝 -y
    t = seg(cmds, t, 0.33, 0.0, 6.67)           # 南行 2.2m
    t = seg(cmds, t, 0.10, 0.40, 1.00)          # 转向东南
    t = seg(cmds, t, 0.30, 0.0, 5.10)           # 绕到 (1.6,-3.6)
    t = seg(cmds, t, 0.08, -0.60, 2.35)         # 转向西南
    t = seg(cmds, t, 0.30, 0.0, 4.35)           # 走到 (0.5,-4.3)
    t = seg(cmds, t, 0.0, 0.0, 45.0)            # 站定
    return cmds


def integrate(cmds):
    x, y, yaw = X0, Y0, YAW0
    pts = [(x, y)]
    for _, v, w in cmds:
        x += v * math.cos(yaw) * DT
        y += v * math.sin(yaw) * DT
        yaw += w * DT
        pts.append((x, y))
    return pts


cmds = route()
pts = integrate(cmds)
px, py = pts[-1]
print('终点 = (%.2f, %.2f)  yaw=%.2f rad' % (px, py, YAW0))
print('距立柱中心 %.2f m (柱面 %.2f m)' %
      (math.hypot(px - 0.5, py + 3.2), math.hypot(px - 0.5, py + 3.2) - 0.4))
mind = min(math.hypot(q[0] - 0.5, q[1] + 3.2) for q in pts)
print('路径上离柱心最近 %.2f m (柱面 %.2f m) —— 需 > 0.6 才不穿模'
      % (mind, mind - 0.4))

with open('/tmp/scn_pillar.jsonl', 'w', encoding='utf-8') as f:
    f.write(json.dumps({'hdr': 1, 'seed': 'scn', 't0_abs': 0.0,
                        'rate_hz': 20.0}) + '\n')
    for t, v, w in cmds:
        f.write(json.dumps({'t': t, 'v': v, 'w': w}) + '\n')
print('写了 %d 拍, 总时长 %.1fs' % (len(cmds), cmds[-1][0]))
