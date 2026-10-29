#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""_p103_draw.py —— 把关节角画成"俯视 + 正视"两张骨架图, 用来看肘朝哪。

平面约定(本体系, 机器人在原点、面朝 +x):
    俯视(左图): 横轴 = y(体侧, 左正), 纵轴 = x(前)
    正视(右图): 横轴 = y(体侧, 左正), 纵轴 = z(上)
图中 ○=肩  ●=肘  ◆=腕  ▲=掌心, 粗线=上臂, 细线=前臂, 虚线=手掌朝向。

用法:
    python3 tests/diag/_p103_draw.py            # 画现网 q 与 human 解对照
    python3 tests/diag/_p103_draw.py <json>     # 画 diag_p103.json 里的实测 q
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, 'vla_ws/src/vla_follow')
import matplotlib                              # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt                # noqa: E402

from vla_follow import arm_kin as K            # noqa: E402
from vla_follow import drink_poses as P        # noqa: E402
from vla_follow import scene_layout as L       # noqa: E402

OUT = 'vla_ws/logs/p103_arms.png'


def chain(side, q):
    """返回 [肩, 肘, 腕, 掌心] 的本体系坐标 + 掌系 z 轴(指向手指)。"""
    pts = [np.asarray(K.SHOULDER[side], float)]
    T = K.frame(t=pts[0])
    wrist_pt = None
    for i, (_n, off, axis, _lim) in enumerate(K.JOINTS):
        T = T @ K.trans(off)
        if i == 3:
            wrist_pt = T[:3, 3].copy()
        T = T @ K.frame(K.rot(axis, q[i]))
    pts.append(K.elbow_pos(side, q))
    pts.append(wrist_pt)
    pts.append(T[:3, 3].copy())
    return pts, T[:3, 2].copy()


def draw(ax_top, ax_front, side, q, color, label):
    pts, fdir = chain(side, q)
    sh, el, wr, pa = pts
    ax_top.plot([sh[1], el[1]], [sh[0], el[0]], '-', color=color, lw=3,
                label=label)
    ax_top.plot([el[1], wr[1], pa[1]], [el[0], wr[0], pa[0]], '-', color=color,
                lw=1.4)
    ax_top.plot([pa[1], pa[1] + 0.05 * fdir[1]], [pa[0], pa[0] + 0.05 * fdir[0]],
                ':', color=color, lw=1.0)
    ax_front.plot([sh[1], el[1]], [sh[2], el[2]], '-', color=color, lw=3)
    ax_front.plot([el[1], wr[1], pa[1]], [el[2], wr[2], pa[2]], '-', color=color,
                  lw=1.4)
    for a in (ax_top, ax_front):
        a.plot(el[1], el[0] if a is ax_top else el[2], 'o', color=color, ms=6)
        a.plot(pa[1], pa[0] if a is ax_top else pa[2], '^', color=color, ms=6)
    ax_top.plot(sh[1], sh[0], 'o', mfc='none', color=color, ms=8)


def main():
    bx, by, _ = L.SERVE_POSE
    base = (bx, by, 0.0)
    Tcp = P.cup_for_pour()
    cup0 = P.cup_on_table()

    cases = []
    if len(sys.argv) > 1:
        d = json.load(open(sys.argv[1]))
        g = (d.get('pour') or {}).get('grasp_cup') or {}
        l = (d.get('pour') or {}).get('lift_cup') or {}
        if g.get('q'):
            cases.append(('L', np.array(g['q']), 'C3', '实测 抓杯'))
        if l.get('q'):
            cases.append(('L', np.array(l['q']), 'C1', '实测 举杯'))
    else:
        q = K.Q_REST.copy()
        for tag, T in (('抓杯', P.cup_grasp_palm(cup0, 'L')),
                       ('举杯', P.cup_grasp_palm(Tcp, 'L'))):
            q, _e, _h = K.ik_world('L', T, base, q, elbow='human')
            cases.append(('L', q.copy(), 'C0', 'human ' + tag))

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.6))
    ax_top, ax_front = axes
    for side, q, c, lab in cases:
        draw(ax_top, ax_front, side, q, c, lab)

    # 目标点(世界 → base)
    for T, name, c in ((cup0, '杯(桌面)', 'k'), (Tcp, '杯(举杯位)', 'g')):
        p = np.linalg.inv(K.base_pose(*base)) @ np.asarray(T, float)
        ax_top.plot(p[1, 3], p[0, 3], 'x', color=c, ms=9)
        ax_top.annotate(name, (p[1, 3], p[0, 3]), fontsize=7,
                        xytext=(3, 3), textcoords='offset points')
        ax_front.plot(p[1, 3], p[2, 3], 'x', color=c, ms=9)

    ax_top.axvline(0.0, color='0.7', lw=0.8, ls='--')
    ax_front.axvline(0.0, color='0.7', lw=0.8, ls='--')
    ax_top.set_xlabel('y / m  (左为 +)')
    ax_top.set_ylabel('x / m  (前为 +)')
    ax_top.set_title('俯视: 肘在体侧还是内收')
    ax_top.set_aspect('equal')
    ax_top.grid(alpha=0.3)
    ax_top.legend(fontsize=7)
    ax_front.set_xlabel('y / m  (左为 +)')
    ax_front.set_ylabel('z / m')
    ax_front.set_title('正视: 肘的高度与左右')
    ax_front.set_aspect('equal')
    ax_front.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(OUT, dpi=130)
    print('写入', OUT)
    for side, q, c, lab in cases:
        m = K.elbow_metrics(side, q)
        el = K.elbow_pos(side, q)
        print('%-10s 肘y%+6.0f 掌y%+6.0f 外扩线%+6.0f 基外扩%+6.0f droop%+6.0f '
              'up_roll%+5.2f 肘%+5.2f twist%+5.2f %s'
              % (lab, el[1] * 1e3, K.fk(side, q)[:3, 3][1] * 1e3,
                 m['out_line_mm'], m['out_base_mm'], m['droop_mm'],
                 q[2], q[3], q[4] + q[6], K.violation_list(side, q)))


if __name__ == '__main__':
    main()
