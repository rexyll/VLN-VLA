#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_p74_cmp2.py —— 以"人第一次起步"为原点对齐后比对两轮轨迹

上一版按录制器首帧对齐, 但录制起点相对人的起点是随机的(演示启动耗时不定),
两轮起点差 2m 以上, 属于对齐错误而非物理不确定。这里用速度剖面找起步时刻。
"""
import json
import sys

STEP = 0.1


def load(p):
    d = json.load(open(p))
    return [[x[0], x[1], x[2]] for x in d['recs']]


def interp(tr, t):
    if t <= tr[0][0]:
        return tr[0][1], tr[0][2]
    if t >= tr[-1][0]:
        return tr[-1][1], tr[-1][2]
    lo, hi = 0, len(tr) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if tr[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    d = tr[hi][0] - tr[lo][0]
    if d <= 0:
        return tr[lo][1], tr[lo][2]
    k = (t - tr[lo][0]) / d
    return (tr[lo][1] + k * (tr[hi][1] - tr[lo][1]),
            tr[lo][2] + k * (tr[hi][2] - tr[lo][2]))


def motion_start(tr, vth=0.05, hold=0.4):
    """返回人开始持续移动的时刻"""
    t = tr[0][0]
    while t < tr[-1][0] - hold:
        a = interp(tr, t)
        b = interp(tr, t + hold)
        d = ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
        if d / hold > vth:
            return t
        t += 0.05
    return tr[0][0]


def main():
    ta = load(sys.argv[1])
    tb = load(sys.argv[2])
    sa = motion_start(ta)
    sb = motion_start(tb)
    print('录制窗口: A %.1f~%.1fs  B %.1f~%.1fs'
          % (ta[0][0], ta[-1][0], tb[0][0], tb[-1][0]))
    print('起步时刻: A %.2fs  B %.2fs (相对各自录制的首帧)' % (sa, sb))
    pa = interp(ta, sa)
    pb = interp(tb, sb)
    print('起步位置: A=(%.3f,%.3f)  B=(%.3f,%.3f)  差=%.3fm'
          % (pa[0], pa[1], pb[0], pb[1],
             ((pa[0] - pb[0]) ** 2 + (pa[1] - pb[1]) ** 2) ** 0.5))

    dur = min(ta[-1][0] - sa, tb[-1][0] - sb)
    t = 0.0
    worst = 0.0
    worst_t = 0.0
    acc = 0.0
    n = 0
    first = None
    while t <= dur:
        a = interp(ta, sa + t)
        b = interp(tb, sb + t)
        d = ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
        acc += d
        n += 1
        if d > worst:
            worst, worst_t = d, t
        if first is None and d > 0.05:
            first = t
        t += STEP
    print('可比时长 %.1fs, 采样 %d' % (dur, n))
    print('mean |Δ| = %.4f m' % (acc / max(1, n)))
    print('max  |Δ| = %.4f m  @ 起步后 %.1fs' % (worst, worst_t))
    print('首次 >0.05m: %s' % ('起步后 %.1fs' % first if first is not None
                              else '从未超过'))
    if worst < 0.05:
        print('判定: PASS')
    elif worst < 0.30:
        print('判定: MARGINAL')
    else:
        print('判定: FAIL')


if __name__ == '__main__':
    main()
