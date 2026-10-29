#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_p74_cmp3.py —— 以「人离开出生点 0.10m」为原点对齐后比对两轮轨迹

前两版的错: 按录制首帧对齐(录制起点相对人的起点随机), 或用速度阈值找起步
(加速度爬升期判不出来, A 轮迟判到人已走 3m)。人的出生点固定, 用位移判据最稳。
"""
import json
import sys

STEP = 0.1
THRESH = 0.10


def load(p):
    d = json.load(open(p))
    return [[x[0], x[1], x[2]] for x in d['recs']], d.get('resets', 0)


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


def leave_point(tr, thresh=THRESH):
    sp = (tr[0][1], tr[0][2])
    for row in tr:
        if ((row[1] - sp[0]) ** 2 + (row[2] - sp[1]) ** 2) ** 0.5 > thresh:
            return row[0], sp
    return tr[0][0], sp


def main():
    ta, ra = load(sys.argv[1])
    tb, rb = load(sys.argv[2])
    print('A: %s  n=%d span=%.1fs resets=%d' % (sys.argv[1], len(ta), ta[-1][0], ra))
    print('B: %s  n=%d span=%.1fs resets=%d' % (sys.argv[2], len(tb), tb[-1][0], rb))
    sa, spa = leave_point(ta)
    sb, spb = leave_point(tb)
    print('出生点: A=(%.3f,%.3f)  B=(%.3f,%.3f)' % (spa[0], spa[1], spb[0], spb[1]))
    print('离出生点 %.2fm 时刻: A %.1fs  B %.1fs (相对各自录制首帧)' % (THRESH, sa, sb))

    dur = min(ta[-1][0] - sa, tb[-1][0] - sb)
    t = 0.0
    worst = 0.0
    worst_t = 0.0
    acc = 0.0
    n = 0
    first = None
    marks = {}
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
        for m in (10, 20, 40, 60, 80):
            if m not in marks and t >= m:
                marks[m] = d
        t += STEP
    print('可比时长 %.1fs, 采样 %d' % (dur, n))
    print('mean |Δ| = %.4f m' % (acc / max(1, n)))
    print('max  |Δ| = %.4f m  @ 起步后 %.1fs' % (worst, worst_t))
    print('首次 >0.05m: %s' % ('起步后 %.1fs' % first if first is not None
                              else '从未超过'))
    print('分段: ' + '  '.join('%ds:%.3f' % (k, v)
                              for k, v in sorted(marks.items())))
    if worst < 0.05:
        print('判定: PASS —— 评测可重复')
    elif worst < 0.30:
        print('判定: MARGINAL —— 大幅改善, 未达 0.05m')
    else:
        print('判定: FAIL')


if __name__ == '__main__':
    main()
