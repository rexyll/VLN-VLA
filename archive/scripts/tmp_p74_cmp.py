#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_p74_cmp.py —— 比对两轮同种子轨迹

1) 先在 ±SHIFT 秒内搜最优时间对齐(两轮节点启动时刻相对仿真零点可能差几秒)
2) 再按 0.1s 栅格算逐点偏差
判定线: max|Δ| < 0.05m
"""
import json
import sys

SHIFT = 4.0
STEP = 0.1


def load(p):
    d = json.load(open(p))
    r = d['recs']
    return [x[0] for x in r], [(x[1], x[2]) for x in r], d


def interp(ts, xy, t):
    if t <= ts[0]:
        return xy[0]
    if t >= ts[-1]:
        return xy[-1]
    lo, hi = 0, len(ts) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if ts[mid] <= t:
            lo = mid
        else:
            hi = mid
    if ts[hi] == ts[lo]:
        return xy[lo]
    k = (t - ts[lo]) / (ts[hi] - ts[lo])
    return (xy[lo][0] + k * (xy[hi][0] - xy[lo][0]),
            xy[lo][1] + k * (xy[hi][1] - xy[lo][1]))


def scan(ta, xa, tb, xb, shift):
    """B 的时间轴整体平移 shift 后与 A 比较, 返回 (mean, max)"""
    t_end = min(ta[-1], tb[-1] - shift)
    t = max(0.0, -shift) + 1.0          # 跳过启动瞬态 1s
    acc = 0.0
    mx = 0.0
    n = 0
    while t <= t_end:
        a = interp(ta, xa, t)
        b = interp(tb, xb, t + shift)
        d = ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
        acc += d
        mx = max(mx, d)
        n += 1
        t += STEP
    return (acc / max(1, n), mx, n, t_end)


def main():
    pa, pb = sys.argv[1], sys.argv[2]
    ta, xa, da = load(pa)
    tb, xb, db = load(pb)
    print('A: %s n=%d dur=%.1f  start=(%.3f,%.3f)'
          % (pa, len(ta), ta[-1], xa[0][0], xa[0][1]))
    print('B: %s n=%d dur=%.1f  start=(%.3f,%.3f)'
          % (pb, len(tb), tb[-1], xb[0][0], xb[0][1]))

    best = None
    s = -SHIFT
    while s <= SHIFT + 1e-9:
        m, mx, n, te = scan(ta, xa, tb, xb, s)
        if best is None or m < best[1]:
            best = (s, m, mx, n, te)
        s += 0.05
    sh_b, mean, mx, n, te = best
    print('最优时间对齐 shift=%+.2fs, 公共时长 %.1fs, 采样 %d' % (sh_b, te, n))
    print('mean |Δ| = %.4f m' % mean)
    print('max  |Δ| = %.4f m' % mx)

    # 逐点明细, 找首次分叉
    t = 1.0
    first = None
    while t <= te:
        a = interp(ta, xa, t)
        b = interp(tb, xb, t + sh_b)
        d = ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
        if first is None and d > 0.05:
            first = t
            break
        t += STEP
    print('首次 >0.05m: %s' % ('%.1fs' % first if first else '从未超过'))
    if mx < 0.05:
        print('判定: PASS —— 评测可重复, 后续可做 A/B')
    elif mx < 0.30:
        print('判定: MARGINAL —— 大幅改善但未达 0.05m')
    else:
        print('判定: FAIL —— 仍不可重复')


if __name__ == '__main__':
    main()
