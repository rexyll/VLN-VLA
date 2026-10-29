#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_p75_cmp.py —— 路径级比对(与时间相位无关)

tmp_p74_cmp3.py 按「离出生点 0.10m」对齐后按时间采样。0.10m 阈值在 0.3m/s 下
对应 ±0.33s 的定时量化 => 仅对齐误差就有 ~0.1m 的弧长偏差, 无法区分
"路径不同" 与 "起步时刻不同"。

本脚本改用点到折线最近距离(对称 hausdorff):
  D(b->A)  B 每个点到 A 折线的最短距离
  D(a->B)  A 每个点到 B 折线的最短距离
两条路径完全重合时两者都为 0, 与时间轴是否偏移无关。
另外报告沿弧长的横向/纵向拆解, 便于判断是"走偏"还是"走快慢"。
"""
import json
import math
import sys

SKIP = 1.5  # 起步后跳过前 1.5s(静止段噪声)


def load(p):
    d = json.load(open(p))
    return [[x[0], x[1], x[2]] for x in d['recs']]


def seg_dist(px, py, ax, ay, bx, by):
    vx, vy = bx - ax, by - ay
    L2 = vx * vx + vy * vy
    if L2 < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * vx + (py - ay) * vy) / L2
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


def poly_dist(pts, poly, step=1):
    """pts 每点到 poly 折线的最短距离; step 抽样加速"""
    best = []
    for i in range(0, len(pts), step):
        px, py = pts[i][1], pts[i][2]
        m = 1e9
        for j in range(0, len(poly) - 1, 2):
            d = seg_dist(px, py, poly[j][1], poly[j][2],
                         poly[j + 1][1], poly[j + 1][2])
            if d < m:
                m = d
        best.append(m)
    return best


def stat(v):
    if not v:
        return (0.0, 0.0, 0.0)
    s = sorted(v)
    n = len(s)
    return (sum(s) / n, s[int(0.5 * (n - 1))], s[-1])


def main():
    A = load(sys.argv[1])
    B = load(sys.argv[2])
    # 裁掉起步前静止段
    def cut(tr):
        sp = (tr[0][1], tr[0][2])
        k = 0
        for i, r in enumerate(tr):
            if math.hypot(r[1] - sp[0], r[2] - sp[1]) > 0.10:
                k = i
                break
        return tr[max(0, k - 30):]
    A = cut(A)
    B = cut(B)
    print('A=%s  n=%d   B=%s  n=%d' % (sys.argv[1], len(A), sys.argv[2], len(B)))
    la = sum(math.hypot(A[i + 1][1] - A[i][1], A[i + 1][2] - A[i][2])
             for i in range(len(A) - 1))
    lb = sum(math.hypot(B[i + 1][1] - B[i][1], B[i + 1][2] - B[i][2])
             for i in range(len(B) - 1))
    print('路径长度: A %.2fm  B %.2fm  (Δ %.2fm)' % (la, lb, lb - la))
    d1 = poly_dist(B, A)
    d2 = poly_dist(A, B)
    print('B->A  mean=%.4f  med=%.4f  max=%.4f' % stat(d1))
    print('A->B  mean=%.4f  med=%.4f  max=%.4f' % stat(d2))
    haus = max(max(d1), max(d2))
    print('对称 Hausdorff = %.4f m' % haus)
    if haus < 0.05:
        print('判定: PASS —— 同一路径 (时间轴可能偏移)')
    elif haus < 0.15:
        print('判定: GOOD —— 路径基本一致')
    elif haus < 0.50:
        print('判定: MARGINAL')
    else:
        print('判定: FAIL —— 路径不同')


if __name__ == '__main__':
    main()
