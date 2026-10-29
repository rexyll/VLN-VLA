#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""局部路径规划器原型 —— 纯几何, 不起仿真, 用于验证 A* 与安全间隙。

设计要点(与 follower 内的实现保持一致):
  1. 栅格以"机器人+目标"的包围盒为窗口, 分辨率 0.10m;
  2. 占据判据 = 车心到障碍表面距离 < ROBOT_R + clear_m  (Minkowski 膨胀的
     精确形式, 不是按外接圆粗略放大);
  3. 起点被挤进膨胀区时分级放宽 clear, 否则"已经贴住"时永远无解;
  4. 目标被挤进膨胀区(人贴障碍站着)时, 回退到"最近的可达格" —— 这就是
     需求里的"到达目标附近"而不是"到达目标点";
  5. 路径做 string-pulling 平滑, 平滑必须重新验证间隙, 不能只看端点。
"""
import heapq
import math

WALL_HARD = 5.75
ROBOT_R = 0.18
GEO = [
    ('box', 3.5, 2.0, 0.40, 0.40, 0.0),      # ob1 0.8x0.8
    ('box', -3.0, 3.2, 0.60, 0.30, 0.5),     # ob2 1.2x0.6 yaw0.5
    ('cyl', 0.5, -3.2, 0.40, 0.0, 0.0),      # ob3 r0.40
    ('cyl', -4.2, -2.5, 0.50, 0.0, 0.0),     # ob4 r0.50
    ('box', 4.2, -3.0, 0.30, 0.30, 0.0),     # ob5 0.6x0.6
    ('box', -1.5, -4.5, 0.50, 0.25, -0.4),   # ob6 1.0x0.5 yaw-0.4
    ('cyl', 2.2, 4.6, 0.35, 0.0, 0.0),       # ob7 r0.35
]


def geo_gap(x, y, g):
    k, ox, oy, a, b, ya = g
    dx, dy = x - ox, y - oy
    if ya:
        c, sn = math.cos(-ya), math.sin(-ya)
        dx, dy = dx * c - dy * sn, dx * sn + dy * c
    if k == 'cyl':
        return math.hypot(dx, dy) - a
    qx, qy = abs(dx) - a, abs(dy) - b
    return math.hypot(max(qx, 0.0), max(qy, 0.0))


def clearance(x, y):
    """车心到最近障碍表面的距离(负=在障碍内)。"""
    m = 99.0
    for g in GEO:
        gp = geo_gap(x, y, g)
        if gp < m:
            m = gp
    return m


def seg_min_clearance(p, q, step=0.04):
    """线段上的最小间隙 —— 平滑与路径校验都用这个。"""
    L = math.hypot(q[0] - p[0], q[1] - p[1])
    n = max(1, int(L / step))
    m = 99.0
    for i in range(n + 1):
        t = i / n
        x = p[0] + (q[0] - p[0]) * t
        y = p[1] + (q[1] - p[1]) * t
        if abs(x) > WALL_HARD or abs(y) > WALL_HARD:
            return -1.0
        c = clearance(x, y)
        if c < m:
            m = c
    return m


NB = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
      (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142))


def plan2(rx, ry, gx, gy, clear_m=0.20, res=0.10, pad=0.9, soft=0.22,
          max_cells=48):
    """三级规划: 先按 clear_m, 失败则减半, 最后只要求不穿模。

    为什么必须分级: 机器人已经被挤进膨胀区时(上一帧的避障把它推到临界),
    若仍按 clear_m 判占据, 起点周围全是"禁止格", A* 一步都走不出去 ——
    原型 T6 就是这个结果(路径长度 0)。实际系统里这种情况必然出现, 因为
    8 扇区测距有 ±0.1m 误差, 安全层允许的最小值可能略低于规划阈值。

    返回 (pts, reached, level)。level 是实际使用的间隙要求:
      clear_m  → 理想;  clear_m/2 → 偏紧(上层降速);
      0.04     → 仅逃逸(上层限速到 crawl)。
    """
    for lv in (clear_m, clear_m * 0.5, 0.04):
        pts, reached, info = plan(rx, ry, gx, gy, lv, res, pad, soft,
                                  max_cells)
        if info == 'ok':
            return pts, reached, lv
    return [(rx, ry), (rx, ry)], False, -1.0


def plan(rx, ry, gx, gy, clear_m=0.20, res=0.10, pad=0.9, soft=0.22,
         max_cells=60):
    """返回 (pts, reached_goal, info)。

    pts: 世界系折线, 首点=机器人位置, 末点=可达的最近目标位。
    reached_goal: 是否真的到了目标格(False 表示"目标不可达, 到附近")。
    """
    ai = max(0.0, ROBOT_R + clear_m)
    # 窗口: 覆盖两端并留 pad, 但限制格数
    x0 = min(rx, gx) - pad
    x1 = max(rx, gx) + pad
    y0 = min(ry, gy) - pad
    y1 = max(ry, gy) + pad
    while (x1 - x0) / res > max_cells or (y1 - y0) / res > max_cells:
        res *= 1.25
    nx = int((x1 - x0) / res) + 1
    ny = int((y1 - y0) / res) + 1

    def w2g(x, y):
        return (int(round((x - x0) / res)), int(round((y - y0) / res)))

    def g2w(i, j):
        return (x0 + i * res, y0 + j * res)

    # 预计算每格的间隙(车心到表面)
    cell_gap = [[99.0] * ny for _ in range(nx)]
    for i in range(nx):
        wx = x0 + i * res
        for j in range(ny):
            wy = y0 + j * res
            if abs(wx) > WALL_HARD or abs(wy) > WALL_HARD:
                cell_gap[i][j] = -9.0
            else:
                cell_gap[i][j] = clearance(wx, wy)

    si, sj = w2g(rx, ry)
    gi, gj = w2g(gx, gy)
    si = max(0, min(nx - 1, si)); sj = max(0, min(ny - 1, sj))
    gi = max(0, min(nx - 1, gi)); gj = max(0, min(ny - 1, gj))

    # --- 起点可行性: 交给 plan2 降级重试, 不在同一级里放宽 ---
    # 若这里放宽, 起点格可通行但邻格仍要求 ai, 等于一步都走不出去。
    if cell_gap[si][sj] < ai:
        return [(rx, ry), (rx, ry)], False, '起点在膨胀区'
    a_start = ai

    # --- 终点: 若目标格在膨胀区内, 找最近的可达格 ---
    reached = cell_gap[gi][gj] >= ai
    if not reached:
        best, bd = None, 1e9
        rad = int(1.6 / res)
        for i in range(max(0, gi - rad), min(nx, gi + rad + 1)):
            for j in range(max(0, gj - rad), min(ny, gj + rad + 1)):
                if cell_gap[i][j] < ai:
                    continue
                d = math.hypot((i - gi) * res, (j - gj) * res)
                if d < bd:
                    bd, best = d, (i, j)
        if best is None:
            return ([(rx, ry), (rx, ry)], False, '目标无可达格')
        gi, gj = best

    def h(i, j):
        dx, dy = abs(i - gi), abs(j - gj)
        return (dx + dy) + (1.4142 - 2.0) * min(dx, dy)

    # --- A* ---
    INF = 1e18
    gcost = [[INF] * ny for _ in range(nx)]
    prev = [[None] * ny for _ in range(nx)]
    gcost[si][sj] = 0.0
    pq = [(h(si, sj), 0.0, si, sj)]
    closed = [[False] * ny for _ in range(nx)]
    while pq:
        _, gc, i, j = heapq.heappop(pq)
        if closed[i][j]:
            continue
        closed[i][j] = True
        if (i, j) == (gi, gj):
            break
        for di, dj, dc in NB:
            ni, nj = i + di, j + dj
            if not (0 <= ni < nx and 0 <= nj < ny) or closed[ni][nj]:
                continue
            need = a_start if (ni, nj) == (gi, gj) else ai
            if cell_gap[ni][nj] < need:
                continue
            if di and dj:      # 不穿越角落
                if cell_gap[i + di][j] < ai or cell_gap[i][j + dj] < ai:
                    continue
            # 软代价: 贴近膨胀边界时加价, 让路径走中间
            over = cell_gap[ni][nj] - ai
            k = 1.0
            if over < soft:
                k = 1.0 + 2.2 * (1.0 - over / soft)
            ng = gc + dc * res * k
            if ng < gcost[ni][nj]:
                gcost[ni][nj] = ng
                prev[ni][nj] = (i, j)
                heapq.heappush(pq, (ng + h(ni, nj), ng, ni, nj))

    if gcost[gi][gj] >= INF:
        return ([(rx, ry), (rx, ry)], False, 'A*无解')

    # 回溯
    cells = []
    cur = (gi, gj)
    while cur is not None:
        cells.append(cur)
        cur = prev[cur[0]][cur[1]]
    cells.reverse()
    pts = [g2w(i, j) for (i, j) in cells]
    pts[0] = (rx, ry)

    # --- string pulling: 能直连就跳过中间点 ---
    out = [pts[0]]
    k = 0
    while k < len(pts) - 1:
        nxt = k + 1
        for m in range(len(pts) - 1, k + 1, -1):
            if seg_min_clearance(pts[k], pts[m]) >= ai - 1e-6:
                nxt = m
                break
        out.append(pts[nxt])
        k = nxt
    return (out, reached, 'ok')


def path_clear(pts):
    m = 99.0
    for a, b in zip(pts, pts[1:]):
        m = min(m, seg_min_clearance(a, b))
    return m - ROBOT_R


def report(tag, rx, ry, gx, gy, clear=0.20):
    pts, reached, info = plan2(rx, ry, gx, gy, clear)
    L = sum(math.hypot(b[0] - a[0], b[1] - a[1])
            for a, b in zip(pts, pts[1:]))
    gap = path_clear(pts)
    print('%s' % tag)
    print('   直线%.2fm  路径%.2fm  %d点  到达=%s  最小间隙%+.2fm  级别%.2f'
          % (math.hypot(gx - rx, gy - ry), L, len(pts), reached, gap, info))
    print('   ' + ' → '.join('(%.2f,%.2f)' % p for p in pts))
    return pts, gap


if __name__ == '__main__':
    report('T1 空旷直线', 0, 0, 3.0, 0.0)
    report('T2 人站 ob2 正后方', 0.0, 0.5, -4.6, 4.4)
    report('T3 eval14 死锁现场(ob6 挡在连线上)', -2.60, -5.03, -0.43, -4.95)
    report('T4 ob5 角落内侧', 2.0, -1.0, 4.9, -2.6)
    report('T5 人贴 ob3 站着(不可达)', -1.5, -3.2, 0.85, -3.2)
    report('T6 起点已在膨胀区(贴 ob2)', -2.35, 2.95, -0.5, 1.0)
    report('T7 人贴墙角落', 0.0, 0.0, 5.0, 5.0)
    report('T8 两侧夹击(ob3 与 ob6 之间)', 1.8, -4.9, -0.2, -2.4)
    report('T9 clear=0.35 更保守', -2.60, -5.03, -0.43, -4.95, 0.35)
