#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P58a: 在 follower.py 里插入 LocalPlanner(栅格 A* + 纯追踪)。"""
import io
import re

SRC = '_p57_follower.py'
DST = '_p58_follower.py'

s = io.open(SRC, encoding='utf-8').read()

# ---- 1. import heapq ----
if 'import heapq' not in s:
    s = s.replace('import json\nimport math',
                  'import heapq\nimport json\nimport math', 1)

PLANNER = '''

class LocalPlanner:
    """真值几何栅格 A* 局部规划 + 纯追踪 —— 需求 2/3/4 的执行体。

    为什么不沿用启发式扇区选择(P46~P52 试了 5 版全失败):
      扇区表锚定在**车头系**, 车头一转, "最优扇区"对应的世界方向就变了,
      于是角速度误差恒为正 -> 匀速转圈; 改成世界系锁存(P49/P52)则走偏
      之后不再修正。根因不是参数, 而是**没有一条一致、可跟踪的路径**。
      A* 给出的是世界系折线, 纯追踪只盯前面 0.5m 的点, 天然稳定。

    占据判据用 `geo_gap < ROBOT_R + clear` —— 这是 Minkowski 膨胀的精确
    形式(车心到障碍表面的距离场), 不用外接圆近似, 所以"能从旁边过去"
    的通道不会被误判成堵死。
    """

    NB = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
          (1, 1, 1.41421356), (1, -1, 1.41421356),
          (-1, 1, 1.41421356), (-1, -1, 1.41421356))

    def __init__(self, cfg):
        self.res = 0.10
        self.pad = 0.90
        self.max_cells = 48
        self.soft = 0.22
        self.pts = []          # 世界系折线, 首点 = 规划时的机器人位置
        self.goal = None
        self.level = 0.0       # 实际采用的间隙要求(clear/half/0.04)
        self.reached = False   # 是否真能走到目标格
        self.version = 0       # 每重规划 +1, 供可视化判断"是否需要重绘"
        self.t_plan = 0.0
        self.busy = False
        self.load_cfg(cfg)

    def load_cfg(self, cfg):
        self.clear_m = float(cfg['obstacle_clear_m'])
        self.period = float(cfg['plan_period_s'])
        self.goal_move = float(cfg['plan_goal_move_m'])
        self.lookahead_m = float(cfg['plan_lookahead_m'])

    # ---------- 几何 ----------
    @staticmethod
    def clearance(x, y):
        m = 99.0
        for g in GEO:
            gp = geo_gap(x, y, g)
            if gp < m:
                m = gp
        return m

    @classmethod
    def seg_clear(cls, p, q, step=0.05):
        L = math.hypot(q[0] - p[0], q[1] - p[1])
        n = max(1, int(L / step))
        m = 99.0
        for i in range(n + 1):
            t = i / n
            x = p[0] + (q[0] - p[0]) * t
            y = p[1] + (q[1] - p[1]) * t
            if abs(x) > WALL_HARD or abs(y) > WALL_HARD:
                return -1.0
            c = cls.clearance(x, y)
            if c < m:
                m = c
        return m

    # ---------- 是否需要重规划 ----------
    def need_replan(self, now, rx, ry, gx, gy):
        if not self.pts or self.goal is None:
            return True
        if now - self.t_plan > self.period:
            return True
        if math.hypot(gx - self.goal[0], gy - self.goal[1]) > self.goal_move:
            return True
        # 机器人被推到路径之外(避障/退让) -> 路径已失效
        if self._dist_to_path(rx, ry) > 0.55:
            return True
        return False

    def _dist_to_path(self, rx, ry):
        m = 1e9
        pts = self.pts
        for i in range(len(pts) - 1):
            a, b = pts[i], pts[i + 1]
            vx, vy = b[0] - a[0], b[1] - a[1]
            L2 = vx * vx + vy * vy
            if L2 < 1e-9:
                d = math.hypot(rx - a[0], ry - a[1])
            else:
                t = ((rx - a[0]) * vx + (ry - a[1]) * vy) / L2
                t = max(0.0, min(1.0, t))
                d = math.hypot(rx - (a[0] + vx * t), ry - (a[1] + vy * t))
            if d < m:
                m = d
        return m

    # ---------- 规划 ----------
    def plan_now(self, rx, ry, gx, gy, now):
        out = None
        for lv in (self.clear_m, self.clear_m * 0.5, 0.04):
            r = self._astar(rx, ry, gx, gy, lv)
            if r is not None:
                out = (r[0], r[1], lv)
                break
        if out is None:
            self.pts = [(rx, ry), (rx, ry)]
            self.level = -1.0
            self.reached = False
        else:
            self.pts, self.reached, self.level = out
        self.goal = (gx, gy)
        self.t_plan = now
        self.version += 1
        return self.pts

    def _astar(self, rx, ry, gx, gy, clear_m):
        ai = ROBOT_R + clear_m
        res = self.res
        x0 = min(rx, gx) - self.pad
        x1 = max(rx, gx) + self.pad
        y0 = min(ry, gy) - self.pad
        y1 = max(ry, gy) + self.pad
        while (x1 - x0) / res > self.max_cells \
                or (y1 - y0) / res > self.max_cells:
            res *= 1.25
        nx = int((x1 - x0) / res) + 1
        ny = int((y1 - y0) / res) + 1

        def w2g(x, y):
            return (int(round((x - x0) / res)), int(round((y - y0) / res)))

        si, sj = w2g(rx, ry)
        gi, gj = w2g(gx, gy)
        si = max(0, min(nx - 1, si))
        sj = max(0, min(ny - 1, sj))
        gi = max(0, min(nx - 1, gi))
        gj = max(0, min(ny - 1, gj))

        gap = [[99.0] * ny for _ in range(nx)]
        for i in range(nx):
            wx = x0 + i * res
            for j in range(ny):
                wy = y0 + j * res
                if abs(wx) > WALL_HARD or abs(wy) > WALL_HARD:
                    gap[i][j] = -9.0
                else:
                    gap[i][j] = self.clearance(wx, wy)
        # 起点必须在自由区。若不在, 交给 plan_now 降级重试 —— 在同一级里
        # 放宽起点的写法会让"起点自由、邻格不自由", 一步也走不出去。
        if gap[si][sj] < ai:
            return None

        reached = gap[gi][gj] >= ai
        if not reached:
            # 目标被挤进膨胀区(人贴障碍站着) -> 取附近最近的可达格,
            # 即"到达目标附近", 而不是永远无解。
            best, bkey = None, (1e9, 1e9)
            rad = max(1, int(1.4 / res))
            for i in range(max(0, gi - rad), min(nx, gi + rad + 1)):
                for j in range(max(0, gj - rad), min(ny, gj + rad + 1)):
                    if gap[i][j] < ai:
                        continue
                    dg = math.hypot((i - gi) * res, (j - gj) * res)
                    dr = math.hypot((i - si) * res, (j - sj) * res)
                    key = (round(dg, 1), dr)   # 先离目标近, 再离机器人近
                    if key < bkey:
                        bkey, best = key, (i, j)
            if best is None:
                return None
            gi, gj = best

        def h(i, j):
            dx, dy = abs(i - gi), abs(j - gj)
            return (dx + dy) + (1.41421356 - 2.0) * min(dx, dy)

        INF = 1e18
        gc = [[INF] * ny for _ in range(nx)]
        prv = [[None] * ny for _ in range(nx)]
        done = [[False] * ny for _ in range(nx)]
        gc[si][sj] = 0.0
        pq = [(h(si, sj), 0.0, si, sj)]
        while pq:
            _, ccost, i, j = heapq.heappop(pq)
            if done[i][j]:
                continue
            done[i][j] = True
            if i == gi and j == gj:
                break
            for di, dj, dc in self.NB:
                ni, nj = i + di, j + dj
                if not (0 <= ni < nx and 0 <= nj < ny) or done[ni][nj]:
                    continue
                if gap[ni][nj] < ai:
                    continue
                if di and dj:                  # 不允许斜穿角落
                    if gap[i + di][j] < ai or gap[i][j + dj] < ai:
                        continue
                over = gap[ni][nj] - ai
                k = 1.0
                if over < self.soft:           # 贴膨胀边界加价 -> 走中间
                    k = 1.0 + 2.2 * (1.0 - over / self.soft)
                ng = ccost + dc * res * k
                if ng < gc[ni][nj]:
                    gc[ni][nj] = ng
                    prv[ni][nj] = (i, j)
                    heapq.heappush(pq, (ng + h(ni, nj), ng, ni, nj))
        if gc[gi][gj] >= INF:
            return None

        cells = []
        cur = (gi, gj)
        while cur is not None:
            cells.append(cur)
            cur = prv[cur[0]][cur[1]]
        cells.reverse()
        pts = [(x0 + i * res, y0 + j * res) for (i, j) in cells]
        pts[0] = (rx, ry)

        # string pulling: 能直连就跳过中间点。必须逐段复验间隙,
        # 只验端点会切进障碍角。
        out = [pts[0]]
        k = 0
        while k < len(pts) - 1:
            nxt = k + 1
            for m in range(len(pts) - 1, k + 1, -1):
                if self.seg_clear(pts[k], pts[m]) >= ai - 1e-6:
                    nxt = m
                    break
            out.append(pts[nxt])
            k = nxt
        return out, reached, clear_m

    # ---------- 路径查询 ----------
    def _nearest_idx(self, rx, ry):
        pts = self.pts
        best, bd = 0, 1e9
        for i, (x, y) in enumerate(pts):
            d = math.hypot(x - rx, y - ry)
            if d < bd:
                best, bd = i, d
        return best

    def remaining(self, rx, ry):
        pts = self.pts
        if len(pts) < 2:
            return 0.0
        i0 = self._nearest_idx(rx, ry)
        L = math.hypot(pts[i0][0] - rx, pts[i0][1] - ry)
        for i in range(i0, len(pts) - 1):
            L += math.hypot(pts[i + 1][0] - pts[i][0],
                            pts[i + 1][1] - pts[i][1])
        return L

    def detour(self, rx, ry, gx, gy):
        """剩余路径长度 / 直线距离。>1.12 说明必须绕行。"""
        d = math.hypot(gx - rx, gy - ry)
        if d < 1e-6:
            return 1.0
        return self.remaining(rx, ry) / d

    def lookahead(self, rx, ry, ld=None):
        pts = self.pts
        if len(pts) < 2:
            return None
        ld = self.lookahead_m if ld is None else ld
        i0 = self._nearest_idx(rx, ry)
        acc = 0.0
        cx, cy = rx, ry
        for i in range(i0, len(pts)):
            x, y = pts[i]
            seg = math.hypot(x - cx, y - cy)
            if acc + seg >= ld:
                k = (ld - acc) / max(seg, 1e-6)
                return (cx + (x - cx) * k, cy + (y - cy) * k)
            acc += seg
            cx, cy = x, y
        return (pts[-1][0], pts[-1][1])
'''

anchor = '\n\ndef wrap(a):\n'
assert s.count(anchor) == 1, ('wrap anchor', s.count(anchor))
s = s.replace(anchor, PLANNER + '\n\ndef wrap(a):\n', 1)

io.open(DST, 'w', encoding='utf-8', newline='\n').write(s)
print('written', DST, len(s.splitlines()), 'lines')
