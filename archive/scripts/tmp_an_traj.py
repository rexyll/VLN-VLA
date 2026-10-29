# -*- coding: utf-8 -*-
"""对任意 traj_log.csv 做方向约定体检:
   用法: python tmp_an_traj.py <csv路径> [最大行数]
   1) 机器人速度方向 · 车头方向  (>0 = 沿车头前进)
   2) 人移动方向 · 自身朝向      (>0 = 人沿自身朝向走)
   3) 转向符号: 回归 dyaw/dt ~ bearing, 斜率>0 = "目标在左(bearing>0)时左转"=正确
      (排除 |brg|<20° 与 |brg|>160° 的样本)
"""
import csv
import math
import sys


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def load(path, cap):
    rows = []
    with open(path, newline='', encoding='utf-8', errors='replace') as f:
        rd = csv.DictReader(f)
        for r in rd:
            try:
                rows.append({k: float(v) for k, v in r.items()
                             if k and v not in (None, '')})
            except (ValueError, TypeError):
                continue
            if len(rows) >= cap:
                break
    return rows


def analyze(path, cap=200000):
    rows = load(path, cap)
    print('=== %s  (采样 %d 行) ===' % (path, len(rows)))
    if len(rows) < 10:
        print('数据不足')
        return

    # 机器人
    fd = []
    for i in range(1, len(rows)):
        dx = rows[i]['rx'] - rows[i - 1]['rx']
        dy = rows[i]['ry'] - rows[i - 1]['ry']
        n = math.hypot(dx, dy)
        if n < 1e-4:
            continue
        yaw = rows[i]['ryaw']
        fd.append((dx * math.cos(yaw) + dy * math.sin(yaw)) / n)
    # 人
    pd = []
    for i in range(1, len(rows)):
        dx = rows[i]['px'] - rows[i - 1]['px']
        dy = rows[i]['py'] - rows[i - 1]['py']
        n = math.hypot(dx, dy)
        if n < 1e-4:
            continue
        pyaw = rows[i]['pyaw']
        pd.append((dx * math.cos(pyaw) + dy * math.sin(pyaw)) / n)

    def rep(tag, arr):
        if not arr:
            print('  %s: 无有效样本' % tag)
            return
        pos = sum(1 for x in arr if x > 0)
        print('  %s 均值%+.3f 中位%+.3f 同向%5.1f%% (n=%d)'
              % (tag, sum(arr) / len(arr), sorted(arr)[len(arr) // 2],
                 100.0 * pos / len(arr), len(arr)))

    rep('[机器人] 速度·车头 ', fd)
    rep('[人]     移动·朝向 ', pd)

    # 转向回归: dyaw/dt 对 bearing 的最小二乘斜率
    xs, ys = [], []
    for i in range(1, len(rows)):
        dt = rows[i]['t'] - rows[i - 1]['t']
        if dt <= 0 or dt > 1.0:
            continue
        dyaw = wrap(rows[i]['ryaw'] - rows[i - 1]['ryaw'])
        if abs(dyaw) < 2e-3:
            continue
        brg = wrap(math.atan2(rows[i]['py'] - rows[i]['ry'],
                              rows[i]['px'] - rows[i]['rx']) - rows[i]['ryaw'])
        if abs(brg) < math.radians(20) or abs(brg) > math.radians(160):
            continue
        xs.append(brg)
        ys.append(dyaw / dt)
    if len(xs) >= 20:
        mx = sum(xs) / len(xs)
        my = sum(ys) / len(ys)
        num = sum((a - mx) * (b - my) for a, b in zip(xs, ys))
        den = sum((a - mx) ** 2 for a in xs)
        slope = num / den if den > 0 else 0.0
        same = sum(1 for a, b in zip(xs, ys) if (a > 0) == (b > 0))
        print('  [转向] 回归斜率 dyaw/dt~bearing = %+.3f  (n=%d, 同号%.1f%%)'
              % (slope, len(xs), 100.0 * same / len(xs)))
        print('         %s' % ('目标在左→左转(正确)' if slope > 0
                              else '目标在左→右转(转向反了!)'))
    else:
        print('  [转向] 样本不足')

    d = sorted(math.hypot(r['px'] - r['rx'], r['py'] - r['ry']) for r in rows)
    print('  [人机距离] 中位 %.2fm  最小 %.2fm  最大 %.2fm' %
          (d[len(d) // 2], d[0], d[-1]))
    print()


if __name__ == '__main__':
    p = sys.argv[1] if len(sys.argv) > 1 else 'deploy/traj_log.csv'
    cap = int(sys.argv[2]) if len(sys.argv) > 2 else 200000
    analyze(p, cap)
