# -*- coding: utf-8 -*-
"""历史轨迹分段体检: 把累积的 traj_log.csv 按"运行段"切开, 逐段算出
   ① 机器人前进方向(速度·车头)  ② 转向斜率(dyaw/dt ~ bearing)
   ③ 人移动方向(移动·自身朝向)

用法: python tmp_an_traj_segments.py <csv> [每段最少行数]
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


def load(path):
    rows = []
    with open(path, newline='', encoding='utf-8', errors='replace') as f:
        for r in csv.DictReader(f):
            try:
                rows.append({k: float(v) for k, v in r.items()
                             if k and v not in (None, '')})
            except (ValueError, TypeError):
                continue
    return rows


def seg_bounds(rows, min_rows):
    """按 t 跳变(>5s)与行数切段。"""
    segs = []
    start = 0
    for i in range(1, len(rows)):
        if rows[i]['t'] - rows[i - 1]['t'] > 5.0:
            if i - start >= min_rows:
                segs.append((start, i))
            start = i
    if len(rows) - start >= min_rows:
        segs.append((start, len(rows)))
    return segs


def analyze_seg(rows, a, b):
    fwd, per = [], []
    for i in range(a + 1, b):
        dx = rows[i]['rx'] - rows[i - 1]['rx']
        dy = rows[i]['ry'] - rows[i - 1]['ry']
        n = math.hypot(dx, dy)
        if n > 1e-4:
            yaw = rows[i]['ryaw']
            fwd.append((dx * math.cos(yaw) + dy * math.sin(yaw)) / n)
        dx = rows[i]['px'] - rows[i - 1]['px']
        dy = rows[i]['py'] - rows[i - 1]['py']
        n = math.hypot(dx, dy)
        if n > 1e-4:
            pyaw = rows[i]['pyaw']
            per.append((dx * math.cos(pyaw) + dy * math.sin(pyaw)) / n)

    xs, ys = [], []
    for i in range(a + 1, b):
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
    slope = float('nan')
    if len(xs) >= 30:
        mx = sum(xs) / len(xs)
        my = sum(ys) / len(ys)
        num = sum((p - mx) * (q - my) for p, q in zip(xs, ys))
        den = sum((p - mx) ** 2 for p in xs)
        slope = num / den if den > 0 else 0.0

    def med(arr):
        if not arr:
            return float('nan')
        s = sorted(arr)
        return s[len(s) // 2]

    d = sorted(math.hypot(r['px'] - r['rx'], r['py'] - r['ry'])
               for r in rows[a:b])
    return {
        't0': rows[a]['t'], 't1': rows[b - 1]['t'], 'n': b - a,
        'fwd': med(fwd), 'per': med(per), 'slope': slope,
        'dmed': d[len(d) // 2] if d else float('nan'),
    }


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else 'traj_log.csv'
    min_rows = int(sys.argv[2]) if len(sys.argv) > 2 else 300
    rows = load(path)
    print('=== %s  总行数 %d ===' % (path, len(rows)))
    print('%-10s %-10s %7s %8s %8s %8s %8s'
          % ('t0', 't1', '行数', '前进·车头', '人·朝向', '转向斜率', '距离中位'))
    for a, b in seg_bounds(rows, min_rows):
        r = analyze_seg(rows, a, b)
        print('%-10.1f %-10.1f %7d %8.3f %8.3f %8.3f %8.2f'
              % (r['t0'], r['t1'], r['n'], r['fwd'], r['per'],
                 r['slope'], r['dmed']))


if __name__ == '__main__':
    main()
