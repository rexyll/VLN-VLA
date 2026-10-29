#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_p77_an.py —— 分析 /follower/diag 录制(jsonl), 输出"徘徊/打摆"关键指标

用法: python3 tmp_p77_an.py diag_scn_pillar.jsonl [diag2.jsonl ...]

指标:
  dur        录制时长
  static_max 最长"完全静止"连续时长(v≈0 且 w≈0)
  wflip      有幅度的 w(|w|>=0.15) 符号翻转次数 与 次/秒
  wswitch    owner 切换次数(需要新版 diag 有 wown 字段)
  mode_sw    mode(ALIGN/MOVE/AVOID) 切换次数
  tail       最后 40s 的 dist 中位 / |brg| 中位
  bclip      近距离方位冻结占比(均值, 新版才有)
"""
import json
import sys


def load(p):
    rows = []
    with open(p, encoding='utf-8') as f:
        for ln in f:
            ln = ln.strip()
            if not ln:
                continue
            try:
                rows.append(json.loads(ln))
            except Exception:
                pass
    return rows


def longest_static(rows):
    best = cur = 0.0
    prev_t = None
    for r in rows:
        t = r.get('t', 0.0)
        dt = 0.0 if prev_t is None else max(0.0, t - prev_t)
        prev_t = t
        if abs(r.get('v', 0.0)) < 0.01 and abs(r.get('w', 0.0)) < 0.05:
            cur += dt
            best = max(best, cur)
        else:
            cur = 0.0
    return best


def flips(rows, thr=0.15):
    n = 0
    last = 0.0
    for r in rows:
        w = r.get('w', 0.0)
        if abs(w) >= thr:
            s = 1 if w > 0 else -1
            if last != 0.0 and s != last:
                n += 1
            last = s
    return n


def switches(rows, key):
    n = 0
    last = None
    for r in rows:
        v = r.get(key)
        if v is None:
            continue
        if last is not None and v != last:
            n += 1
        last = v
    return n


def med(xs):
    if not xs:
        return 0.0
    xs = sorted(xs)
    return xs[len(xs) // 2]


def wander(rows, tail_s=40.0):
    """尾段(最后 tail_s 秒)位移指标 —— 判定"原地打摆"
    返回 (净位移 m, 路径长 m, 偏航 churn 度, 净转角 度)
    """
    if not rows:
        return 0.0, 0.0, 0.0, 0.0
    t1 = rows[-1].get('t', 0.0)
    seg = [r for r in rows if r.get('t', 0.0) > t1 - tail_s]
    if len(seg) < 2:
        return 0.0, 0.0, 0.0, 0.0
    x0, y0 = seg[0].get('x', 0.0), seg[0].get('y', 0.0)
    x1, y1 = seg[-1].get('x', 0.0), seg[-1].get('y', 0.0)
    net = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
    path = 0.0
    churn = 0.0
    prev = seg[0]
    prev_yaw = prev.get('yaw', 0.0)
    for r in seg[1:]:
        path += (((r.get('x', 0.0) - prev.get('x', 0.0)) ** 2
                  + (r.get('y', 0.0) - prev.get('y', 0.0)) ** 2) ** 0.5)
        dy = r.get('yaw', 0.0) - prev_yaw
        while dy > 3.14159265:
            dy -= 6.2831853
        while dy < -3.14159265:
            dy += 6.2831853
        churn += abs(dy)
        prev_yaw = r.get('yaw', 0.0)
        prev = r
    net_yaw = abs(prev_yaw - seg[0].get('yaw', 0.0))
    return net, path, churn * 57.29578, net_yaw * 57.29578


def report(p):
    rows = load(p)
    if not rows:
        print('%-28s 空文件' % p)
        return
    t0, t1 = rows[0].get('t', 0.0), rows[-1].get('t', 0.0)
    dur = max(1e-6, t1 - t0)
    nf = flips(rows)
    tail = [r for r in rows if r.get('t', 0) > t1 - 40.0]
    td = [r.get('dist', 0.0) for r in tail if r.get('dist', 0) > 0]
    tb = [abs(r.get('brg', 0.0)) for r in tail if r.get('dist', 0) > 0]
    bc = [r.get('bclip', 0.0) for r in rows if 'bclip' in r]
    print('%s' % p)
    print('  dur=%5.1fs n=%d | 最长静止=%5.1fs | w 换向=%3d (%.2f 次/s)'
          % (dur, len(rows), longest_static(rows), nf, nf / dur))
    print('  mode 切换=%d | owner 切换=%s | 尾段 dist 中位=%.2f |brg| 中位=%.0f°'
          % (switches(rows, 'mode'), switches(rows, 'wown') or '-',
             med(td), med(tb)))
    wn, wp, wc, wny = wander(rows)
    print('  尾40s: 净位移=%.2fm 路径长=%.2fm | 偏航 churn=%.0f° 净转=%.0f°'
          % (wn, wp, wc, wny))
    if bc:
        print('  近距离方位冻结占比(均)=%.2f  最大=%.2f' % (sum(bc) / len(bc), max(bc)))
    if 'wown' in rows[-1]:
        from collections import Counter
        c = Counter(r.get('wown', '') for r in rows)
        print('  owner 时间占比: ' + ', '.join('%s=%.0f%%'
              % (k, 100.0 * v / len(rows)) for k, v in c.most_common()))


if __name__ == '__main__':
    for p in (sys.argv[1:] or ['diag_scn_pillar.jsonl']):
        report(p)
