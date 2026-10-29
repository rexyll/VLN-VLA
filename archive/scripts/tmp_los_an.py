#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_los_an.py —— 丢目标归因: 遮挡(几何) vs 漏检(感知)

统计口径:
  miss         = det == False
  occluded     = 机器人→人 的连线穿过障碍(OBB)
  los_clear    = 没穿过障碍
  一次"丢失事件" = 连续 miss >= 3 拍(>=0.3s)
"""
import json
import sys


def load(fn):
    rows = []
    for ln in open(fn, encoding='utf-8'):
        try:
            rows.append(json.loads(ln))
        except Exception:
            pass
    return rows


def episodes(rows, k=3):
    out, cur = [], []
    for r in rows:
        if not r['det']:
            cur.append(r)
        else:
            if len(cur) >= k:
                out.append(cur)
            cur = []
    if len(cur) >= k:
        out.append(cur)
    return out


def main():
    for fn in sys.argv[1:]:
        rows = load(fn)
        n = len(rows)
        if not n:
            print(fn, '空')
            continue
        miss = [r for r in rows if not r['det']]
        moc = [r for r in miss if r['occl']]
        mcl = [r for r in miss if not r['occl']]
        print('=' * 62)
        print('%s  样本 %d  时长 %.0fs' % (fn, n, rows[-1]['t']))
        print('  检出率 %.1f%%   miss %d (%.1f%%)' %
              (100.0 * (n - len(miss)) / n, len(miss), 100.0 * len(miss) / n))
        print('  miss 且视线被障碍挡住 : %d (%.1f%% of miss)' %
              (len(moc), 100.0 * len(moc) / max(len(miss), 1)))
        print('  miss 但视线通畅       : %d (%.1f%% of miss)' %
              (len(mcl), 100.0 * len(mcl) / max(len(miss), 1)))
        # 视线通畅却漏检: 距离分布
        if mcl:
            print('  --- 视线通畅仍漏检的距离分布 ---')
            bks = [(0, 0.8), (0.8, 1.2), (1.2, 2.0), (2.0, 3.0),
                   (3.0, 5.0), (5.0, 99)]
            for lo, hi in bks:
                s = [r for r in mcl if lo <= r['d'] < hi]
                if s:
                    print('      %.1f~%.1fm: %3d 拍  人离障碍中位 %.2fm  '
                          '在front视野内 %d%%'
                          % (lo, hi, len(s),
                             sorted(r['pd'] for r in s)[len(s) // 2],
                             100 * sum(1 for r in s if r['in_fov']) // len(s)))
        ev = episodes(rows)
        print('  丢失事件(>=0.3s): %d 次' % len(ev))
        dur = [(e[-1]['t'] - e[0]['t'] + 0.1, e[0], e[-1]) for e in ev]
        dur.sort(reverse=True, key=lambda x: x[0])
        for d, a, b in dur[:12]:
            print('     %5.1fs  d=%4.2f  brg=%+6.1f°  occl=%-10s '
                  '人离障碍%.2fm  trk=%s' %
                  (d, a['d'], a['brg'], ','.join(a['occl']) or '-',
                   a['pd'], a['trk']))
        if dur:
            print('  丢失总时长 %.1fs / %.0fs = %.1f%%' %
                  (sum(x[0] for x in dur), rows[-1]['t'],
                   100 * sum(x[0] for x in dur) / rows[-1]['t']))
        # 人离障碍物的距离 vs 检出率
        print('  --- 人-障碍距离 vs 检出率 ---')
        for lo, hi in [(0, 0.2), (0.2, 0.4), (0.4, 0.7), (0.7, 1.2),
                       (1.2, 99)]:
            s = [r for r in rows if lo <= r['pd'] < hi]
            if s:
                det = sum(1 for r in s if r['det'])
                print('     pd %.1f~%.1fm: %4d 拍  检出率 %5.1f%%' %
                      (lo, hi, len(s), 100.0 * det / len(s)))


if __name__ == '__main__':
    main()
