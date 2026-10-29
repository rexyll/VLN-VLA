"""看"放回"那个位形的 up_roll ↔ twist 交换曲线: 到底能不能两者都留余量,
还是这个位形天生就卡在人体包络的边上。

动机(P103 第 4 轮): 放回 L 实测 `up_roll=−1.91`(R4 上限 1.90)、`twist=−1.70`
(软限 1.70) —— 两项都贴在边界上, 一次过、下一次就 1.91 挂掉。判据本身没有
错, 问题是代价函数把解推到两个软约束的**折点**上。这份脚本把整族解的两个
分量画出来, 才知道该"调代价"还是"认命"。

用法: python3 tests/diag/_p103_tradeoff.py [tests/out/test_p103.json] [tag] [side]
"""
import json
import sys

import numpy as np

sys.path.insert(0, 'vla_ws/src/vla_follow')
from vla_follow import arm_kin as K  # noqa: E402

PANS = (-1.2, -0.8, -0.4, 0.0, 0.4, 0.8, 1.2)
LIFTS = (-1.6, -1.2, -0.8, -0.4, 0.0)
UROLLS = (-1.9, -1.6, -1.3, -1.0, -0.7, -0.4, -0.1,
          0.2, 0.5, 0.8, 1.1, 1.4, 1.7)
ELBOWS = (-0.4, -0.9, -1.4, -1.9)


def main():
    p = sys.argv[1] if len(sys.argv) > 1 else 'tests/out/test_p103.json'
    tag = sys.argv[2] if len(sys.argv) > 2 else '放回'
    side = sys.argv[3] if len(sys.argv) > 3 else 'L'
    a = json.load(open(p, encoding='utf-8'))['result']['arm']
    st = (a.get(tag) or {}).get(side)
    if not st:
        print('没有 %s/%s 的数据' % (tag, side))
        return
    T = K.fk(side, np.asarray(st['q'], float))
    rows, seen = [], set()
    for pn in PANS:
        for lf in LIFTS:
            for v in UROLLS:
                for e in ELBOWS:
                    s = K.Q_REST.copy()
                    s[0], s[1], s[2], s[3] = pn, lf, v, e
                    q, _err, _h = K.ik_best(side, T, s)
                    q = np.clip(q, K.LIM_LO, K.LIM_HI)
                    if float(np.linalg.norm(K.pose_err(K.fk(side, q), T))) > 2e-3:
                        continue
                    key = tuple(np.round(q / 0.08).astype(int))
                    if key in seen:
                        continue
                    seen.add(key)
                    rows.append(dict(q=q, up=float(q[2]),
                                     tw=float(q[4] + q[6]),
                                     el=float(q[3]), wp=float(q[5]),
                                     bad=K.violation_list(side, q),
                                     cost=K.human_penalty(side, q),
                                     out=K.elbow_metrics(side, q)['out_line_mm']))
    rows.sort(key=lambda r: r['up'])
    print('%s %s  分支 %d 条   实测 up_roll=%+.2f twist=%+.2f'
          % (tag, side, len(rows), st['up_roll'], st['twist']))
    print('%7s %7s %7s %7s %7s %6s  %s'
          % ('up_roll', 'twist', 'elbow', 'wrist_p', '代价', '外扩', '违反'))
    for r in rows:
        # 只打印"两项都还算靠内"的那一段, 其余略过
        if abs(r['up']) > 1.95 or abs(r['tw']) > 2.1:
            continue
        print('%+7.2f %+7.2f %+7.2f %+7.2f %6.3f %+6.0f  %s'
              % (r['up'], r['tw'], r['el'], r['wp'], r['cost'], r['out'],
                 ('★' + ' '.join(r['bad'])) if r['bad'] else 'OK'))
    safe = [r for r in rows if abs(r['up']) <= 1.85 and abs(r['tw']) <= 1.95
            and not r['bad']]
    print('两项都留 0.05 余量且合规的分支: %d 条' % len(safe))
    if safe:
        b = min(safe, key=lambda r: r['cost'])
        print('  其中代价最小: up_roll %+.2f twist %+.2f 肘 %+.2f 外扩 %+.0f'
              ' 代价 %.3f' % (b['up'], b['tw'], b['el'], b['out'], b['cost']))


if __name__ == '__main__':
    main()
