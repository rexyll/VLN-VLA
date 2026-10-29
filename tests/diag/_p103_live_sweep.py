"""用**实测**位形反查分支: 拿验收 JSON 里那几个位形的掌心位姿, 稠密撒种子
重解一遍, 看"合规"的解到底存不存在、外扩多少。

比 `_p103_branch_sweep.py` 更贴近现场 —— 目标是实车跑出来的, 不是标称位。
用法: python3 tests/diag/_p103_live_sweep.py [tests/out/test_p103.json]
"""
import json
import sys

import numpy as np

sys.path.insert(0, 'vla_ws/src/vla_follow')
from vla_follow import arm_kin as K  # noqa: E402

PANS = (-0.9, -0.3, 0.0, 0.3, 0.9)
LIFTS = (-1.4, -0.9, -0.4, 0.1)
UROLLS = (-1.8, -1.4, -1.0, -0.6, -0.2, 0.2, 0.6, 1.0, 1.4, 1.8)
ELBOWS = (-0.4, -1.0, -1.6, -2.1)


def sweep(side, T, tag, q_live):
    seen = {}
    for p in PANS:
        for lf in LIFTS:
            for v in UROLLS:
                for e in ELBOWS:
                    s = K.Q_REST.copy()
                    s[0], s[1], s[2], s[3] = p, lf, v, e
                    q, err, _h = K.ik_best(side, T, s)
                    q = np.clip(q, K.LIM_LO, K.LIM_HI)
                    if float(np.linalg.norm(K.pose_err(K.fk(side, q), T))) > 2e-3:
                        continue
                    key = tuple(np.round(q / 0.08).astype(int))
                    if key in seen:
                        continue
                    m = dict(K.elbow_metrics(side, q))
                    m.update(q=q, cost=K.human_penalty(side, q),
                             bad=K.violation_list(side, q),
                             twist=float(q[4] + q[6]), elbow=float(q[3]),
                             wrist_p=float(q[5]),
                             dq=float(np.max(np.abs(q - np.asarray(q_live)))))
                    seen[key] = m
    rows = sorted(seen.values(), key=lambda r: r['cost'])
    ok = [r for r in rows if not r['bad']]
    print('=' * 96)
    print('%s  独立分支 %d / 合规 %d   (实测分支代价 %.3f, 违反 %s)'
          % (tag, len(rows), len(ok),
             K.human_penalty(side, np.asarray(q_live)),
             K.violation_list(side, np.asarray(q_live))))
    print('%6s %7s %7s %7s %7s %7s %6s  %s'
          % ('代价', '外扩线', 'droop', 'twist', 'up_roll', 'elbow', 'Δq', '构型'))
    for r in rows[:8]:
        print('%6.3f %+7.0f %+7.0f %+7.2f %+7.2f %+7.2f %6.2f  %s'
              % (r['cost'], r['out_line_mm'], r['droop_mm'], r['twist'],
                 r['up_roll'], r['elbow'], r['dq'],
                 ('★' + ' '.join(r['bad'])) if r['bad'] else 'OK'))
    if ok:
        b = max(ok, key=lambda r: r['out_line_mm'])
        print('合规解里外扩最大 %+.0fmm (droop %+.0f, twist %+.2f, up_roll %+.2f,'
              ' Δq %.2f)' % (b['out_line_mm'], b['droop_mm'], b['twist'],
                             b['up_roll'], b['dq']))
        c = min(ok, key=lambda r: r['dq'])
        print('合规解里离实测位形最近 Δq %.2f, 外扩 %+.0f, up_roll %+.2f'
              % (c['dq'], c['out_line_mm'], c['up_roll']))
    return rows


def main():
    p = sys.argv[1] if len(sys.argv) > 1 else 'tests/out/test_p103.json'
    a = json.load(open(p, encoding='utf-8'))['result']['arm']
    for tag, side in (('grasp_cup', 'L'), ('pour', 'R'), ('放回', 'L'),
                      ('捧杯', 'R'), ('捧杯', 'L')):
        st = (a.get(tag) or {}).get(side)
        if not st:
            continue
        q = np.asarray(st['q'], float)
        sweep(side, K.fk(side, q), '%s %s' % (tag, side), q)


if __name__ == '__main__':
    main()
