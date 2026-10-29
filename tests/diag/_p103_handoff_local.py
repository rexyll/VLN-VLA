"""扫"撤手"两步几何: 局部跟踪下每一步都要合规(不只是终点)。

hands_off 用 `elbow=None`, 每步用上一步的解作种子。所以中间路点也会被
执行 —— 它越限, 画面上就是"抽手时肩突然内旋一下"。这里把顺序和分量都扫一遍。
用法: python3 tests/diag/_p103_handoff_local.py [json]
"""
import json
import sys

import numpy as np

sys.path.insert(0, 'vla_ws/src/vla_follow')
from vla_follow import arm_kin as K  # noqa: E402


def steps(side, q0, plan):
    """plan = [(dy, dz), ...]; 返回每一步的 (q, err, bad)。"""
    q = np.asarray(q0, float)
    T = K.fk(side, q)
    sgn = 1.0 if side == 'L' else -1.0
    out = []
    for dy, dz in plan:
        T2 = T.copy()
        T2[:3, 3] = T[:3, 3] + np.array([0.0, sgn * dy, dz])
        q2, err, _how = K.ik_best(side, T2, q.copy())
        q2 = np.clip(q2, K.LIM_LO, K.LIM_HI)
        out.append((q2, err, K.violation_list(side, q2), T2))
        q, T = q2, T2
    return out


def main():
    p = sys.argv[1] if len(sys.argv) > 1 else 'tests/out/test_p103.json'
    arm = json.load(open(p, encoding='utf-8'))['result']['arm']
    st = arm.get('放回') or {}
    cand = (0.08, 0.10, 0.12)
    plans = {}
    for o in cand:
        for u in cand:
            plans['外张%.2f→抬%.2f' % (o, u)] = [(o, 0.0), (0.0, u)]
            plans['抬%.2f→外张%.2f' % (u, o)] = [(0.0, u), (o, 0.0)]
            plans['半半%.2f/%.2f ×2' % (o, u)] = [(o / 2, u / 2), (o / 2, u / 2)]
    for name, plan in plans.items():
        line = '%-24s' % name
        okall = True
        for side in ('L', 'R'):
            if side not in st:
                continue
            r = steps(side, st[side]['q'], plan)
            worst = None
            for i, (q, err, bad, T) in enumerate(r):
                if bad:
                    okall = False
                    worst = (i, bad)
            q, err, bad, T = r[-1]
            line += ' | %s 终:腕=%+.2f 肘=%+.2f 扭=%+.2f 掌高=%.3f%s' % (
                side, q[2], q[3], q[4] + q[6], T[2, 3],
                ('' if not worst else '  ←第%d步 %s' % (worst[0], ','.join(worst[1]))))
        print(('✔ ' if okall else '✘ ') + line)


if __name__ == '__main__':
    main()
