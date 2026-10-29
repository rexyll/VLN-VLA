"""复现"撤手"这一步的逆解: 从放回位(合规)把掌心平移 (向后 120mm, 向上 90mm),
看 `ik_human` / `ik_best` 各给出什么, 以及为什么会出现 R5/R6/R7 三越的位形。

用法: python3 tests/diag/_p103_handoff_solve.py [tests/out/test_p103.json]
"""
import json
import sys

import numpy as np

sys.path.insert(0, 'vla_ws/src/vla_follow')
from vla_follow import arm_kin as K  # noqa: E402


def show(tag, side, q):
    m = K.elbow_metrics(side, q)
    print('  %-22s q=%s' % (tag, np.round(q, 2)))
    print('  %-22s up_roll=%+.2f 肘=%+.2f twist=%+.2f wrist_p=%+.2f '
          '外扩=%+.0f 违=%s'
          % ('', q[2], q[3], q[4] + q[6], q[5], m['out_line_mm'],
             K.violation_list(side, q) or '无'))


def main():
    p = sys.argv[1] if len(sys.argv) > 1 else 'tests/out/test_p103.json'
    arm = json.load(open(p, encoding='utf-8'))['result']['arm']
    for side in ('L', 'R'):
        st = (arm.get('放回') or {}).get(side)
        if not st:
            continue
        q0 = np.asarray(st['q'], float)
        T = K.fk(side, q0)
        print('=' * 88)
        print('放回 %s (起点)' % side)
        show('起点', side, q0)
        for dx, dz in ((-0.12, 0.09), (-0.12, 0.0), (0.0, 0.09)):
            T2 = T.copy()
            T2[:3, 3] = T2[:3, 3] + np.array([dx, 0.0, dz])
            print('-- 平移 dx=%+.2f dz=%+.2f' % (dx, dz))
            qa, ea, ha = K.ik_best(side, T2, q0.copy())
            qa = np.clip(qa, K.LIM_LO, K.LIM_HI)
            show('ik_best(局部)', side, qa)
            print('    残差 %.1fmm 来源 %s 代价 %.3f'
                  % (ea * 1e3, ha, K.human_penalty(side, qa)))
            qb, eb, hb = K.ik_human(side, T2, q0.copy())
            show('ik_human(策略)', side, qb)
            print('    残差 %.1fmm 来源 %s 代价 %.3f'
                  % (eb * 1e3, hb, K.human_penalty(side, qb)))
            # 候选都长什么样(只看合规且 4mm 内的)
            ok = []
            for v in K.HUMAN_SEEDS:
                s = K.Q_REST.copy()
                s[0], s[1], s[2], s[3] = v
                q, err, _h = K.ik_best(side, T2, s)
                if err > 4e-3:
                    continue
                q = np.clip(q, K.LIM_LO, K.LIM_HI)
                if K.violation_list(side, q):
                    continue
                ok.append((K.human_penalty(side, q), q))
            ok.sort(key=lambda c: c[0])
            print('    4mm 内的合规候选 %d 条%s'
                  % (len(ok), ('，最优代价 %.3f' % ok[0][0]) if ok else ''))


if __name__ == '__main__':
    main()
