"""量"撤手"的三维可行域: 后撤(back) × 侧向离开(out) × 抬升(up)。

撤手的目的只有一个: 让手掌离开杯壁, 之后倒车 1.15m 不蹭杯子。方向和分量
都可以商量 —— 原来固定成"后撤 120 + 抬 90", 实测这个位形在人体规则下
**一条合规分支都没有**(见 _p103_handoff_sweep.py), 所以要换几何。

输出每格: 合规候选条数 / 最优代价 / 掌心到杯轴的估计水平距离 / 掌心高度。
用法: python3 tests/diag/_p103_handoff_3d.py [tests/out/test_p103.json] [--relax]
"""
import json
import sys

import numpy as np

sys.path.insert(0, 'vla_ws/src/vla_follow')
from vla_follow import arm_kin as K  # noqa: E402

BACK = (0.00, 0.04, 0.08)
OUT = (0.04, 0.08, 0.12)
UP = (0.04, 0.08, 0.12)


def solve(side, T, tol=4e-3):
    ok = []
    for v in K.HUMAN_SEEDS:
        s = K.Q_REST.copy()
        s[0], s[1], s[2], s[3] = v
        q, err, _h = K.ik_best(side, T, s)
        if err > tol:
            continue
        q = np.clip(q, K.LIM_LO, K.LIM_HI)
        if K.violation_list(side, q):
            continue
        ok.append((K.human_penalty(side, q), q))
    ok.sort(key=lambda c: c[0])
    return (len(ok), ok[0][0] if ok else float('nan'),
            ok[0][1] if ok else None)


def relax(T, tilt=0.35):
    from scipy.spatial.transform import Rotation as R
    R0 = np.asarray(T[:3, :3], float)
    yaw = np.arctan2(R0[1, 0], R0[0, 0])
    Rn = (R.from_euler('z', yaw) * R.from_euler('x', np.pi)).as_matrix()
    T2 = np.asarray(T, float).copy()
    u, _s, vt = np.linalg.svd((1.0 - tilt) * R0 + tilt * Rn)
    T2[:3, :3] = u @ vt
    return T2


def main():
    p = sys.argv[1] if len(sys.argv) > 1 else 'tests/out/test_p103.json'
    do_relax = '--relax' in sys.argv
    arm = json.load(open(p, encoding='utf-8'))['result']['arm']
    st = arm.get('放回') or {}
    if not st:
        print('没有 放回 检查点'); return
    # 杯轴的粗略估计: 两手捧杯时掌心分列杯轴两侧 -> 取两掌心中点
    TL = K.fk('L', np.asarray(st['L']['q'], float))
    TR = K.fk('R', np.asarray(st['R']['q'], float))
    axis = 0.5 * (TL[:3, 3] + TR[:3, 3])
    axis[2] = 0.0
    print('杯轴估计 (x,y)=(%.3f, %.3f)  两掌 z=%.3f/%.3f'
          % (axis[0], axis[1], TL[2, 3], TR[2, 3]))
    for side in ('L', 'R'):
        q0 = np.asarray(st[side]['q'], float)
        T0 = K.fk(side, q0)
        sgn = 1.0 if side == 'L' else -1.0
        print('=' * 92)
        print('放回 %s 起点 up=%+.2f el=%+.2f wp=%+.2f 掌=(%.3f,%.3f,%.3f)'
              % (side, q0[2], q0[3], q0[5], T0[0, 3], T0[1, 3], T0[2, 3]))
        print('%-22s %s' % ('back/out/up', '合规条数 代价 | 离杯轴mm 掌高mm | 关节'))
        for db in BACK:
            for do in OUT:
                for du in UP:
                    T2 = T0.copy()
                    T2[:3, 3] = T2[:3, 3] + np.array([-db, sgn * do, du])
                    if do_relax:
                        T2 = relax(T2)
                    n, cost, q = solve(side, T2)
                    tgt = T2[:3, 3]
                    ax = float(np.hypot(tgt[0] - axis[0], tgt[1] - axis[1]))
                    if not n:
                        print('%-22s 0' % ('%.2f/%.2f/%.2f' % (db, do, du)))
                        continue
                    print('%-22s %2d  %.2f | %5.0f %6.0f | up=%+.2f el=%+.2f '
                          'tw=%+.2f wp=%+.2f'
                          % ('%.2f/%.2f/%.2f' % (db, do, du), n, cost,
                             ax * 1e3, tgt[2] * 1e3, q[2], q[3],
                             q[4] + q[6], q[5]))
        print('起点离杯轴 %.0fmm 掌高 %.0fmm'
              % (np.hypot(T0[0, 3] - axis[0], T0[1, 3] - axis[1]) * 1e3,
                 T0[2, 3] * 1e3))


if __name__ == '__main__':
    main()
