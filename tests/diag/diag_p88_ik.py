#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""diag_p88_ik.py —— P88 离线体检: "双手捧杯 + 杯柄朝人"能不能够到

P88 要在机器人**还没转身**的时候, 把杯子端到胸前并且让**杯柄指向人**。
人的方位是 62.9°, 所以杯子的 yaw 要从 0 转到 62.9° —— 两只手的掌心朝向
会跟着一起转 63°。这一转会不会顶到腕部限位, 离线先算清楚, 别等进仿真
再靠猜。

用法: python3 tests/diag/diag_p88_ik.py
"""
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, 'vla_ws', 'src', 'vla_follow'))

import numpy as np  # noqa: E402

from vla_follow import arm_kin as K          # noqa: E402
from vla_follow import drink_poses as P      # noqa: E402
from vla_follow import scene_layout as L     # noqa: E402

OK_M = 0.010
OK_R = 0.030


def probe(tag, T_palm, side, base, seed):
    q, _e, how = K.ik_world(side, T_palm, base, seed)
    err = K.pose_err(K.fk_world(side, q, base), T_palm)
    ep = float(np.linalg.norm(err[:3]))
    er = float(np.linalg.norm(err[3:]))
    marg = float(np.min(np.minimum(q - K.LIM_LO, K.LIM_HI - q)))
    flag = ''
    if ep > OK_M or er > OK_R:
        flag = '  <<< 不可达'
    elif math.degrees(marg) < 3.0:
        flag = '  << 贴限位'
    print('%-2s %-28s 位置%6.1fmm 姿态%5.1f° 余量%6.1f° %-10s%s'
          % (side, tag, ep * 1000.0, math.degrees(er),
             math.degrees(marg), how, flag))
    return ep <= OK_M and er <= OK_R, q


def sweep():
    """杯子 yaw x 前伸距离 的可达性扫描。

    动机: 机器人**还没转身**时, "杯柄朝人"要求杯 yaw~62.9°, 两手腕要跟着
    转 63°。先确认是"整体不可行"还是"只是距离没选对"。
    """
    bx, by, _ = L.SERVE_POSE
    base = (bx, by, 0.0)
    seed = {'L': K.Q_REST.copy(), 'R': K.Q_REST.copy()}
    print('\n%-6s %-5s %-4s  %-14s %-14s'
          % ('byaw', 'r', 'z', 'L(位置/余量)', 'R(位置/余量)'))
    for byaw_deg in (30, 45, 55, 63, 75, 90):
        for r in (0.40, 0.45, 0.50):
            byaw = math.radians(byaw_deg)
            T = P.cup_in_base(bx, by, byaw, r, L.CUP_CARRY_Z)
            cell = []
            for side in ('L', 'R'):
                Tp = P.cup_grasp_palm(T, side)
                q, _e, _how = K.ik_world(side, Tp, base, seed[side])
                err = K.pose_err(K.fk_world(side, q, base), Tp)
                ep = float(np.linalg.norm(err[:3]))
                er = float(np.linalg.norm(err[3:]))
                marg = math.degrees(float(np.min(np.minimum(
                    q - K.LIM_LO, K.LIM_HI - q))))
                bad = ep > OK_M or er > OK_R
                cell.append('%5.1fmm%s/%5.1f' % (ep * 1000, '!' if bad else ' ', marg))
                if not bad:
                    seed[side] = q
            print('%-6d %-5.2f %-4.2f  %-14s %-14s'
                  % (byaw_deg, r, L.CUP_CARRY_Z, cell[0], cell[1]))
    print('(! = 位置/姿态残差超限)')


def main():
    bx, by, _ = L.SERVE_POSE
    base = (bx, by, 0.0)
    seed = {'L': K.Q_REST.copy(), 'R': K.Q_REST.copy()}
    cands = []
    for byaw, tag in ((0.0, 'byaw=0'),
                      (L.PERSON_SPOT_YAW, 'byaw=人(62.9°)')):
        for fn, fname in ((P.carry_cup, 'carry'), (P.present_cup, 'present')):
            T = fn(bx, by, byaw)
            cands.append(('%s %s' % (fname, tag), T))
    ok_all = True
    for tag, T in cands:
        for side in ('L', 'R'):
            ok, q = probe(tag, P.cup_grasp_palm(T, side), side, base, seed[side])
            if ok:
                seed[side] = q
            ok_all = ok_all and ok

    # 关键对照: base 也转到 HANDOVER_YAW 之后, 相对构型与 byaw=0 完全一样
    print('-' * 92)
    base_h = (bx, by, L.HANDOVER_YAW)
    for fn, fname in ((P.carry_cup, 'carry'), (P.present_cup, 'present')):
        T = fn(bx, by, L.HANDOVER_YAW)      # cup_in_base 假设 base yaw == byaw
        for side in ('L', 'R'):
            probe('%s 转身后(朝人)' % fname, P.cup_grasp_palm(T, side),
                  side, base_h, seed[side])

    # 两掌是否真的在杯体两侧(夹角 ~180°) —— 用 FK 出来的掌心位置验证
    print('-' * 92)
    for tag, T in cands:
        pL = K.fk_world('L', seed['L'], base)[:3, 3]
        pR = K.fk_world('R', seed['R'], base)[:3, 3]
        ax = (pL + pR) / 2.0
        vL, vR = pL - ax, pR - ax
        nL, nR = np.linalg.norm(vL), np.linalg.norm(vR)
        ang = math.degrees(math.acos(max(-1.0, min(
            1.0, float(vL / nL @ (vR / nR))))))
        print('%-24s 两掌间距 %.3fm  夹持角 %.1f°  杯心 %s'
              % (tag, float(np.linalg.norm(pL - pR)), ang,
                 np.round(T[:3, 3], 3)))
    print('-' * 92)
    print('全部可达' if ok_all else '有不可达位形')
    if len(sys.argv) > 1 and sys.argv[1] == 'sweep':
        sweep()
    return 0 if ok_all else 2


if __name__ == '__main__':
    sys.exit(main())
