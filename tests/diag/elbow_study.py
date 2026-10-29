#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""elbow_study.py —— 离线研究"捧杯时肘部朝哪"(P101), 不用起仿真。

问题: 双手捧杯/递杯时, 实测肘部是**朝下**的(像端着托盘), 而人抱杯子时肘是
**朝外**的(前臂横在胸前)。7 轴臂对 6 维位姿冗余 1 维, 这一维就是"肘往哪
转" —— 由 IK 的种子/零空间偏置决定, 不是目标位姿能定死的。

本脚本: 固定杯子在胸前 carry 位, 取两侧掌心目标, 用不同种子解 IK, 打印
    ① 残差(必须小, 否则这个种子是废解)
    ② 肘部的横向分量(相对肩, 沿体侧 ±y) —— 越大越"朝外"
    ③ up_roll 角(理论上就是"肘的朝向盘"转动角)
用来挑一个能稳定产出"肘朝外"的种子。
"""
import math
import sys

import numpy as np

sys.path.insert(0, 'vla_ws/src/vla_follow')
from vla_follow import arm_kin as K          # noqa: E402
from vla_follow import drink_poses as P     # noqa: E402
from vla_follow import scene_layout as L    # noqa: E402


def elbow_local(side, q):
    """肘点(本体系): 链上 up_roll 之前那个平移 -L1 的点。"""
    T = K.frame(t=K.SHOULDER[side])
    for i, (_n, off, _axis, _lim) in enumerate(K.JOINTS):
        T = T @ K.trans(off)
        if i == 2:          # up_roll: 它的原点就是肘
            return T[:3, 3].copy()
        T = T @ K.frame(K.rot(K.JOINTS[i][2], q[i]))
    return T[:3, 3].copy()


def main():
    bx, by, byaw = L.SERVE_POSE[0], L.SERVE_POSE[1], L.SERVE_POSE[2]
    Tcup = P.cup_in_base(bx, by, byaw, L.CUP_CARRY_R, L.CUP_CARRY_Z)
    print('carry 杯位 = %s' % np.round(Tcup[:3, 3], 3))
    base = (bx, by, byaw)
    for side in ('L', 'R'):
        Tp = (P.cup_grasp_palm(Tcup, side) if side == 'L'
              else P.cup_cradle_palm(Tcup, side))
        sh = np.array(K.SHOULDER[side])
        print('\n== %s 侧  掌心目标 t=%s' % (side, np.round(Tp[:3, 3], 3)))
        seeds = [('Q_REST', K.Q_REST.copy()), ('Q_HANG', K.Q_HANG.copy())]
        for v in (-2.0, -1.7, -1.4, -0.9, 0.9, 1.4, 1.7, 2.0):
            s = K.Q_REST.copy()
            s[2] = v
            seeds.append(('up_roll=%+.1f' % v, s))
        sign = 1.0 if side == 'L' else -1.0
        best = None
        for name, seed in seeds:
            q, err, how = K.ik_world(side, Tp, base, seed)
            el = elbow_local(side, q)
            palm = K.fk_world(side, q, base)[:3, 3]
            d_el = el - sh
            # '肘朝外'打分: 横向外扩 - 1.2×"肘举得比手还高"
            out = sign * d_el[1]
            high = max(0.0, el[2] - palm[2])
            score = out - 1.2 * high
            print('   %-12s err=%.4f [%s] 肘=%-28s 相对肩 y=%+.3f z=%+.3f  '
                  'up_roll=%+.2f  out=%+.3f score=%+.3f'
                  % (name, err, how, np.round(el, 3), d_el[1], d_el[2],
                     q[2], out, score))
            if err < 2e-3 and (best is None or score > best[0]):
                best = (score, name, q)
        if best:
            print('   >>> 选中 %s (score %+.3f) up_roll=%+.2f q=%s'
                  % (best[1], best[0], best[2][2], np.round(best[2], 2)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
