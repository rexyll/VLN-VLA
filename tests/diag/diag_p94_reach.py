#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""diag_p94_reach.py —— P94 递杯可达性扫描(离线, 不进仿真)

问题: `_present_target()` 把杯子送到 `r = clamp(0.50, 0.62, 车→人手距离)`。
人站在名义接水位时 r=0.62 正好够得着; 但 §6.7 要给人加 ±0.15m 的位置扰动,
人往外站时车→人手变成 0.77m, 而 r 上限卡死在 0.62 —— 杯子会停在离人手
150mm 的地方, 后面 `hand_to` 一交接杯子就"瞬移"过去。

所以先离线把"能伸多远"扫出来: 递杯方向相对名义偏 ±30°、前伸 r 从 0.50
扫到 0.85, 左右手各过一遍 KDL 逆解, 报位置残差 / 姿态残差 / 限位余量。
这样改 `CUP_PRESENT_R` 上限之前就有数, 不用靠仿真里试错。

用法:
    python3 tests/diag/diag_p94_reach.py
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, 'vla_ws', 'src', 'vla_follow'))

import numpy as np                                        # noqa: E402

from vla_follow import arm_kin as K                       # noqa: E402
from vla_follow import drink_poses as P                   # noqa: E402
from vla_follow import scene_layout as L                  # noqa: E402

OK_MM = 10.0      # 位置残差上限(与 check_ik.py 同口径)
MARGIN_DEG = 3.0  # 限位余量下限


def probe(side, base, r, z, seed):
    """把杯子放到 base 系 (r, 0, z), 算该侧掌心的逆解"""
    bx, by, byaw = base
    T = P.cup_in_base(bx, by, byaw, r, z)
    Tp = P.cup_cradle_palm(T, side)
    q, _e, how = K.ik_world(side, Tp, base, seed)
    err = K.pose_err(K.fk_world(side, q, base), Tp)
    ep = float(np.linalg.norm(err[:3])) * 1000.0
    er = math.degrees(float(np.linalg.norm(err[3:])))
    marg = math.degrees(float(np.min(np.minimum(q - K.LIM_LO, K.LIM_HI - q))))
    return ep, er, marg, how, q


def main():
    print('PyKDL 可用 =', K.HAVE_KDL)
    bx, by, _ = L.SERVE_POSE
    print('服务位 = (%.2f, %.2f)  名义递杯朝向 HANDOVER_YAW = %.1f°  '
          '名义前伸 CUP_PRESENT_R = %.2f m'
          % (bx, by, math.degrees(L.HANDOVER_YAW), L.CUP_PRESENT_R))
    print('-' * 96)
    print('%-6s %-6s | %-8s %-8s | %-8s %-8s' % ('偏角', '前伸',
                                                 'L 残差mm', 'L 余量°',
                                                 'R 残差mm', 'R 余量°'))
    print('-' * 96)
    seed = {'L': K.Q_REST.copy(), 'R': K.Q_REST.copy()}
    # 每个方向都从 Q_REST 起算(仿真里每次递杯也是从捧杯位连续过去的,
    # 这里刻意用同一个种子, 才看得出"方向"本身的影响而不是种子差异)
    reach_max = {}
    for alpha in (-30, -15, 0, 15, 30):
        byaw = L.HANDOVER_YAW + math.radians(alpha)
        base = (bx, by, byaw)
        ok_r = []
        for r in (0.50, 0.55, 0.60, 0.62, 0.65, 0.70, 0.75, 0.80, 0.85):
            row = []
            for side in ('L', 'R'):
                ep, er, marg, how, q = probe(side, base, r,
                                             L.CUP_PRESENT_Z, seed[side])
                row.append((ep, er, marg, how, q))
                if ep < 0.010:
                    seed[side] = q
            ep_l, _er_l, mg_l, _h_l, _ = row[0]
            ep_r, _er_r, mg_r, _h_r, _ = row[1]
            flag = ''
            if ep_l > OK_MM or ep_r > OK_MM:
                flag = '  <<< 不可达'
            elif mg_l < MARGIN_DEG or mg_r < MARGIN_DEG:
                flag = '  << 贴限位'
            else:
                ok_r.append(r)
            print('%-6.0f %-6.2f | %-8.2f %-8.1f | %-8.2f %-8.1f%s'
                  % (alpha, r, ep_l, mg_l, ep_r, mg_r, flag))
        reach_max[alpha] = max(ok_r) if ok_r else None
        print('-' * 96)
    print('结论: 各方向"位置残差 <%.0fmm 且限位余量 ≥%.0f°"的最大前伸:'
          % (OK_MM, MARGIN_DEG))
    for a, r in reach_max.items():
        print('  偏角 %+3d°: %s' % (a, '不可达' if r is None else '%.2f m' % r))
    lim = [r for r in reach_max.values() if r is not None]
    if lim:
        print('全方向安全上限 = %.2f m（名义 %.2f m，可放宽 %.0f mm）'
              % (min(lim), L.CUP_PRESENT_R,
                 (min(lim) - L.CUP_PRESENT_R) * 1000.0))
    return 0


if __name__ == '__main__':
    sys.exit(main())
