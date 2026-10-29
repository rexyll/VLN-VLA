#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""diag_p88_turn.py —— 定位"转身之后杯子跟着跑了 15cm"

现象: [9] 转身到位(base yaw 61.6°), 但杯心离 base 只有 0.352m(应在 0.50m),
杯柄朝向也差 18.6°。怀疑是转身过程中**左臂的关节角变了**(杯子钉在左掌心
@冻结偏移, 臂不动杯子就不会动), 也可能是 FK/base 的读法有问题。

本探针把转身前后各量一次: L 侧 7 个关节角 / 掌心世界位姿 / 杯子世界位姿 /
base, 一比就知道是谁动了。

用法: bash tools/start_scene.sh; python3 tests/diag/diag_p88_turn.py
"""
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, 'vla_ws', 'src', 'vla_follow'))

import numpy as np  # noqa: E402
import rclpy  # noqa: E402

from vla_follow import arm_kin as K           # noqa: E402
from vla_follow import drink_motion as M      # noqa: E402
from vla_follow import drink_poses as P       # noqa: E402
from vla_follow import scene_layout as L      # noqa: E402
from vla_follow.drink_task import DrinkTask   # noqa: E402


def dump(node, tag):
    q = M.q_meas(node, 'L')
    Tp = M.wrist(node, 'L')
    Tc = M.object_pose(node, 'cup')
    bx, by, byaw = node.base
    print('--- %s ---' % tag)
    print('  base      = (%.3f, %.3f, %.1f°)' % (bx, by, math.degrees(byaw)))
    print('  L joints  = %s' % np.round(np.degrees(q), 1))
    print('  L palm    = %s   |palm-base|=%.3f'
          % (np.round(Tp[:3, 3], 3),
             float(np.linalg.norm(Tp[:2, 3] - np.array([bx, by])))))
    print('  cup       = %s   |cup-base|=%.3f  yaw=%.1f°'
          % (np.round(Tc[:3, 3], 3),
             float(np.linalg.norm(Tc[:2, 3] - np.array([bx, by]))),
             math.degrees(math.atan2(Tc[1, 0], Tc[0, 0]))))
    # 掌心在 base 系里的位置(与 base 无关的纯运动学量)
    Tb = K.base_pose(bx, by, byaw)
    p = np.linalg.inv(Tb) @ np.append(Tp[:3, 3], 1.0)
    print('  palm@base = %s' % np.round(p[:3], 3))
    return q.copy(), Tp.copy(), Tc.copy()


def main():
    rclpy.init()
    node = DrinkTask()
    t0 = 0
    while t0 < 10.0 and len(node.js) < 34:
        node.spin_for(0.2)
        t0 += 0.2
    bx, by, _ = L.SERVE_POSE
    M.place_robot(node, K.frame(K.rot_z(0.0), (bx, by, 0.0)))
    node.home()
    r = M.pour_into_cup(node, fill=0.65)
    print('pour ok=%s res=%s' % (r['ok'], r['res']))
    Tk_home = P.kettle_on_table()
    M.put_down(node, 'R', 'kettle', Tk_home)
    node.arm['R'].move(K.Q_REST, timeout=10.0)
    node.spin_for(0.4)
    M.place_held(node, 'L', 'cup', P.carry_cup(bx, by, 0.0), tol_mm=5.0)
    node.spin_for(0.6)
    q0, Tp0, Tc0 = dump(node, '放壶+捧杯后(转身前)')

    M.hand_onto(node, 'R', 'cup', lambda T: P.cup_cradle_palm(T, 'R'))
    dump(node, '右手贴上后')
    node.spin_for(1.0)
    dump(node, '右手贴上+1s')

    ok, err = M.turn_base(node, L.HANDOVER_YAW)
    print('turn_base -> %s, err=%.2f°' % (ok, err))
    node.spin_for(0.8)
    q1, Tp1, Tc1 = dump(node, '转身后')

    print('--- 差量 ---')
    print('  dq (deg)  = %s' % np.round(np.degrees(q1 - q0), 1))
    print('  |dq|max   = %.1f°' % np.degrees(np.max(np.abs(q1 - q0))))
    print('  palm 位移 = %.1f mm' % (np.linalg.norm(Tp1[:3, 3] - Tp0[:3, 3]) * 1000))
    print('  cup 位移  = %.1f mm' % (np.linalg.norm(Tc1[:3, 3] - Tc0[:3, 3]) * 1000))
    print('  cup 相对掌心位移 = %.1f mm'
          % (np.linalg.norm((Tc1[:3, 3] - Tp1[:3, 3])
                            - (Tc0[:3, 3] - Tp0[:3, 3])) * 1000))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
