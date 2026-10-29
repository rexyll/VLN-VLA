#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""diag_p103_off.py —— 抓杯之后核对"冻结偏移"与举杯的掌心目标。

症状: 举杯那一步满 8 轮都停在 ~103mm, 关节顶在 lift=+0.90 / elbow=−2.50。
如果 `carry.grab` 冻结的偏移 `off` 有病(姿态分量过大), 那么
`held_palm_target = T_obj_goal @ inv(off)` 要求的**掌心姿态**就会远离可达范围,
位置再近也解不出来。本脚本把两个候选目标摆在一起对比:
    A 用冻结偏移算的掌心目标   (`M.held_palm_target`)
    B 名义抓取位形算的掌心目标 (`P.cup_grasp_palm(Tcup_pour,'L')`)
并各自离线解一次 IK, 看残差与关节。
"""
import sys

import numpy as np
import rclpy

from vla_follow import arm_kin as K
from vla_follow import drink_motion as M
from vla_follow import drink_poses as P


def main():
    from vla_follow.drink_task import DrinkTask
    rclpy.init()
    node = DrinkTask()
    node.spin_for(3.0)
    T_serve = K.frame(K.rot_z(0.0), (M.L.SERVE_POSE[0], M.L.SERVE_POSE[1], 0.0))
    M.place_robot(node, T_serve)
    node.home()
    node.spin_for(1.0)
    ok, res = M.grasp(node, 'L', 'cup', lambda T: P.cup_grasp_palm(T, 'L'),
                      K.GRIP_CUP)
    print('grasp ok=%s res=%.1fmm' % (ok, res))
    off = node.carry.items['cup']['offset']
    print('off 平移 = %s (|t|=%.1fmm)' % (np.round(off[:3, 3] * 1e3, 1),
                                        np.linalg.norm(off[:3, 3]) * 1e3))
    rpy = K.mat_to_rpy(off[:3, :3])
    print('off 姿态 = %s deg' % np.round(np.degrees(rpy), 1))
    print('掌心实测 = %s' % K.lamp(M.wrist(node, 'L')))

    # 抓握瞬间"实测掌心 vs 命令掌心"的姿态误差 —— 冻结偏移 off 会把它原样
    # 带进后面每一次 place_held 的目标里(实测偏移与名义差 10.8°)
    Tc = M.object_pose(node, 'cup')
    Tcmd = P.cup_grasp_palm(Tc, 'L')
    print('抓握姿态误差: 位置 %.1fmm 姿态 %.1f°'
          % (M.pos_err_mm(M.wrist(node, 'L'), Tcmd),
             M.axis_angle_deg(M.wrist(node, 'L'), Tcmd)))
    off_nom = np.linalg.inv(Tcmd) @ np.asarray(Tc, float)
    print('名义 off: |t|=%.1fmm 姿态差 %.1f°'
          % (np.linalg.norm(off_nom[:3, 3]) * 1e3,
             M.axis_angle_deg(off, off_nom)))

    Tcp = P.cup_for_pour()
    A = M.held_palm_target(node, 'cup', Tcp)
    B = P.cup_grasp_palm(Tcp, 'L')
    print('A(冻结偏移) 掌心目标 t=%s' % np.round(A[:3, 3], 3))
    print('B(名义抓握) 掌心目标 t=%s' % np.round(B[:3, 3], 3))
    print('两者位置差 %.1fmm, 姿态差 %.1f°'
          % (M.pos_err_mm(A, B), M.axis_angle_deg(A, B)))
    q_now = M.q_meas(node, 'L')
    base = node.base
    for name, T in (('A', A), ('B', B)):
        q, err, how = K.ik_world('L', T, base, q_now, elbow='human')
        print('%s: IK err %.1fmm [%s] q=%s' % (name, err * 1e3, how,
                                              np.round(q, 2)))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
