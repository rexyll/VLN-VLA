#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""p102_joints_audit.py —— 离线体检"整条动作链有没有违反人体关节规律"(P103)。

用户 2026-10-28: "从水壶倒水的时候, 机器人左手臂肘部弯向内侧……另外检查其他
运动关节是否有违背人体肢体正常运动轨迹的现象"。

本脚本不起仿真, 只做正/逆解, 按**运行时的真实种子链**把每个关键位形解一遍:
    种子从 Q_REST 起, 每解出一个位形就把它交给下一步当种子(与 drink_task 里
    `reach()` 的 q_now 种子等价), 因此能复现"运行时到底落在哪个分支上"。

对每个位形给出人体规则的六项体检(本体系, 单位 rad/m):
    R1 肘反关节       elbow > 0            —— 人的肘不能向后折
    R2 肘穿过中线     L: elbow_y < 0 / R: > 0
    (原 R3 已删, 见 arm_kin.violation_list)
    R4 上臂内旋过大   |up_roll| > 1.90 (109°) —— 肩的旋转余量(见 arm_kin 注释)
    R5 前臂扭转过大   |fore_roll + wrist_r| > 2.00 (115°)
    R6 腕反折         |wrist_p| > 1.50 (86°, 即机械限位)
    R7 肘过折         |elbow| > 2.10 (120°)
另报 elbow_metrics 的 out_line / out_base / droop 三个口径(与验收同源)。

用法:  python3 tests/diag/p102_joints_audit.py            # 现网配置(elbow=None)
       python3 tests/diag/p102_joints_audit.py out        # 全程肘朝外
"""
import sys

import numpy as np

sys.path.insert(0, 'vla_ws/src/vla_follow')
from vla_follow import arm_kin as K          # noqa: E402
from vla_follow import drink_poses as P     # noqa: E402
from vla_follow import scene_layout as L    # noqa: E402

# 阈值直接取 arm_kin 那一套, 免得审计脚本和运行时策略各说各话
# (P103 把 R4 从 1.75 校准到 1.90、R6 从 1.40 校准到 1.50, 就是同一件事)
R4_MAX = K.HUMAN_UPROLL      # |up_roll|
R5_MAX = K.HUMAN_TWIST       # |fore_roll + wrist_r|
R6_MAX = K.HUMAN_WRIST_P     # |wrist_p|
R7_MAX = K.HUMAN_ELBOW       # |elbow|


def body_of(side, q):
    """本体系下的肘点/掌心**绝对**坐标(判"过没过身体中线"只能看绝对 y)。"""
    el = K.elbow_pos(side, q)
    palm = K.fk(side, q)[:3, 3]
    return el, palm


def check(side, tag, q, T_tgt, base):
    pe = K.pose_err(K.fk_world(side, q, base), T_tgt)
    ep, er = float(np.linalg.norm(pe[:3])), float(np.linalg.norm(pe[3:]))
    m = K.elbow_metrics(side, q)
    el, palm = body_of(side, q)
    sgn = 1.0 if side == 'L' else -1.0
    rows = [
        ('R1 肘反关节', 'elbow=%+.2f' % q[3], q[3] > 0.02),
        ('R2 肘过中线', '肘y=%+.0fmm' % (el[1] * 1e3), sgn * el[1] < -0.005),
        ('R4 肩内旋', 'up_roll=%+.2f' % q[2], abs(q[2]) > R4_MAX),
        ('R5 前臂扭转', 'fore+wrist=%+.2f' % (q[4] + q[6]),
         abs(q[4] + q[6]) > R5_MAX),
        ('R6 腕反折', 'wrist_p=%+.2f' % q[5], abs(q[5]) > R6_MAX),
        ('R7 肘过折', 'elbow=%+.2f' % q[3], abs(q[3]) > R7_MAX),
    ]
    bad = [r[0] for r in rows if r[2]]
    print('%-3s %-16s err %4.1f/%4.1fmm 外扩线%+6.1f 基外扩%+6.1f droop%+6.1f '
          'up_roll%+5.2f 肘%+5.2f twist%+5.2f 代价%.2f %s'
          % (side, tag, ep * 1e3, er * 1e3, m['out_line_mm'],
             m['out_base_mm'], m['droop_mm'], q[2], q[3], q[4] + q[6],
             K.human_penalty(side, q),
             ('★ ' + ' '.join(bad)) if bad else 'OK'))
    if bad:
        print('       q = %s' % np.round(q, 2))
        for name, val, hit in rows:
            if hit:
                print('       %-12s %s' % (name, val))
    return ep, er, bad


def run(mode):
    seed = {'L': K.Q_REST.copy(), 'R': K.Q_REST.copy()}
    print('=== mode: elbow=%s ===' % mode)
    worst = []
    for side, tag, T, base in P.cases():
        q, _e, how = K.ik_world(side, T, base, seed[side], elbow=mode)
        ep, er, bad = check(side, tag, q, T, base)
        if ep < 0.01:
            seed[side] = q
        if bad:
            worst.append((side, tag, bad))
    print('--- 违反人体规则的位形: %d 个' % len(worst))
    for side, tag, bad in worst:
        print('    %s %s: %s' % (side, tag, ' '.join(bad)))
    return worst


if __name__ == '__main__':
    mode = sys.argv[1] if len(sys.argv) > 1 else None
    run(mode)
