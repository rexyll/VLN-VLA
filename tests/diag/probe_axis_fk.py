#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""probe_axis_fk.py —— 逐轴激励 + FK/真值对比(P85 自检的强化版)

为什么需要它
===========
Q_REST = [0, -0.55, 0, -0.85, 0, 0.35, 0], 三个 **z 轴关节**
(up_roll / fore_roll / wrist_r) 全是 0。所以"home 位 FK 与真值 0.0mm"这个
结论**没有验证任何 z 轴方向** —— 如果 SDF 里某一侧某个轴被镜像成 -z, home 位
照样完美吻合, 但一旦转起来就会朝反方向跑(表现为"下发 +0.7 实际到 -0.7",
外层伺服会误判成"卡住了")。

本探针在 Q_REST 基线上逐个叠加单轴激励, 每个位形都做两件事:
  ① FK(实测 q) vs gz 真值 → 位置/姿态误差(验证 SDF 与 arm_kin 是否同向)
  ② 实测 q vs 指令 q      → 到位精度 + 用时(顺带抓"间歇性卡顿")

用法:
    python3 tests/diag/probe_axis_fk.py
    python3 tests/diag/probe_axis_fk.py R      # 只测一侧
"""
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from vla_follow import arm_kin as K
from vla_follow.joint_servo import JointServo

try:
    import gz.transport13 as gz_transport
    from gz.msgs10.pose_v_pb2 import Pose_V as GzPose_V
    HAVE_GZ = True
except Exception:
    HAVE_GZ = False

# (标签, 在 Q_REST 上的叠加量)
CASES = [
    ('回 home',            [0.0] * 7),
    ('pan +0.60',          [0.60, 0, 0, 0, 0, 0, 0]),
    ('lift +0.40',         [0, 0.40, 0, 0, 0, 0, 0]),
    ('up_roll +0.70',      [0, 0, 0.70, 0, 0, 0, 0]),
    ('elbow +0.40',        [0, 0, 0, 0.40, 0, 0, 0]),
    ('fore_roll +0.80',    [0, 0, 0, 0, 0.80, 0, 0]),
    ('wrist_p -0.50',      [0, 0, 0, 0, 0, -0.50, 0]),
    ('wrist_r +0.90',      [0, 0, 0, 0, 0, 0, 0.90]),
    ('混合位形',           [0.40, 0.30, 0.50, 0.30, -0.60, -0.35, 0.70]),
    ('回 home',            [0.0] * 7),
]


class Probe(Node):

    def __init__(self):
        super().__init__('probe_axis_fk')
        self.js = {}
        self.links = {}
        self.create_subscription(JointState, '/drink/joint_state', self._cb, 20)
        if HAVE_GZ:
            self._gz = gz_transport.Node()
            self._gz.subscribe(GzPose_V,
                               '/world/follow_world/dynamic_pose/info',
                               self._gt)

    def _cb(self, m):
        self.js.update(zip(m.name, m.position))

    def _gt(self, m):
        for p in m.pose:
            self.links[p.name] = p

    def spin_for(self, s):
        t0 = time.time()
        while time.time() - t0 < s:
            rclpy.spin_once(self, timeout_sec=0.005)


def gt_T(p):
    q = p.orientation
    return K.frame(K.quat_to_mat(q.x, q.y, q.z, q.w),
                   (p.position.x, p.position.y, p.position.z))


def main():
    sides = [a for a in sys.argv[1:] if a in ('R', 'L')] or ['R', 'L']
    rclpy.init()
    n = Probe()
    n.spin_for(2.0)
    if not n.js:
        print('没有 /drink/joint_state')
        return 1
    servo = {s: JointServo(n, s) for s in sides}

    print('=' * 92)
    print('逐轴激励: FK vs 真值  (位置 mm / 姿态 °)   +   到位精度 / 用时')
    print('=' * 92)
    hdr = '%-14s %-3s %8s %8s %8s %8s %8s' % (
        '位形', '侧', '指令残差°', 'FK-GT mm', 'FK-GT °', '用时s', '结果')
    print(hdr)
    print('-' * 92)
    bad = []
    for tag, delta in CASES:
        q_tgt = np.clip(K.Q_REST + np.array(delta, float), K.LIM_LO, K.LIM_HI)
        for s in sides:
            t0 = time.time()
            ok, err = servo[s].move(q_tgt, timeout=8.0)
            dt = time.time() - t0
            n.spin_for(0.15)
            names = ['%s_%s_joint' % (s, j[0]) for j in K.JOINTS]
            q = np.array([n.js[x] for x in names], float)
            dq = math.degrees(float(np.max(np.abs(q - q_tgt))))
            key = '%s_palm' % s
            if key in n.links:
                e = K.pose_err(K.fk(s, q), gt_T(n.links[key]))
                dp = float(np.linalg.norm(e[:3])) * 1000.0
                dr = math.degrees(float(np.linalg.norm(e[3:])))
            else:
                dp = dr = float('nan')
            good = ok and dq < 2.0 and dp < 5.0 and dr < 2.0
            if not good:
                bad.append((tag, s, dq, dp, dr, dt))
            print('%-14s %-3s %8.1f %8.1f %8.2f %8.1f %8s'
                  % (tag, s, dq, dp, dr, dt, 'OK' if good else '**NG**'))
    print('=' * 92)
    if bad:
        print('异常 %d 项:' % len(bad))
        for t, s, dq, dp, dr, dt in bad:
            print('   %-14s %s  残差 %.1f°  FK-GT %.1fmm / %.2f°  用时 %.1fs'
                  % (t, s, dq, dp, dr, dt))
        print('结论: 不通过')
    else:
        print('结论: 通过(7 个轴方向、到位精度、FK 一致性全部符合)')
    print('=' * 92)
    rclpy.shutdown()
    return 0 if not bad else 2


if __name__ == '__main__':
    sys.exit(main())
