#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""probe_palm_gt.py —— 用 gz 真值校验 arm_kin 的 FK(P85 前置探针)

问题: P84 曾记录"/world/follow_world/dynamic_pose/info 里 link 的位姿名字错位
一位(R_L1 的 pose 其实是肩部偏置)"。但那时虚拟机里叠着 19 个 gz 世界, 订阅方
读到的是 19 份交错位姿 —— 这条结论很可能是那个污染的假象。

本探针把 arm_kin.fk() 算出的手掌位姿和 gz 里 R_palm/L_palm 的真值位姿逐项对比。
若误差 <5mm, 说明: ①FK 与 SDF 一致 ②link 真值可用 ③drink_task 的 /drink/ck
自检可以拿真值当参照(比"FK 自比"有意义得多)。

用法(世界已在跑, 已 source install/setup.bash):
    python3 tests/diag/probe_palm_gt.py
"""
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

sys.path.insert(0, '/home/earl/workspace/VLA/vla_ws/src/vla_follow')
from vla_follow import arm_kin as K          # noqa: E402

import gz.transport13 as gz_transport                       # noqa: E402
from gz.msgs10.pose_v_pb2 import Pose_V as GzPose_V         # noqa: E402


def gz_pose_to_T(p):
    q = p.orientation
    R = K.quat_to_mat(q.x, q.y, q.z, q.w)
    return K.frame(R, (p.position.x, p.position.y, p.position.z))


class Probe(Node):

    def __init__(self):
        super().__init__('probe_palm_gt')
        self.js = {}
        self.links = {}
        self.create_subscription(JointState, '/drink/joint_state', self._js, 20)
        self._gz = gz_transport.Node()
        self._gz.subscribe(GzPose_V,
                           '/world/follow_world/dynamic_pose/info',
                           self._pose)

    def _js(self, msg):
        self.js.update(zip(msg.name, msg.position))

    def _pose(self, msg):
        for p in msg.pose:
            self.links[p.name] = p

    def spin_for(self, s):
        t0 = time.time()
        while time.time() - t0 < s:
            rclpy.spin_once(self, timeout_sec=0.005)


def main():
    rclpy.init()
    n = Probe()
    n.spin_for(3.0)

    if not n.js:
        print('没有 /drink/joint_state, 世界没起?')
        return 1

    print('=' * 74)
    print('FK 手掌 vs gz 真值 (R_palm / L_palm)')
    print('=' * 74)
    ok = True
    for side in ('R', 'L'):
        names = ['%s_%s_joint' % (side, j[0]) for j in K.JOINTS]
        miss = [x for x in names if x not in n.js]
        if miss:
            print('%s 侧缺关节: %s' % (side, miss))
            ok = False
            continue
        q = np.array([n.js[x] for x in names])
        T_fk = K.fk(side, q)
        key = '%s_palm' % side
        if key not in n.links:
            print('%s 不在真值里; 现有 link 名: %s'
                  % (key, sorted(n.links)[:8]))
            ok = False
            continue
        T_gt = gz_pose_to_T(n.links[key])
        e = K.pose_err(T_fk, T_gt)
        dp = float(np.linalg.norm(e[:3]))
        dr = math.degrees(float(np.linalg.norm(e[3:])))
        print('%s  q(°)= %s' % (side, ' '.join('%+6.1f' % math.degrees(v)
                                                for v in q)))
        print('   FK  t=(%+.4f %+.4f %+.4f)  %s'
              % (T_fk[0, 3], T_fk[1, 3], T_fk[2, 3], K.lamp(T_fk)))
        print('   GT  t=(%+.4f %+.4f %+.4f)  %s'
              % (T_gt[0, 3], T_gt[1, 3], T_gt[2, 3], K.lamp(T_gt)))
        print('   差  位置 %.1f mm  姿态 %.2f°'
              % (dp * 1000.0, dr))
        ok = ok and dp < 0.005

    # 顺带把每个臂 link 的真值拉出来看看名字到底对不对
    print('-' * 74)
    print('各臂 link 真值位置(z 递减应是 pan→lift→…→palm):')
    for side in ('R',):
        for nm in sorted(n.links):
            if nm.startswith(side + '_') and 'hand' not in nm:
                t = n.links[nm].position
                print('   %-14s (%+.3f %+.3f %+.3f)' % (nm, t.x, t.y, t.z))
    print('=' * 74)
    print('结论: %s' % ('FK 与真值一致(<5mm), link 真值可用于自检'
                        if ok else 'FK 与真值不一致, 需继续排查'))
    rclpy.shutdown()
    return 0 if ok else 2


if __name__ == '__main__':
    sys.exit(main())
