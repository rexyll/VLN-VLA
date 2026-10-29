#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""probe_arm.py —— 单臂标定探针: 找 FK 与仿真对不上的那个常数

P84 的 FK 自检出现 "姿态误差 1.1° 但位置误差 742mm", 这种组合只有三种可能:
  (a) 平移基准不同 (base 用地面系, 仿真用 chassis 系, 差一个 z)
  (b) 某个 joint 的 <pose> 偏置写错, 但轴方向对 → 只错位置
  (c) 某关节没到位 (指令 Q_REST, 实测差 16°) → 位置全错

本探针把三种可能一次分清:
  1) 逐个关节打印 目标/实测/误差  —— 若某关节恒定差一个值, 是 (c)
  2) 打印 sim 里 chassis / torso / R_L0..R_L6 / R_palm 的世界位置
     与 arm_kin.fk_joint_frames 的预测值逐点比对 —— 第一个出现偏差的
     关节就是偏置出问题的那个, 是 (b) 的特征
  3) 单独打印 z 分量差 —— (a) 的特征是"所有点 z 差同一个常数, xy 不差"

用法: python3 tests/diag/probe_arm.py [R|L] [保持秒数]
"""
import math
import sys
import time

import numpy as np
import rclpy
import gz.transport13 as gz_transport
from gz.msgs10.pose_v_pb2 import Pose_V
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from vla_follow import arm_kin as K

SIDE = (sys.argv[1] if len(sys.argv) > 1 else 'R').upper()
HOLD = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0
TRUTH = '/world/follow_world/dynamic_pose/info'


class Probe(Node):

    def __init__(self):
        super().__init__('probe_arm')
        self.js = {}
        self.truth = {}
        self.pub = {}
        names = ['%s_%s_joint' % (SIDE, j[0]) for j in K.JOINTS]
        names += ['%s_joint' % x for x in K.hand_joints(SIDE)]
        for n in names:
            self.pub[n] = self.create_publisher(Float64,
                                                '/drink/cmd/%s' % n, 10)
        self.create_subscription(JointState, '/drink/joint_state', self._js, 20)
        self.gz = gz_transport.Node()
        self.gz.subscribe(Pose_V, TRUTH, self._truth)

    def _js(self, msg):
        self.js = dict(zip(msg.name, msg.position))

    def _truth(self, msg):
        t = {}
        for p in msg.pose:
            t[p.name] = np.array([p.position.x, p.position.y, p.position.z])
        self.truth = t

    def spin(self, s):
        t0 = time.time()
        while time.time() - t0 < s:
            rclpy.spin_once(self, timeout_sec=0.02)

    def avg(self, s=0.8):
        """窗口平均。关节是欠阻尼的, 单点采样会把残余抖动读成"稳态误差",
        这正是 P84 一开始误判的原因, 所以读数一律取平均。"""
        acc = {}
        cnt = 0
        t0 = time.time()
        while time.time() - t0 < s:
            rclpy.spin_once(self, timeout_sec=0.02)
            for k, v in self.js.items():
                acc[k] = acc.get(k, 0.0) + v
            cnt += 1
        return {k: v / max(cnt, 1) for k, v in acc.items()}


def main():
    rclpy.init()
    p = Probe()
    p.spin(1.5)

    tgt = {}
    for (jn, _o, _a, _l), v in zip(K.JOINTS, K.Q_REST):
        tgt['%s_%s_joint' % (SIDE, jn)] = v
    for n, v in zip(K.hand_joints(SIDE), K.grip_values(SIDE, K.GRIP_OPEN)):
        tgt[n + '_joint'] = v

    t0 = time.time()
    while time.time() - t0 < HOLD:
        for n, v in tgt.items():
            m = Float64()
            m.data = float(v)
            p.pub[n].publish(m)
        p.spin(0.02)
        time.sleep(0.03)
    js = p.avg(0.8)

    print('=' * 70)
    print('关节到位情况 (%s 侧, 目标 Q_REST)' % SIDE)
    print('=' * 70)
    q = []
    for (jn, _o, _a, lim), v in zip(K.JOINTS, K.Q_REST):
        n = '%s_%s_joint' % (SIDE, jn)
        actual = js.get(n, float('nan'))
        q.append(actual)
        print('  %-14s 目标 %+7.3f  实测 %+7.3f  差 %+7.3f rad (%+6.2f°)%s'
              % (jn, v, actual, actual - v, np.degrees(actual - v),
                 '' if abs(actual - v) < 0.05 else '   <<< 未到位'))
    q = np.array(q)

    print()
    print('=' * 70)
    print('链路逐点比对: sim 真值 vs arm_kin 预测')
    print('=' * 70)
    Tb = p.truth.get('chassis')
    if Tb is None:
        print('没有 chassis 真值, 退出')
        return 1
    # 压到地面投影系: arm_kin 的 SHOULDER_Z 是离地高度
    Tb = Tb.copy()
    Tb[2] = 0.0
    print('  chassis 世界位置 (%.4f, %.4f, %.4f)' % tuple(Tb))

    # 预测用 chassis 位姿 + 实测 q
    from vla_follow import arm_kin as KK
    T = KK.frame(t=KK.SHOULDER[SIDE])
    pred = {'shoulder': T[:3, 3] + Tb}
    for i, (jn, off, axis, _lim) in enumerate(K.JOINTS):
        T = T @ KK.trans(off)
        pred['%s_L%d' % (SIDE, i)] = T[:3, 3] + Tb
        T = T @ KK.frame(KK.rot(axis, q[i]))
    palm_pred = (T @ KK.trans((0.0, 0.0, -KK.PALM_OFF)))[:3, 3] + Tb
    pred['palm'] = palm_pred

    keys = ['torso'] + ['%s_L%d' % (SIDE, i) for i in range(7)] + ['%s_palm' % SIDE]
    for k in keys:
        sim = p.truth.get(k)
        pk = 'palm' if k.endswith('_palm') else (
            'shoulder' if k == 'torso' else k)
        if sim is None:
            print('  %-10s sim 无此实体' % k)
            continue
        if pk not in pred:
            print('  %-10s 无预测' % k)
            continue
        d = pred[pk] - sim
        print('  %-10s sim (%.3f,%.3f,%.3f)  pred (%.3f,%.3f,%.3f)  '
              '差 (%+.1f, %+.1f, %+.1f)mm'
              % (k, sim[0], sim[1], sim[2],
                 pred[pk][0], pred[pk][1], pred[pk][2],
                 d[0] * 1000, d[1] * 1000, d[2] * 1000))
    # ---- 用真值连杆位置反算关节角, 和 joint_state 对照
    # 这一段是为了把"读数骗人"和"控制不到位"彻底分开:
    #   同一时刻, 由 sim 里连杆位置反算出的角度 vs joint_state 报的角度
    #   两者一致 → 控制确实不到位; 两者不一致 → 读数不可信
    print()
    print('=' * 70)
    print('真值反算关节角 vs joint_state 读数')
    print('=' * 70)

    def ang(a, b):
        """a→b 矢量相对 -z 的偏角(绕 y 轴, 正负按 x 分量)"""
        d = b - a
        if np.linalg.norm(d) < 1e-6:
            return float('nan')
        return math.atan2(-d[0], -d[2])

    pairs = (('lift', 'R_L1', 'R_L2'), ('elbow', 'R_L2', 'R_L4'),
             ('wrist_p', 'R_L6', 'R_palm'))
    for jn, a, b in pairs:
        Ta, Tb_ = p.truth.get(a), p.truth.get(b)
        if Ta is None or Tb_ is None:
            print('  %-8s 真值缺 %s/%s' % (jn, a, b))
            continue
        th = ang(Ta, Tb_)
        rep = js.get('%s_%s_joint' % (SIDE, jn), float('nan'))
        print('  %-8s 真值 %+7.3f   joint_state %+7.3f   差 %+7.3f'
              % (jn, th, rep, rep - th))

    print()
    print('>> 若所有行差值都是一个常数 → 平移基准错(把 base 当成地面系)')
    print('>> 若从某一行开始突然跳变 → 那个关节的 <pose> 偏置写错')
    p.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
