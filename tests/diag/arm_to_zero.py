#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arm_to_zero.py —— 把双臂摆到 q=0(笔直下垂), 用于复现"冷启动"工况

用途: 验证 drink_task 上电时序。q=0 与 Q_REST 差 30~50°, 若节点把 0 当
默认指令先发出去, home() 就得先走完这条错误行程(实测 12s + 超时)。
本脚本把手臂摆到最"不 home"的合法位形, 然后立刻跑
    python3 -m vla_follow.drink_task --once
看 home 用时是否仍然 <2s。
"""
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from vla_follow import arm_kin as K
from vla_follow.joint_servo import JointServo


class N(Node):
    def __init__(self):
        super().__init__('arm_to_zero')
        self.js = {}
        self.create_subscription(JointState, '/drink/joint_state', self._cb, 20)

    def _cb(self, m):
        self.js.update(zip(m.name, m.position))

    def spin_for(self, s):
        t0 = time.time()
        while time.time() - t0 < s:
            rclpy.spin_once(self, timeout_sec=0.005)


def main():
    rclpy.init()
    n = N()
    n.spin_for(1.5)
    t0 = time.time()
    ok = True
    for s in ('R', 'L'):
        a = JointServo(n, s)
        tgt = [0.0] * 7
        o, e = a.move(tgt, timeout=10.0)
        ok = ok and o
        print('%s 侧到 q=0: %s  用时 %.1fs'
              % (s, '到位' if o else '超时', time.time() - t0))
        # 逐轴打印: 哪一根不动就能直接看出来
        print('   轴   : %s' % ' '.join('%8s' % j[0] for j in K.JOINTS))
        print('   目标 : %s' % ' '.join('%+8.3f' % v for v in tgt))
        print('   实测 : %s' % ' '.join('%+8.3f' % v for v in a.mea))
        print('   残差°: %s' % ' '.join('%+8.1f' % v
                                        for v in (e * 57.2958)))
        t0 = time.time()
    n.spin_for(0.5)
    rclpy.shutdown()
    return 0 if ok else 2


if __name__ == '__main__':
    sys.exit(main())
