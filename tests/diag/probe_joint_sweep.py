#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""probe_joint_sweep.py —— 单关节阶跃扫描: 判断"到不了位"是偏置还是钳位

P84 卡在一个怪现象: Q_REST 里三个 pitch 关节 (lift/elbow/wrist_p) 稳定停在
离目标 6~16° 的地方, 而把 p_gain 从 60 提到 240 误差几乎不变 —— 说明不是
"力平衡的稳态误差"(那会随 p_gain 成反比缩小), 只可能是以下几种:
  (a) 关节被物理钳住 (碰撞/限位/别的东西顶着)
  (b) 指令根本没进到控制器 (话题名对不上, 或者插件没加载)
  (c) 读回来的角度与指令不在同一个零点

本探针逐个下发阶跃指令, 打印每个目标下的稳定实测值:
  - 若实测始终 = 目标 + 常数 → (c) 零点偏置
  - 若实测停在某个固定值不再变化 → (a) 钳位, 那个值就是限位
  - 若实测根本不动 → (b) 指令不通

用法: python3 tests/diag/probe_joint_sweep.py R_lift_joint -0.3 -0.6 -1.0 -1.5
"""
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

JOINT = sys.argv[1] if len(sys.argv) > 1 else 'R_lift_joint'
VALS = [float(v) for v in sys.argv[2:]] or [0.0, -0.3, -0.6, -1.0, -1.5]


class Sweep(Node):

    def __init__(self):
        super().__init__('probe_joint_sweep')
        self.js = {}
        self.create_subscription(JointState, '/drink/joint_state', self._js, 20)
        self.pub = self.create_publisher(Float64, '/drink/cmd/%s' % JOINT, 10)

    def _js(self, msg):
        self.js = dict(zip(msg.name, msg.position))

    def spin(self, s):
        t0 = time.time()
        while time.time() - t0 < s:
            rclpy.spin_once(self, timeout_sec=0.02)


def main():
    rclpy.init()
    n = Sweep()
    n.spin(1.0)
    print('关节 %s  起始 %.4f' % (JOINT, n.js.get(JOINT, float('nan'))))
    print('  目标      实测      差        (等待 2.5s 收敛)')
    for v in VALS:
        t0 = time.time()
        while time.time() - t0 < 2.5:
            m = Float64()
            m.data = v
            n.pub.publish(m)
            n.spin(0.02)
        a = n.js.get(JOINT, float('nan'))
        print('  %+7.3f  %+7.3f  %+7.3f' % (v, a, a - v))
    # 最后把整条臂都打出来: 若"只动了 R_lift_joint, 但变化出现在别的名字上",
    # 说明 joint_state 的名字与真实关节错位了(而不是控制不到位)
    side = JOINT[0]
    print()
    print('--- 该侧全部关节最后读数 ---')
    for j in ('pan', 'lift', 'up_roll', 'elbow', 'fore_roll', 'wrist_p',
              'wrist_r'):
        k = '%s_%s_joint' % (side, j)
        print('   %-16s %+8.4f' % (k, n.js.get(k, float('nan'))))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
