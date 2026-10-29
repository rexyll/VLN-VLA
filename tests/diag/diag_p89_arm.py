#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""diag_p89_arm.py —— 人的手臂到底是绕哪个点转的?

P89 [5] 里 `person_arm_joint` 从 0 转到 -1.0, 杯心抬高 842.5mm。按几何
(手点相对肩关节 (0.20, 0, -0.02), 臂长 0.2m) 最多只能抬 ~177mm, 所以
旋转中心肯定不在肩关节。本探针把三个时刻的 person_arms 世界位姿 / 人手点 /
关节角都打出来, 反推真实的转轴。

用法: bash tools/start_scene.sh; python3 tests/diag/diag_p89_arm.py
"""
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, 'vla_ws', 'src', 'vla_follow'))

import numpy as np  # noqa: E402
import rclpy  # noqa: E402

from vla_follow import drink_motion as M      # noqa: E402
from vla_follow.drink_task import DrinkTask   # noqa: E402


def dump(node, tag):
    Tp = M.object_pose(node, 'person')
    Th = M.person_hand_world(node, Tp)
    p = node.links.get('person_arms')
    print('--- %s ---' % tag)
    print('  person     = %s' % np.round(Tp[:3, 3], 4))
    print('  person yaw = %.2f°'
          % math.degrees(math.atan2(Tp[1, 0], Tp[0, 0])))
    if p is not None:
        print('  arms(raw)  = %s   q=(%.3f,%.3f,%.3f,%.3f)'
              % (np.round([p.position.x, p.position.y, p.position.z], 4),
                 p.orientation.x, p.orientation.y,
                 p.orientation.z, p.orientation.w))
    print('  hand point = %s'
          % (np.round(Th[:3, 3], 4) if Th is not None else None))
    print('  q[person_arm_joint] = %s'
          % np.round(np.degrees(node.js.get('person_arm_joint', [9e9])), 2))
    return Th


def main():
    rclpy.init()
    node = DrinkTask()
    t0 = 0.0
    while t0 < 10.0 and len(node.js) < 34:
        node.spin_for(0.2)
        t0 += 0.2
    print('joint_state=%d  links=%d' % (len(node.js), len(node.links)))

    node.bus.set(['person_arm_joint'], [0.0])
    node.spin_for(2.0)
    a = dump(node, 'joint 0.0')

    node.bus.set(['person_arm_joint'], [-1.00])
    node.spin_for(2.5)
    b = dump(node, 'joint -1.0')

    node.bus.set(['person_arm_joint'], [-0.50])
    node.spin_for(2.5)
    c = dump(node, 'joint -0.5')

    node.bus.set(['person_arm_joint'], [0.0])
    node.spin_for(2.0)
    d = dump(node, 'joint 0.0 (回)')

    if a is not None and b is not None:
        print('位移 0->-1.0 : %s  |d|=%.4f'
              % (np.round(b[:3, 3] - a[:3, 3], 4),
                 np.linalg.norm(b[:3, 3] - a[:3, 3])))
    if a is not None and c is not None:
        print('位移 0->-0.5 : %s  |d|=%.4f'
              % (np.round(c[:3, 3] - a[:3, 3], 4),
                 np.linalg.norm(c[:3, 3] - a[:3, 3])))
    if a is not None and d is not None:
        print('回位残差     : %.4f m' % np.linalg.norm(d[:3, 3] - a[:3, 3]))

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
