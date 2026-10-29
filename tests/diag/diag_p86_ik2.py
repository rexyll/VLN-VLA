#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""diag_p86_ik2.py —— 逐行复刻 test_p86 [1]~[4], 找出 IK 输入差异"""
import math
import numpy as np
import rclpy

from vla_follow import arm_kin as K, drink_poses as P, scene_layout as L
from vla_follow.drink_task import DrinkTask

SIDE, MODEL = 'R', 'kettle'


def T_of(node, name):
    p = node.links.get(name)
    if p is None:
        return None
    q = p.orientation
    return K.frame(K.quat_to_mat(q.x, q.y, q.z, q.w),
                   (p.position.x, p.position.y, p.position.z))


def dump(Tk, node, tag):
    Tg = P.kettle_grasp_palm(Tk)
    q_g, e_g, how = K.ik_world(SIDE, Tg, node.base, K.Q_REST)
    marg = math.degrees(float(np.min(np.minimum(q_g - K.LIM_LO,
                                              K.LIM_HI - q_g))))
    print('%s 壶t=%s q=(%.4f,%.4f,%.4f,%.4f)'
          % (tag, np.round(Tk[:3, 3], 4),
             node.links[MODEL].orientation.x, node.links[MODEL].orientation.y,
             node.links[MODEL].orientation.z, node.links[MODEL].orientation.w))
    print('   base=%s  掌心目标=%s' % (np.round(node.base, 4), np.round(Tg[:3, 3], 4)))
    print('   ik_world: how=%s e=%.4f (%.1f "mm") marg=%.0f°  q=%s'
          % (how, e_g, e_g * 1000, marg, np.round(q_g, 3)))


def main():
    rclpy.init()
    node = DrinkTask()
    node.spin_for(2.5)
    dump(T_of(node, MODEL), node, '[A] 上电后')

    node.carry.place('robot', K.frame(K.rot_z(0.0),
                                      (L.SERVE_POSE[0], L.SERVE_POSE[1], 0.0)))
    node.spin_for(0.8)
    node.sync_base_from_truth()
    dump(T_of(node, MODEL), node, '[B] 到服务位')

    node.home()
    dump(T_of(node, MODEL), node, '[C] home 之后')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
