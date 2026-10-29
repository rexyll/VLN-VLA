#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""diag_p86_ik.py —— 复现 test_p86 [4] 的 IK 全流程并打印分解误差"""
import math
import numpy as np
import rclpy

from vla_follow import arm_kin as K, drink_poses as P, scene_layout as L
from vla_follow.drink_task import DrinkTask


def T_of(node, name):
    p = node.links.get(name)
    if p is None:
        return None
    q = p.orientation
    return K.frame(K.quat_to_mat(q.x, q.y, q.z, q.w),
                   (p.position.x, p.position.y, p.position.z))


def main():
    rclpy.init()
    node = DrinkTask()
    node.spin_for(2.5)
    node.carry.place('robot', K.frame(K.rot_z(0.0),
                                      (L.SERVE_POSE[0], L.SERVE_POSE[1], 0.0)))
    node.spin_for(0.8)
    node.sync_base_from_truth()
    print('base =', np.round(node.base, 4))

    Tk = T_of(node, 'kettle')
    print('壶真值 t =', np.round(Tk[:3, 3], 4))
    q = node.links['kettle'].orientation
    print('壶真值 q = (%.4f %.4f %.4f %.4f)' % (q.x, q.y, q.z, q.w))
    print('壶真值 R =\n', np.round(Tk[:3, :3], 3))

    Tg = P.kettle_grasp_palm(Tk)
    print('抓取掌心(世界) t =', np.round(Tg[:3, 3], 4))
    Tl = np.linalg.inv(K.base_pose(*node.base)) @ Tg
    print('抓取掌心(本体) t =', np.round(Tl[:3, 3], 4))
    print('目标-肩 %.4f m' % np.linalg.norm(Tl[:3, 3] - np.array(K.SHOULDER['R'])))

    for tag, fn in (('dls', lambda: K.ik('R', Tl, K.Q_REST)),
                    ('kdl-lma', lambda: K.ik_kdl('R', Tl, K.Q_REST, 'lma')),
                    ('ik_best', lambda: K.ik_best('R', Tl, K.Q_REST))):
        try:
            out = fn()
        except Exception as ex:
            print('%-8s 异常 %s' % (tag, ex))
            continue
        qq, ee = out[0], out[1]
        Fe = K.fk('R', qq)
        ep = np.linalg.norm(Fe[:3, 3] - Tl[:3, 3])
        er = np.linalg.norm(K.pose_err(Fe, Tl)[3:])
        print('%-8s e=%.4f  pos=%.4f m  rot=%.2f°  q=%s'
              % (tag, ee, ep, np.degrees(er), np.round(qq, 3)))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
