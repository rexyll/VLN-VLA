#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""diag_release_drift.py —— 松手后物体是否带残余速度继续滑"""
import sys
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


def q_meas(node):
    names = ['%s_%s_joint' % (SIDE, j[0]) for j in K.JOINTS]
    return np.array([node.js[n] for n in names], float)


def move_to(node, T, tol_mm=6.0, passes=4):
    for it in range(passes):
        q, _e, _h = K.ik_world(SIDE, T, node.base, q_meas(node))
        node.arm[SIDE].move(q, timeout=10.0, tol_deg=(0.6 if it else None))
        res = np.linalg.norm(K.pose_err(
            K.fk_world(SIDE, q_meas(node), node.base), T)[:3]) * 1000
        if res < tol_mm:
            break
    return res


rclpy.init()
node = DrinkTask()
while len(node.js) < 34:
    node.spin_for(0.2)
node.carry.place('robot', K.frame(K.rot_z(0.0),
                                 (L.SERVE_POSE[0], L.SERVE_POSE[1], 0.0)))
node.spin_for(0.8)
node.sync_base_from_truth()
node.home()

Tk0 = T_of(node, MODEL)
Tg = P.kettle_grasp_palm(Tk0)
print('抓取位残差 %.1f mm' % move_to(node, Tg))
print('抓握前壶 t =', np.round(Tk0[:3, 3], 4))
node.carry.grab(MODEL, SIDE)
node.spin_for(0.4)
Tk_a = T_of(node, MODEL)
print('抓握后壶 t =', np.round(Tk_a[:3, 3], 4), ' 位移 %.1f mm'
      % (np.linalg.norm(Tk_a[:3, 3] - Tk0[:3, 3]) * 1000))

T_lift = K.frame(Tg[:3, :3], Tg[:3, 3] + np.array([0.0, 0.0, 0.12]))
print('抬升位残差 %.1f mm' % move_to(node, T_lift))
print('抬起后壶 z = %.4f (原 %.4f)' % (T_of(node, MODEL)[2, 3], Tk0[2, 3]))

print('回抓取位残差 %.1f mm' % move_to(node, Tg))
node.spin_for(0.3)
Tk_b = T_of(node, MODEL)
print('回位后壶 t =', np.round(Tk_b[:3, 3], 4), ' 偏差 %.1f mm'
      % (np.linalg.norm(Tk_b[:3, 3] - Tk0[:3, 3]) * 1000))

node.carry.release(MODEL)
print('--- 松手后逐拍采样 ---')
for i in range(10):
    node.spin_for(0.2)
    t = T_of(node, MODEL)[:3, 3]
    print('  t+%.1fs  t=%s  偏离原位 %.1f mm'
          % (0.2 * (i + 1), np.round(t, 4),
             np.linalg.norm(t - Tk0[:3, 3]) * 1000))

node.destroy_node()
rclpy.shutdown()
