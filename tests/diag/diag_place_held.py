#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""diag_place_held.py —— 送位闭环每轮残差 + 关节级误差明细"""
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
    return np.array([node.js['%s_%s_joint' % (SIDE, j[0])] for j in K.JOINTS],
                    float)


def move_to(node, T, tol_mm=2.0, passes=5, tag=''):
    sched = (None, 0.6, 0.25, 0.25, 0.25)
    for it in range(passes):
        q, _e, _h = K.ik_world(SIDE, T, node.base, q_meas(node))
        ok, err = node.arm[SIDE].move(q, timeout=10.0, tol_deg=sched[min(it, 4)])
        res = np.linalg.norm(K.pose_err(
            K.fk_world(SIDE, q_meas(node), node.base), T)[:3]) * 1000
        print('   %s pass%d ok=%s 位置残差 %.1f mm 关节残差(°)=%s'
              % (tag, it, ok, res,
                 ' '.join('%+5.2f' % v for v in np.degrees(err))))
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
move_to(node, Tg, 5.0, tag='抓取')
node.carry.grab(MODEL, SIDE)
T_lift = K.frame(Tg[:3, :3], Tg[:3, 3] + np.array([0.0, 0.0, 0.12]))
move_to(node, T_lift, 6.0, tag='抬升')
print('抬起后壶 z=%.4f' % T_of(node, MODEL)[2, 3])

print('--- 送位闭环 ---')
for i in range(6):
    Tk = T_of(node, MODEL)
    res = np.linalg.norm(Tk[:3, 3] - Tk0[:3, 3]) * 1000
    print('轮%d 物体残差 %.1f mm' % (i, res))
    if res < 2.0:
        break
    Tp = node.carry.palm_world(SIDE)
    Tp_goal = (Tk0 @ np.linalg.inv(Tk)) @ Tp
    move_to(node, Tp_goal, 2.0, tag='  修正')

def res_now():
    return np.linalg.norm(T_of(node, MODEL)[:3, 3] - Tk0[:3, 3]) * 1000


print('送位后       %.1f mm' % res_now())
node.hand[SIDE].move(K.grip_values(SIDE, K.GRIP_OPEN), timeout=5.0)
node.spin_for(0.2)
print('张手后       %.1f mm' % res_now())
# 张手无接触后再对一次位
for i in range(4):
    Tk = T_of(node, MODEL)
    if np.linalg.norm(Tk[:3, 3] - Tk0[:3, 3]) * 1000 < 2.0:
        break
    Tp = node.carry.palm_world(SIDE)
    move_to(node, (Tk0 @ np.linalg.inv(Tk)) @ Tp, 2.0, tag='  再对')
node.spin_for(0.2)
print('再对位后     %.1f mm' % res_now())
node.carry.release(MODEL)
for i in range(4):
    node.spin_for(0.15)
    print('松钉后 t+%.2fs %.1f mm' % (0.15 * (i + 1), res_now()))
move_to(node, T_pre, 8.0, tag='撤退')
node.spin_for(0.5)
print('撤退后       %.1f mm' % res_now())

node.destroy_node()
rclpy.shutdown()
