#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""diag_water3.py —— 分步定位"水位关节什么时候失效" """
import rclpy
from vla_follow import arm_kin as K
from vla_follow import drink_motion as M
from vla_follow import drink_poses as P
from vla_follow import scene_layout as L
from vla_follow.drink_task import DrinkTask

rclpy.init()
node = DrinkTask()
while len(node.js) < 34:
    node.spin_for(0.2)
node.carry.place('robot', K.frame(K.rot_z(0.0),
                                 (L.SERVE_POSE[0], L.SERVE_POSE[1], 0.0)))
node.spin_for(0.8)
node.sync_base_from_truth()


def wtest(tag, v):
    node.bus.set(['water_joint'], [v])
    node.spin_for(1.6)
    w = node.links.get('water')
    print('%-22s cmd=%.3f -> water.z=%.4f  (q=%.4f)'
          % (tag, v, w.position.z, w.position.z + 0.05))


wtest('① 初始(未钉未举)', 0.006)
wtest('② 再确认', 0.012)
node.bus.set(['water_joint'], [0.0])
node.spin_for(0.8)

node.home()
wtest('③ home 之后', 0.006)

M.grasp(node, 'L', 'cup', lambda T: P.cup_grasp_palm(T, 'L'), K.GRIP_CUP)
wtest('④ 抓杯之后', 0.012)

M.place_held(node, 'L', 'cup', P.cup_for_pour(), tol_mm=4.0)
wtest('⑤ 举杯之后', 0.018)

M.grasp(node, 'R', 'kettle', lambda T: P.kettle_grasp_palm(T), K.GRIP_KETTLE)
wtest('⑥ 抓壶之后', 0.024)

Tk_pour, _tip = P.kettle_for_pour(M.object_pose(node, 'cup'))
M.place_held(node, 'R', 'kettle', Tk_pour, tol_mm=5.0)
wtest('⑦ 壶到倒水位之后', 0.012)

node.bus.set(['jet_joint'], [0.16])
node.spin_for(1.6)
j = node.links.get('jet')
print('jet cmd=0.16 -> 位移 %.4f' % (
    (j.position.x ** 2 + j.position.y ** 2 + j.position.z ** 2) ** 0.5
    - (0.02229 ** 2 + 0.04252 ** 2) ** 0.5))

node.destroy_node()
rclpy.shutdown()
