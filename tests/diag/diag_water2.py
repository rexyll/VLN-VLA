#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""diag_water2.py —— 用 /drink/joint_state 直接看 jet/water 关节角"""
import rclpy
from vla_follow.drink_task import DrinkTask

rclpy.init()
node = DrinkTask()
node.spin_for(3.0)
keys = sorted(node.js.keys())
print('joint_state 共 %d 路' % len(keys))
print('含 jet/water:', [k for k in keys if 'jet' in k or 'water' in k or 'person' in k])
print('初始化 jet=%.4f water=%.4f'
      % (node.js.get('jet_joint', float('nan')),
         node.js.get('water_joint', float('nan'))))

for v in (0.024, 0.012, 0.0):
    node.bus.set(['water_joint'], [v])
    node.spin_for(1.5)
    print('命令 water=%.3f -> 实测 %.4f   (link z=%s)'
          % (v, node.js.get('water_joint', float('nan')),
             None if 'water' not in node.links
             else round(node.links['water'].position.z, 5)))

node.destroy_node()
rclpy.shutdown()
