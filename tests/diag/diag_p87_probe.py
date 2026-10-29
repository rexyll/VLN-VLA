#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""diag_p87_probe.py —— 单独验证 water_joint 行程与相机订阅"""
import time
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

from vla_follow.drink_task import DrinkTask


def z_of(node, name):
    p = node.links.get(name)
    return None if p is None else (p.position.x, p.position.y, p.position.z)


rclpy.init()
node = DrinkTask()
node.spin_for(3.0)

img_n = {'n': 0}
def cb(_m):
    img_n['n'] += 1

node.create_subscription(Image, '/drink_cam/image', cb, 5)
best = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                  history=HistoryPolicy.KEEP_LAST)
img_n2 = {'n': 0}
def cb2(_m):
    img_n2['n'] += 1
node.create_subscription(Image, '/drink_cam/image', cb2, best)

print('water 初始 =', z_of(node, 'water'), ' jet 初始 =', z_of(node, 'jet'))
node.spin_for(6.0)
print('相机: 默认QoS 收到 %d 帧, BEST_EFFORT 收到 %d 帧'
      % (img_n['n'], img_n2['n']))

pub = node.create_publisher(Float64, '/drink/cmd/water_joint', 10)
for i in range(60):
    pub.publish(Float64(data=0.0156))
    node.spin_for(0.05)
print('直接发 60 帧后 water =', z_of(node, 'water'))
node.bus.set(['water_joint'], [0.024])
node.spin_for(2.0)
print('走 bus 到 0.024 后 water =', z_of(node, 'water'))

node.destroy_node()
rclpy.shutdown()
