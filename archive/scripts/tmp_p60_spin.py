#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P60 诊断: gz 侧 /model/robot/cmd_vel 是否真的驱动机器人旋转,
以及旋转时 /perception/proximity 的扇区值是否跟随更新。"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import gz.transport13 as gt
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist as GzTwist

rclpy.init()
node = Node('p60_spin')
prox = {}
yaw = {'v': None}


def pcb(m):
    try:
        prox.update(json.loads(m.data))
    except Exception:
        pass


node.create_subscription(String, '/perception/proximity', pcb, 10)

gn = gt.Node()


def pose_cb(m):
    for p in m.pose:
        if p.name == 'robot':
            q = p.orientation
            yaw['v'] = math.degrees(math.atan2(
                2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


gn.subscribe(Pose_V, '/world/follow_world/dynamic_pose/info', pose_cb)

pub = gn.advertise('/model/robot/cmd_vel', GzTwist)
zero = GzTwist()
for _ in range(10):
    pub.publish(zero)
    time.sleep(0.05)

print('=== 静止 ===')
t = time.time()
while time.time() - t < 2:
    rclpy.spin_once(node, timeout_sec=0.1)
print(' yaw=%.1f  front=%.2f fl=%.2f fr=%.2f'
      % (yaw['v'] or -999, prox.get('front', -1),
         prox.get('fl', -1), prox.get('fr', -1)))

tw = GzTwist()
tw.angular.z = 1.5
for _ in range(5):
    pub.publish(tw)
    time.sleep(0.05)

print('=== w=1.5 旋转 6s ===')
t = time.time()
n = 0
while time.time() - t < 6:
    rclpy.spin_once(node, timeout_sec=0.1)
    n += 1
    if n % 20 == 0:
        print(' t=%.1f yaw=%.1f age=%.2f front=%.2f fl=%.2f'
              % (time.time() - t, yaw['v'] or -999, prox.get('age', -1),
                 prox.get('front', -1), prox.get('fl', -1)))
        prox.clear()

for _ in range(10):
    pub.publish(zero)
    time.sleep(0.05)
print('=== 停 ===')
t = time.time()
while time.time() - t < 2:
    rclpy.spin_once(node, timeout_sec=0.1)
print(' yaw=%.1f  front=%.2f' % (yaw['v'] or -999, prox.get('front', -1)))
