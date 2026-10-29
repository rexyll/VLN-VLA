#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_official_test.py —— 在官方 diff_drive 示例世界里测试差速响应
(先启动: gz sim -r /usr/share/gz/gz-sim8/worlds/diff_drive_skid.sdf)
"""
import math
import sys
import time

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist
import gz.transport13 as gzt

WORLD = '/world/diff_drive/dynamic_pose/info'
CMD = '/model/vehicle_blue/cmd_vel'


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.x * q.x))


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


node = gzt.Node()
pub = node.advertise(CMD, Twist)
state = {'p': None, 'acc': 0.0, 'accv': 0.0, 'prev': None, 'n': 0,
         't0': None, 't1': None}


def cb(m):
    st = m.header.stamp.sec + m.header.stamp.nsec * 1e-9
    for p in m.pose:
        if p.name != 'vehicle_blue':
            continue
        y = yaw_of(p.orientation)
        x, yy = p.position.x, p.position.y
        if state['prev'] is not None:
            ds = st - state['prev'][3]
            if ds > 1e-6:
                state['acc'] += wrap(y - state['prev'][2])
                state['accv'] += ((x - state['prev'][0]) * math.cos(state['prev'][2])
                                  + (yy - state['prev'][1]) * math.sin(state['prev'][2]))
                state['n'] += 1
        state['prev'] = (x, yy, y, st)
        if state['t0'] is None:
            state['t0'] = st
        state['t1'] = st


node.subscribe(Pose_V, WORLD, cb)
t = time.time()
while state['prev'] is None and time.time() - t < 15:
    time.sleep(0.1)
if state['prev'] is None:
    print('未收到官方示例位姿, 检查世界名')
    sys.exit(1)
print('已连接官方示例, 初始位姿', state['prev'][:3])

v, w, sec = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
state['acc'] = state['accv'] = 0.0
state['n'] = 0
state['t0'] = None
t = time.time()
while time.time() - t < sec:
    m = Twist()
    m.linear.x = v
    m.angular.z = w
    pub.publish(m)
    time.sleep(0.02)
pub.publish(Twist())
time.sleep(0.8)
ds = (state['t1'] - state['t0']) or 1.0
print('官方示例 cmd v=%+.2f w=%+.2f → 实测 v=%+.3f m/s (比 %+.2f) '
      'w=%+.3f rad/s (比 %+.2f)' % (
          v, w, state['accv'] / ds,
          (state['accv'] / ds) / v if abs(v) > 1e-6 else 0,
          state['acc'] / ds,
          (state['acc'] / ds) / w if abs(w) > 1e-6 else 0))
