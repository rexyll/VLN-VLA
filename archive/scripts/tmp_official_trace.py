#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_official_trace.py —— 官方示例: 打印 vehicle_blue 位姿时序列 (v w sec)"""
import math
import sys
import time

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist
import gz.transport13 as gzt


def rpy(q):
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                      1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.x * q.x))
    return roll, pitch, yaw


node = gzt.Node()
pub = node.advertise('/model/vehicle_blue/cmd_vel', Twist)
st = {'p': None}
stamp = {'t': None}


def cb(m):
    st['t'] = m.header.stamp.sec + m.header.stamp.nsec * 1e-9
    for p in m.pose:
        if p.name == 'vehicle_blue':
            st['p'] = (p.position.x, p.position.y, p.position.z,
                       rpy(p.orientation))


node.subscribe(Pose_V, '/world/diff_drive/dynamic_pose/info', cb)
t = time.time()
while st['p'] is None and time.time() - t < 15:
    time.sleep(0.1)
print('settle 5s ...')
time.sleep(5)
p = st['p']
print('settled pos=(%.2f,%.2f,%.2f) rpy=(%.1f,%.1f,%.1f)' % (
    p[0], p[1], p[2], math.degrees(p[3][0]), math.degrees(p[3][1]),
    math.degrees(p[3][2])))
v, w, sec = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
t = time.time()
nxt = 0.0
while time.time() - t < sec:
    m = Twist()
    m.linear.x = v
    m.angular.z = w
    pub.publish(m)
    el = time.time() - t
    if el >= nxt:
        nxt += 0.5
        p = st['p']
        print('t=%.1f pos=(%+.2f,%+.2f) yaw=%+.0f° pitch=%+.1f°' % (
            el, p[0], p[1], math.degrees(p[3][2]), math.degrees(p[3][1])))
    time.sleep(0.02)
pub.publish(Twist())
print('sim time elapsed %.2fs (wall %.2f)' % (st['t'], time.time() - t))
