#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P60 决定性实验: 人从背后推静止的机器人, 机器人是否被推走/推倒。
1) 机器人(0,0,yaw0) 单独静置 8s  -> 记 pitch/pos
2) 人放到(2,0) 朝 -x, 用 cmd_vel 让它走向机器人 -> 记 pitch/pos
用法: python3 tmp_p60_push.py
"""
import math
import subprocess
import time

import gz.transport13 as gt
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist as GzTwist

st = {}
gn = gt.Node()


def cb(m):
    for p in m.pose:
        if p.name in ('robot', 'person'):
            q = p.orientation
            pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
            roll = math.atan2(2 * (q.w * q.x + q.y * q.z),
                              1 - 2 * (q.x * q.x + q.y * q.y))
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            st[p.name] = (round(p.position.x, 2), round(p.position.y, 2),
                          round(p.position.z, 3), round(math.degrees(roll), 1),
                          round(math.degrees(pitch), 1),
                          round(math.degrees(yaw), 1))


gn.subscribe(Pose_V, '/world/follow_world/dynamic_pose/info', cb)


def set_pose(name, x, y, z, yaw=0.0):
    req = ('name: "%s", position: {x: %s, y: %s, z: %s}, '
           'orientation: {z: %s, w: %s}'
           % (name, x, y, z, math.sin(yaw / 2), math.cos(yaw / 2)))
    for _ in range(3):
        r = subprocess.run(
            ['gz', 'service', '-s', '/world/follow_world/set_pose',
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '2000', '--req', req], capture_output=True, text=True)
        if 'true' in (r.stdout or '').lower():
            return True
        time.sleep(0.2)
    return False


# 停掉会动的东西
subprocess.run(['pkill', '-f', 'person_wal[k]'], capture_output=True)
subprocess.run(['pkill', '-f', 'vla_follo[w]/lib'], capture_output=True)
time.sleep(1.0)

pub = gn.advertise('/model/person/cmd_vel', GzTwist)
zero = GzTwist()
for _ in range(10):
    pub.publish(zero)
    time.sleep(0.05)

set_pose('robot', 0.0, 0.0, 0.15, 0.0)
set_pose('person', 3.0, 0.0, 0.02, 0.0)
time.sleep(2.0)

print('=== 阶段1: 人远离, 机器人静置 8s ===')
t0 = time.time()
while time.time() - t0 < 8:
    time.sleep(0.5)
    r = st.get('robot')
    if r:
        print(' t=%4.1f robot=(%6.2f,%6.2f,%6.3f) roll=%+5.1f pitch=%+6.1f yaw=%+6.1f'
              % (time.time() - t0, r[0], r[1], r[2], r[3], r[4], r[5]))

print('=== 阶段2: 人从 +x 推向机器人(前进 -x) 12s ===')
tw = GzTwist()
tw.linear.x = -0.35   # 人的车头朝 +x, 倒车走向 -x 的机器人
t0 = time.time()
while time.time() - t0 < 12:
    pub.publish(tw)
    time.sleep(0.1)
    if int((time.time() - t0) * 2) % 2 == 0:
        r = st.get('robot')
        p = st.get('person')
        if r:
            print(' t=%4.1f robot=(%6.2f,%6.2f,%6.3f) roll=%+5.1f pitch=%+6.1f '
                  'yaw=%+6.1f | person=(%6.2f,%6.2f)'
                  % (time.time() - t0, r[0], r[1], r[2], r[3], r[4], r[5],
                     p[0] if p else -1, p[1] if p else -1))
for _ in range(10):
    pub.publish(zero)
    time.sleep(0.05)
