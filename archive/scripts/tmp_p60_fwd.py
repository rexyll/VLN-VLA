#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P60: 直线前进是否把车体翘起来(fixed caster 锁死 -> 拖拽 -> 抬头)。
用法: python3 tmp_p60_fwd.py [v] [dur]
"""
import math
import subprocess
import sys
import time

import gz.transport13 as gt
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist as GzTwist

V = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0

subprocess.run(['pkill', '-f', 'person_wal[k]'], capture_output=True)
subprocess.run(['pkill', '-f', 'vla_follo[w]/lib'], capture_output=True)
time.sleep(1.0)

st = {}
gn = gt.Node()


def cb(m):
    for p in m.pose:
        if p.name == 'robot':
            q = p.orientation
            pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            st['v'] = (p.position.x, p.position.y, p.position.z,
                       math.degrees(pitch), math.degrees(yaw))


gn.subscribe(Pose_V, '/world/follow_world/dynamic_pose/info', cb)
pub = gn.advertise('/model/robot/cmd_vel', GzTwist)
zero = GzTwist()
tw = GzTwist()
tw.linear.x = V

for _ in range(10):
    pub.publish(zero)
    time.sleep(0.05)
time.sleep(1.0)
v = st.get('v')
print('起步 pos=(%.2f,%.2f,%.3f) pitch=%+.1f yaw=%+.1f' % v)

t0 = time.time()
while time.time() - t0 < DUR:
    pub.publish(tw)
    time.sleep(0.15)
    v = st.get('v')
    if v:
        print(' t=%4.1f pos=(%6.2f,%6.2f,%6.3f) pitch=%+6.1f yaw=%+7.1f'
              % (time.time() - t0, v[0], v[1], v[2], v[3], v[4]))
for _ in range(10):
    pub.publish(zero)
    time.sleep(0.05)
