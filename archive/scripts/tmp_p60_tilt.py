#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P60 物理实验: 杀掉 follower 后直接驱动 /model/robot/cmd_vel,
观察 yaw(是否转)与 pitch(是否被掀翻)。
用法: python3 tmp_p60_tilt.py [w] [dur]
"""
import math
import subprocess
import sys
import time

import gz.transport13 as gt
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist as GzTwist

W = float(sys.argv[1]) if len(sys.argv) > 1 else 1.5
DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0

subprocess.run(['pkill', '-f', 'vla_follo[w]/lib'], capture_output=True)
time.sleep(1.0)

st = {}
gn = gt.Node()


def cb(m):
    for p in m.pose:
        if p.name == 'robot':
            q = p.orientation
            roll = math.atan2(2 * (q.w * q.x + q.y * q.z),
                              1 - 2 * (q.x * q.x + q.y * q.y))
            pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            st['v'] = (p.position.x, p.position.y, p.position.z,
                       math.degrees(roll), math.degrees(pitch),
                       math.degrees(yaw))


gn.subscribe(Pose_V, '/world/follow_world/dynamic_pose/info', cb)

pub = gn.advertise('/model/robot/cmd_vel', GzTwist)
zero = GzTwist()
tw = GzTwist()
tw.angular.z = W

for _ in range(10):
    pub.publish(zero)
    time.sleep(0.05)
time.sleep(1.5)
v = st.get('v')
print('起步: pos=(%.2f,%.2f,%.3f) roll=%+.1f pitch=%+.1f yaw=%+.1f'
      % (v[0], v[1], v[2], v[3], v[4], v[5]))

# P60: 先把车摆正 —— 从高处落下, 看 pitch 能否回到 0; 再转, 看是否会
# 重新被掀翻。这能把"倾斜是转向造成的"与"倾斜是别的事造成的"分开。
if len(sys.argv) > 3 and sys.argv[3] == 'reset':
    req = ('name: "robot", position: {x: -3.0, y: 0.0, z: 0.35}, '
           'orientation: {z: 0.0, w: 1.0}')
    for _ in range(3):
        subprocess.run(['gz', 'service', '-s', '/world/follow_world/set_pose',
                        '--reqtype', 'gz.msgs.Pose',
                        '--reptype', 'gz.msgs.Boolean', '--timeout', '2000',
                        '--req', req], capture_output=True, text=True)
        time.sleep(0.3)
    time.sleep(2.5)
    v = st.get('v')
    print('摆正后: pos=(%.2f,%.2f,%.3f) roll=%+.1f pitch=%+.1f yaw=%+.1f'
          % (v[0], v[1], v[2], v[3], v[4], v[5]))

for _ in range(5):
    pub.publish(tw)
    time.sleep(0.05)

t0 = time.time()
while time.time() - t0 < DUR:
    time.sleep(0.4)
    pub.publish(tw)
    v = st.get('v')
    if v:
        print(' t=%4.1f pos=(%6.2f,%6.2f,%6.3f) roll=%+6.1f pitch=%+6.1f yaw=%+7.1f'
              % (time.time() - t0, v[0], v[1], v[2], v[3], v[4], v[5]))

for _ in range(15):
    pub.publish(zero)
    time.sleep(0.05)
time.sleep(1.0)
v = st.get('v')
print('停后: pos=(%.2f,%.2f,%.3f) roll=%+.1f pitch=%+.1f yaw=%+.1f'
      % (v[0], v[1], v[2], v[3], v[4], v[5]))
