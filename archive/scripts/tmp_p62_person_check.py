#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P62b: 验证 person 模型的运动方向约定。

robot 与 person 是两个独立的 DiffDrive 模型, P62 对两者都交换了
left_joint/right_joint 绑定, 所以 person 也必须复验一次:
  - person_walk 约定: msg.linear.x = v (v>0) 表示沿自身朝向 pyaw 前进
                        msg.angular.z = w (w>0) 表示逆时针(左转)
  - 期望: 人前进方向·自身朝向 = +1 ; 人左转时 pyaw 增大

直接驱动 /model/person/cmd_vel, 用 /world/follow_world/dynamic_pose/info
里的 person 位姿判定。
"""
import math
import sys
import time

import gz.transport13 as gt
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist as GzTwist

TAG = sys.argv[1] if len(sys.argv) > 1 else 'P62'
st = {}
gn = gt.Node()


def cb(m):
    for p in m.pose:
        if p.name == 'person':
            q = p.orientation
            st['person'] = (p.position.x, p.position.y,
                            math.degrees(math.atan2(
                                2 * (q.w * q.z + q.x * q.y),
                                1 - 2 * (q.y * q.y + q.z * q.z))))


gn.subscribe(Pose_V, '/world/follow_world/dynamic_pose/info', cb)
pub = gn.advertise('/model/person/cmd_vel', GzTwist)
zero = GzTwist()


def zero_out(n=6):
    for _ in range(n):
        pub.publish(zero)
        time.sleep(0.05)


def drive(v, w, secs, tag):
    tw = GzTwist()
    tw.linear.x = v
    tw.angular.z = w
    p0 = st.get('person')
    t0 = time.time()
    while time.time() - t0 < secs:
        pub.publish(tw)
        time.sleep(0.1)
    zero_out(4)
    time.sleep(0.6)
    p = st.get('person')
    if not (p and p0):
        print('  [%s] 无数据' % tag)
        return None
    dx, dy = p[0] - p0[0], p[1] - p0[1]
    n = math.hypot(dx, dy)
    fwd = (dx * math.cos(p0[2]) + dy * math.sin(p0[2])) / n if n > 1e-6 else 0.0
    dyaw = p[2] - p0[2]
    print('  %-12s Δpos=(%+.3f,%+.3f) 前向点积=%+.3f  Δyaw=%+7.1f'
          % (tag, dx, dy, fwd, dyaw), flush=True)
    return fwd, dyaw


print('===== person 方向判定 [%s] =====' % TAG)
time.sleep(2.0)
zero_out(8)
time.sleep(1.0)
if 'person' in st:
    print('  初始 person pos=(%+.3f,%+.3f) yaw=%+.1f'
          % (st['person'][0], st['person'][1], st['person'][2]))
r1 = drive(0.4, 0.0, 3.0, 'cmd.x=+0.4')
r2 = drive(0.0, 1.2, 3.0, 'cmd.z=+1.2')
print()
if r1 and r2:
    print('VERDICT [%s] 人前进=%s  人左转(w>0)=%s'
          % (TAG, 'OK' if r1[0] > 0.8 else 'BAD',
             'OK' if r2[1] > 30 else 'BAD'))
