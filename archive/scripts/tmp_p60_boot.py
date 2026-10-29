#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P60: 采样机器人从仿真启动到 40s 的姿态演变(含静止实体话题)。
"""
import math
import time

import gz.transport13 as gt
from gz.msgs10.pose_v_pb2 import Pose_V

st = {}
gn = gt.Node()


def mk(tag):
    def cb(m):
        for p in m.pose:
            if p.name in ('robot', 'caster', 'left_wheel', 'right_wheel',
                          'chassis', 'person'):
                q = p.orientation
                pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
                roll = math.atan2(2 * (q.w * q.x + q.y * q.z),
                                  1 - 2 * (q.x * q.x + q.y * q.y))
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                 1 - 2 * (q.y * q.y + q.z * q.z))
                st[p.name] = (round(p.position.x, 3), round(p.position.y, 3),
                              round(p.position.z, 3), round(math.degrees(roll), 1),
                              round(math.degrees(pitch), 1),
                              round(math.degrees(yaw), 1))
    return cb


gn.subscribe(Pose_V, '/world/follow_world/pose/info', mk('static'))
gn.subscribe(Pose_V, '/world/follow_world/dynamic_pose/info', mk('dyn'))

t0 = time.time()
last = None
while time.time() - t0 < 45:
    time.sleep(0.4)
    cur = st.get('robot')
    if cur and cur != last:
        last = cur
        print('t=%5.1f robot=%s' % (time.time() - t0, cur))
        for k in ('chassis', 'left_wheel', 'right_wheel', 'caster', 'person'):
            if k in st:
                print('        %-11s %s' % (k, st[k]))
