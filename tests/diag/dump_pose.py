#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_dump_pose.py —— 打印 dynamic_pose/info 全部条目(模型+连杆)的位置与姿态"""
import math
import time

import gz.transport13 as gzt
from gz.msgs10.pose_v_pb2 import Pose_V


def rpy(q):
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                      1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.x * q.x))
    return roll, pitch, yaw


got = []


def cb(m):
    if got:
        return
    for p in m.pose:
        got.append(p)


node = gzt.Node()
node.subscribe(Pose_V, '/world/follow_world/dynamic_pose/info', cb)
t0 = time.time()
while not got and time.time() - t0 < 10:
    time.sleep(0.05)
for p in got:
    r, pi, y = rpy(p.orientation)
    print('%-24s pos=(%+.3f,%+.3f,%+.3f) rpy=(%+6.1f,%+6.1f,%+7.1f)°' % (
        p.name, p.position.x, p.position.y, p.position.z,
        math.degrees(r), math.degrees(pi), math.degrees(y)))
