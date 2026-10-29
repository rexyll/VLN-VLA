#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P60: 测机器人姿态(俯仰/横滚)与各 link 世界高度, 判断是否车体倾斜。"""
import math
import time

import gz.transport13 as gt
from gz.msgs10.pose_v_pb2 import Pose_V

names = {}
gn = gt.Node()


def cb(m):
    for p in m.pose:
        if p.name in ('robot', 'person', 'chassis', 'left_wheel', 'right_wheel',
                      'caster'):
            q = p.orientation
            names[p.name] = (p.position.x, p.position.y, p.position.z,
                             q.x, q.y, q.z, q.w)


gn.subscribe(Pose_V, '/world/follow_world/dynamic_pose/info', cb)
t = time.time()
while time.time() - t < 3:
    time.sleep(0.1)

for k in sorted(names):
    x, y, z, qx, qy, qz, qw = names[k]
    # ZYX 欧拉角
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    print('%-11s pos=(%6.2f,%6.2f,%6.3f)  roll=%+6.1f° pitch=%+6.1f° yaw=%+7.1f°'
          % (k, x, y, z, math.degrees(roll), math.degrees(pitch),
             math.degrees(yaw)))
