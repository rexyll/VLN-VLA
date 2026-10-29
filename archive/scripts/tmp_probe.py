#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_probe.py —— 3 秒内能否收到机器人真值位姿(启动自检)"""
import time

import gz.transport13 as gzt
from gz.msgs10.pose_v_pb2 import Pose_V

cnt = [0]


def cb(m):
    for p in m.pose:
        if p.name == 'robot':
            cnt[0] += 1
            return


n = gzt.Node()
n.subscribe(Pose_V, '/world/follow_world/dynamic_pose/info', cb)
t = time.time()
while time.time() - t < 3.0 and cnt[0] == 0:
    time.sleep(0.05)
if cnt[0]:
    print('POSE_OK 3s内收到 %d 帧' % cnt[0])
else:
    print('POSE_FAIL 3s 内未收到 /world/follow_world/dynamic_pose/info')
