#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""probe_sag_trace.py —— 5ms 高频采样: set_pose 之后物体到底怎么走

前面几个探针的读数互相矛盾(有时 0.0mm 精确, 有时一路下坠到 -100mm), 全部
指向同一个嫌疑: **读回的位置与 teleport 之间存在时序错位**。
本探针每 5ms 采一次真值, 持续 300ms, 画出"teleport 后的实际轨迹":

  * 若第 1 帧就在目标高度   → 复位生效, 之前的下坠是采样滞后
  * 若从 0 处缓慢线性下落    → 复位生效但速度没清零(需要每帧都发)
  * 若直接停在桌面高度       → 复位根本没生效

用法: python3 tests/diag/probe_sag_trace.py
"""
import sys
import time

import gz.transport13 as gz_transport
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.boolean_pb2 import Boolean

WORLD = 'follow_world'
TOPIC = '/world/%s/dynamic_pose/info' % WORLD
SVC = '/world/%s/set_pose' % WORLD
MODEL = 'kettle'

link = {}
tmsg = [0.0]


def cb(msg):
    tmsg[0] = time.time()
    for p in msg.pose:
        link[p.name] = (p.position.x, p.position.y, p.position.z)


def main():
    node = gz_transport.Node()
    node.subscribe(Pose_V, TOPIC, cb)
    time.sleep(2.0)
    p0 = link[MODEL]
    m = Pose()
    m.name = MODEL
    m.orientation.w = 1.0
    m.position.x, m.position.y, m.position.z = p0[0], p0[1], p0[2] + 0.10

    print('桌面 z=%.4f  目标 z=%.4f' % (p0[2], p0[2] + 0.10))
    print('-' * 60)
    print('%8s %10s %10s %8s' % ('t(ms)', 'read_z', 'msg_age', 'sag(mm)'))
    t0 = time.time()
    node.request(SVC, m, Pose, Boolean, 1000)
    print('   (request 往返 %.1f ms)' % ((time.time() - t0) * 1000.0))
    t0 = time.time()                 # 计时从 teleport **完成之后**开始
    while (time.time() - t0) < 0.30:
        now = time.time()
        z = link[MODEL][2]
        print('%8.1f %10.4f %10.1f %8.1f'
              % ((now - t0) * 1000.0, z, (now - tmsg[0]) * 1000.0,
                 (z - p0[2] - 0.10) * 1000.0))
        time.sleep(0.005)

    # 复原
    m.position.z = p0[2]
    node.request(SVC, m, Pose, Boolean, 1000)
    time.sleep(0.8)
    print('还原 z=%.4f' % link[MODEL][2])
    return 0


if __name__ == '__main__':
    sys.exit(main())
