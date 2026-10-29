#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""probe_follow_sag.py —— 位姿跟随的"帧间下坠"逐样本测量

probe_setpose.py 的 [3] 报出 "-100 ~ -3.1mm", 首样本恰好等于整个抬升量,
说明读到的是 teleport **之前**的旧消息(订阅回调滞后)。本探针把每一帧的
时间戳/请求耗时/发送目标/读回位置全打出来, 一次看清:

  * dynamic_pose/info 的实际发布频率(决定跟随能达到的最高刷新率)
  * set_pose 单次请求的往返耗时(决定跟随循环能跑多快)
  * 每个睡眠周期内物体真正掉了多少(物理积分 vs teleport 重置的锯齿)

用法: python3 tests/diag/probe_follow_sag.py [频率Hz]
"""
import math
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
stamp = [0.0]
nmsg = [0]


def cb(msg):
    nmsg[0] += 1
    stamp[0] = time.time()
    for p in msg.pose:
        link[p.name] = (p.position.x, p.position.y, p.position.z)


def main():
    hz = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    node = gz_transport.Node()
    node.subscribe(Pose_V, TOPIC, cb)
    time.sleep(2.0)
    if MODEL not in link:
        print('没有 %s' % MODEL)
        return 2
    p0 = link[MODEL]
    print('起点 %s   累计消息 %d' % ('(%.4f %.4f %.4f)' % p0, nmsg[0]))

    # 话题频率
    n0, t0 = nmsg[0], time.time()
    time.sleep(1.0)
    print('dynamic_pose/info 频率 %.1f Hz' % ((nmsg[0] - n0) / (time.time() - t0)))

    m = Pose()
    m.name = MODEL
    m.orientation.w = 1.0
    hold_z = p0[2] + 0.10
    dt = 1.0 / hz
    print('-' * 78)
    print('%6s %8s %10s %10s %8s' % ('t(s)', 'req(ms)', 'target_z', 'read_z', 'sag(mm)'))
    t_start = time.time()
    sags = []
    while time.time() - t_start < 1.2:
        m.position.x, m.position.y, m.position.z = p0[0], p0[1], hold_z
        tr = time.time()
        node.request(SVC, m, Pose, Boolean, 1000)
        req_ms = (time.time() - tr) * 1000.0
        time.sleep(dt)
        z = link[MODEL][2]
        sag = (z - hold_z) * 1000.0
        sags.append(sag)
        print('%6.3f %8.2f %10.4f %10.4f %8.2f'
              % (time.time() - t_start, req_ms, hold_z, z, sag))

    body = sags[3:]                      # 丢掉前几帧(回调滞后)
    if body:
        print('-' * 78)
        print('稳态: n=%d  min %.1f  max %.1f  mean %.1f  RMS %.1f mm'
              % (len(body), min(body), max(body),
                 sum(body) / len(body),
                 math.sqrt(sum(s * s for s in body) / len(body))))
    # 还原
    m.position.x, m.position.y, m.position.z = p0
    node.request(SVC, m, Pose, Boolean, 2000)
    time.sleep(1.0)
    print('还原后 z = %.4f (原位 %.4f)' % (link[MODEL][2], p0[2]))
    return 0


if __name__ == '__main__':
    sys.exit(main())
