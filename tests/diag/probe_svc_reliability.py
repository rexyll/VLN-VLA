#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""probe_svc_reliability.py —— set_pose 服务调用的可靠性

前面几个探针里偶发"请求整 1000ms 超时"(probe_setpose_detail 的 call2、
probe_follow_sag 的最后一次), 对"每帧跟随"是致命的: 丢掉一帧物体就掉 1 米。

本探针比较两种姿势, 各打 100 次, 统计失败率与耗时分布:
  A) 同一个 gz Node 既订阅 dynamic_pose/info 又发服务请求
     —— 订阅回调(56Hz)与服务应答可能抢同一个 transport 线程
  B) **专用 Node 只发请求, 不订阅任何话题**
  C) B 姿势 + 失败立即重试一次

判据: 失败率 0 且 p99 耗时 < 20ms 才算可用于 20Hz 跟随。

用法: python3 tests/diag/probe_svc_reliability.py [次数]
"""
import statistics
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

N = int(sys.argv[1]) if len(sys.argv) > 1 else 100
pos = {}


def cb(msg):
    for p in msg.pose:
        pos[p.name] = (p.position.x, p.position.y, p.position.z)


def mk(x, y, z):
    m = Pose()
    m.name = MODEL
    m.position.x, m.position.y, m.position.z = x, y, z
    m.orientation.w = 1.0
    return m


def run(tag, node, target, n=N):
    lat = []
    fails = 0
    for _ in range(n):
        t = time.time()
        ret = node.request(SVC, target, Pose, Boolean, 1000)
        dt = (time.time() - t) * 1000.0
        ok = (isinstance(ret, tuple) and bool(ret[0])
              and bool(getattr(ret[1], 'data', False)))
        if not ok:
            # 重试一次
            t2 = time.time()
            ret = node.request(SVC, target, Pose, Boolean, 1000)
            ok = (isinstance(ret, tuple) and bool(ret[0])
                  and bool(getattr(ret[1], 'data', False)))
            if ok:
                dt = (time.time() - t2) * 1000.0
        if not ok:
            fails += 1
        lat.append(dt)
        time.sleep(0.02)
    lat_sorted = sorted(lat)
    p50 = statistics.median(lat)
    p99 = lat_sorted[min(len(lat_sorted) - 1, int(0.99 * len(lat_sorted)))]
    print('%-6s 失败 %2d/%d   p50 %6.2fms  p99 %7.2fms  max %7.2fms'
          % (tag, fails, n, p50, p99, lat_sorted[-1]))
    return fails


def main():
    # A: 订阅 + 请求 同一节点
    na = gz_transport.Node()
    na.subscribe(Pose_V, TOPIC, cb)
    time.sleep(2.0)
    if MODEL not in pos:
        print('没有 %s' % MODEL)
        return 2
    p0 = pos[MODEL]
    print('kettle 静止在 %s' % (p0,))
    print('-' * 74)
    fa = run('A 混合', na, mk(*p0))

    # B/C: 专用节点
    nb = gz_transport.Node()
    time.sleep(0.5)
    fb = run('B 专用', nb, mk(*p0))
    print('-' * 74)
    print('A 失败 %d  B 失败 %d  ->  %s'
          % (fa, fb, '专用节点更稳' if fb < fa else
             ('两者相当' if fb == fa else '混合节点更稳')))
    return 0 if (fa + fb) == 0 else 2


if __name__ == '__main__':
    sys.exit(main())
