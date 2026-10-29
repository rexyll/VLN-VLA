#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""probe_svc_retry.py —— 短超时 + 同帧重试能否把 set_pose 变成可靠的 20Hz 通道

probe_svc_reliability 测出: 无论是否与订阅共用节点, 单次 set_pose 都有约 5%
概率**整个 1000ms 超时拿不到应答**。若直接用于 20Hz 跟随, 每秒会卡一次 1 秒。

但"5% 概率丢一次"和"必须等满 1 秒"是两件事: 把超时压到 60ms 并在同一个
节拍内连发 3 次, 只要丢包是独立事件, 全丢概率就是 1.25e-4 —— 也就是几十秒
才可能漏一帧, 且漏的那一帧下一拍就补上。

本探针量化:
  * 每拍(20Hz) 用 timeout=T 连发 R 次, 统计"整拍失败"次数与每拍耗时
  * 扫描 T ∈ {60, 120, 1000} 看能否把 p99 压到节拍预算内
  * 顺带对比 set_pose_vector(批量服务) 是否更稳

用法: python3 tests/diag/probe_svc_retry.py [拍数]
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
pos = {}


def cb(msg):
    for p in msg.pose:
        pos[p.name] = (p.position.x, p.position.y, p.position.z)


def main():
    ticks = int(sys.argv[1]) if len(sys.argv) > 1 else 120
    node = gz_transport.Node()
    node.subscribe(Pose_V, TOPIC, cb)
    time.sleep(2.0)
    if MODEL not in pos:
        print('没有 %s' % MODEL)
        return 2
    p0 = list(pos[MODEL])
    m = Pose()
    m.name = MODEL
    m.orientation.w = 1.0

    print('=' * 78)
    print('set_pose 短超时 + 同帧重试   (每拍 %d 次, 共 %d 拍)' % (3, ticks))
    print('=' * 78)
    print('%8s %8s %8s %9s %9s %8s' %
          ('timeout', 'tries', '失败拍', 'p50(ms)', 'p99(ms)', 'max(ms)'))
    for to in (60, 120):
        fails = 0
        lat = []
        for i in range(ticks):
            # 目标绕 z 微动 ±2mm, 避免"总是同一个请求"
            m.position.x = p0[0]
            m.position.y = p0[1]
            m.position.z = p0[2] + 0.002 * (1 if i % 2 else -1)
            t = time.time()
            ok = False
            for _ in range(3):
                ret = node.request(SVC, m, Pose, Boolean, to)
                if (isinstance(ret, tuple) and bool(ret[0])
                        and bool(getattr(ret[1], 'data', False))):
                    ok = True
                    break
            lat.append((time.time() - t) * 1000.0)
            if not ok:
                fails += 1
                print('     !! 第 %d 拍 3 次全失败' % i)
            time.sleep(0.05 - min(0.05, (time.time() - t)))
        ls = sorted(lat)
        print('%8d %8d %8d %9.2f %9.2f %8.2f'
              % (to, 3, fails, statistics.median(lat),
                 ls[int(0.99 * len(ls))], ls[-1]))

    # 复原
    m.position.x, m.position.y, m.position.z = p0
    node.request(SVC, m, Pose, Boolean, 1000)
    return 0


if __name__ == '__main__':
    sys.exit(main())
