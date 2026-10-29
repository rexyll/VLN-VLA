#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_unpause.py —— 用 gz.transport13 查询/解除仿真暂停 (CLI 数据通道在此 VM 不可用)"""
import sys
import time

import gz.transport13 as gzt
from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.msgs10.world_control_pb2 import WorldControl
from gz.msgs10.boolean_pb2 import Boolean

node = gzt.Node()
stats = []
node.subscribe(WorldStatistics, '/stats', lambda m: stats.append(m))
t0 = time.time()
while time.time() - t0 < 4 and not stats:
    time.sleep(0.1)
if not stats:
    print('STATS-NO-DATA')
    sys.exit(1)
s = stats[-1]
paused = s.paused
print('paused=%s sim_time=%.1f rtf=%.2f' % (
    paused, s.sim_time.sec + s.sim_time.nsec * 1e-9, s.real_time_factor))
if not paused:
    print('RUNNING')
    sys.exit(0)
print('→ 解除暂停...')
ok = node.request('/world/follow_world/control', WorldControl(pause=False),
                  'gz.msgs.WorldControl', 'gz.msgs.Boolean', 2000)
print('unpause 响应:', ok)
time.sleep(1.5)
stats.clear()
t0 = time.time()
while time.time() - t0 < 4 and len(stats) < 5:
    time.sleep(0.1)
if stats:
    s2 = stats[-1]
    print('after: paused=%s sim_time=%.1f' % (
        s2.paused, s2.sim_time.sec + s2.sim_time.nsec * 1e-9))
