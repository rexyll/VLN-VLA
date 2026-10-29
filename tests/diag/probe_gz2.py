#!/usr/bin/env python3
"""probe_gz2.py —— set_pose 服务延迟 + 订阅可用性探针(开发期用)

① 同步 request 调 /world/follow_world/set_pose 200 次, 量平均耗时
   (喝水任务要按 25~30Hz 把杯子/水壶贴到手上, 单次必须 ≪ 10ms)
② 订 /clock 看订阅链路是否通(排查 dynamic_pose/info 收不到的原因)
用法: python3 probe_gz2.py
"""
import time

import gz.transport13 as gz
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.clock_pb2 import Clock

n = {"c": 0}


def cb(msg):
    n["c"] += 1
    return True


node = gz.Node()
print("sub /clock =", node.subscribe(Clock, '/clock', cb))
t0 = time.time()
while time.time() - t0 < 3.0:
    time.sleep(0.05)
print("clock msgs in 3s =", n["c"])

req = Pose()
req.name = 'obstacles'
req.orientation.w = 1.0
try:
    resp = node.request('/world/follow_world/set_pose', req, Pose, Boolean, 200)
    print("set_pose ok =", resp)
except Exception as e:  # noqa: BLE001
    print("set_pose err:", repr(e))

t0 = time.time()
N = 200
ok = 0
for i in range(N):
    req.position.z = 0.0
    try:
        r = node.request('/world/follow_world/set_pose', req, Pose, Boolean, 50)
        ok += 1 if r.data else 0
    except Exception as e:  # noqa: BLE001
        print("loop err:", repr(e))
        break
dt = time.time() - t0
print("set_pose x%d: ok=%d, %.2f ms/call (%.0f Hz max)"
      % (N, ok, dt * 1000.0 / max(1, N), N / max(1e-6, dt)))
