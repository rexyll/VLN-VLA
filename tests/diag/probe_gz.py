#!/usr/bin/env python3
"""probe_gz.py —— gz 接口探针(开发期用, 不改动仿真)

确认三件事:
  ① /world/follow_world/dynamic_pose/info 里有没有 **link 级** 位姿
     (有 → 手掌/物体的真值可直接读, 不用另搭反馈链)
  ② gz.transport13.Node.request 的签名
  ③ /world/follow_world/set_pose 服务是否可调
用法: python3 probe_gz.py
"""
import inspect
import time

import gz.transport13 as gz
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean

names = {}
stat = {"n": 0}


def cb(msg):
    stat["n"] += 1
    for p in msg.pose:
        names[p.name] = (round(p.position.x, 3), round(p.position.y, 3),
                         round(p.position.z, 3))
    return True


node = gz.Node()
print("subscribe =",
      node.subscribe(Pose_V, '/world/follow_world/dynamic_pose/info', cb))
t0 = time.time()
while time.time() - t0 < 12.0 and stat['n'] < 3:
    time.sleep(0.1)
print("msgs =", stat['n'], "entities =", len(names))
for k in sorted(names):
    print("  ENT", k, names[k])

print("request sig:", inspect.signature(gz.Node.request))

req = Pose()
req.name = 'obstacles'
req.orientation.w = 1.0
res = {"r": None}


def scb(msg):
    res["r"] = msg


try:
    node.request('/world/follow_world/set_pose', req, 500, scb)
    print("request(topic,req,timeout,cb) OK")
    t0 = time.time()
    while time.time() - t0 < 2.0 and res['r'] is None:
        time.sleep(0.05)
    print("resp =", res['r'])
except TypeError as e:
    print("4-arg request failed:", e)
