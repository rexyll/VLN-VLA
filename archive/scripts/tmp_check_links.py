#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""检查 person / robot 各 link 的实际高度 —— 验证孤立 link 是否还在掉落。

正常值(person 立于地面 z=0.02):
  person::skirt  z ≈ 0.30      person::torso  z ≈ 0.83
  person::head   z ≈ 1.33      robot::caster  z ≈ 0.03
若 torso/head 的 z 掉到 0.1x, 说明 fixed joint 没生效。
"""
import sys
import time

import gz.transport13 as gt
from gz.msgs10.pose_v_pb2 import Pose_V

WORLD = 'follow_world'
res = {}


def cb(msg):
    for p in msg.pose:
        n = p.name
        if n.startswith('person::') or n.startswith('robot::'):
            res[n] = (p.position.x, p.position.y, p.position.z)


node = gt.Node()
node.subscribe(Pose_V, '/world/%s/pose/info' % WORLD, cb)

t0 = time.time()
while time.time() - t0 < 3.0 and len(res) < 6:
    time.sleep(0.1)

KEYS = ['person::skirt', 'person::torso', 'person::head',
        'robot::chassis', 'robot::caster']
print('%-16s %8s %8s %8s' % ('entity', 'x', 'y', 'z'))
for k in KEYS:
    v = res.get(k)
    print('%-16s %8.2f %8.2f %8.2f' % ((k,) + v) if v
          else '%-16s %8s' % (k, '(未见)'))

bad = []
for k, lo in (('person::torso', 0.60), ('person::head', 1.05),
              ('robot::caster', -0.05)):
    v = res.get(k)
    if v and v[2] < lo:
        bad.append('%s z=%.2f < %.2f' % (k, v[2], lo))
print()
print('判定: ' + ('✓ 全部在位' if not bad else '✗ 掉落: ' + '; '.join(bad)))
sys.exit(0 if not bad else 1)
