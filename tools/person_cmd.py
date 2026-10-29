#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""person_cmd.py —— 给人发 /person/command 并读 pw 日志确认(调试用)。
用法: python3 tools/person_cmd.py STOP|GO_DRINK|DRINK_DONE
STOP 的落地面证: person_walk 打 "PERSON 停顿"。
"""
import json
import os
import sys
import time

import rclpy
from std_msgs.msg import String

act = sys.argv[1] if len(sys.argv) > 1 else 'STOP'
LOG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..',
                   'vla_ws', 'logs', 'pw.log')
MARK = {'STOP': '停顿', 'GO_DRINK': '收到"去喝水"',
        'DRINK_DONE': '恢复随机活动'}.get(act, 'PERSON')


def log_has(tag, since):
    try:
        with open(LOG, encoding='utf-8', errors='ignore') as f:
            lines = f.readlines()
        return any(tag in l for l in lines[since:])
    except Exception:
        return False


rclpy.init()
n = rclpy.create_node('person_cmd')
pub = n.create_publisher(String, '/person/command', 10)
try:
    with open(LOG, encoding='utf-8', errors='ignore') as f:
        since = sum(1 for _ in f)
except Exception:
    since = 0
sent = 0
t0 = time.time()
ok = False
while time.time() - t0 < 10.0:
    pub.publish(String(data=json.dumps({'action': act})))
    sent += 1
    t1 = time.time()
    while time.time() - t1 < 0.3:
        rclpy.spin_once(n, timeout_sec=0.05)
    if log_has(MARK, since):
        ok = True
        break
print('person %s: %s (%d 条)' % (act, '已确认' if ok else '未确认!', sent))
n.destroy_node()
rclpy.shutdown()
