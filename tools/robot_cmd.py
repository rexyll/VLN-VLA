#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""robot_cmd.py —— 给机器人发 /robot/command(调试用)。
用法: python3 tools/robot_cmd.py STOP|FOLLOW|DRINK
发布后**读 follower 日志确认落地**(DDS 发现期会丢前几条, §2.1/#24)。
"""
import json
import os
import sys
import time

import rclpy
from std_msgs.msg import String

act = sys.argv[1] if len(sys.argv) > 1 else 'STOP'
LOG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..',
                   'vla_ws', 'logs', 'follower.log')


def log_has(tag):
    try:
        with open(LOG, encoding='utf-8', errors='ignore') as f:
            return tag in f.read()
    except Exception:
        return False


rclpy.init()
n = rclpy.create_node('robot_cmd')
pub = n.create_publisher(String, '/robot/command', 10)
mark = ('收到停止指令' if act == 'STOP'
        else '收到跟随指令' if act == 'FOLLOW' else '收到喝水请求')
sent = 0
t0 = time.time()
ok = False
while time.time() - t0 < 10.0:
    pub.publish(String(data=json.dumps({'action': act})))
    sent += 1
    t1 = time.time()
    while time.time() - t1 < 0.3:
        rclpy.spin_once(n, timeout_sec=0.05)
    if log_has(mark):
        ok = True
        break
print('robot %s: %s (%d 条)' % (act, '已确认' if ok else '未确认!', sent))
n.destroy_node()
rclpy.shutdown()
