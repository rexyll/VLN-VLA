#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_set_follow.py —— 评测前置: 在整个评测期间把跟随距离钉死

演示语音素材里说的是"保持两米的距离", 会把 target_d 改成 2.0, 与配置里
调好的 track_dist_m=1.0 冲突, 评测口径(0.9~1.1m 带宽)就对不上了。语音
触发时刻不确定, 所以这里按 4Hz 持续重发, 保证整轮实验的基准一致。

用法: python3 tmp_set_follow.py <距离m> [持续秒数]
"""
import json
import sys
import time

import rclpy
from std_msgs.msg import String

D = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0

rclpy.init()
node = rclpy.create_node('set_follow_dist')
pub = node.create_publisher(String, '/robot/command', 10)
msg = String()
msg.data = json.dumps({'action': 'FOLLOW', 'distance': D})
t0 = time.time()
while time.time() - t0 < DUR:
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(0.15)
node.destroy_node()
rclpy.shutdown()
