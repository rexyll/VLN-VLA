#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""park_streams.py —— 应急: 把悬空的水柱段/液面盘摆回地板下(隐藏位)。"""
import rclpy
import time
from std_msgs.msg import String

NAMES = ['cup_water_disc'] + ['stream%d' % i for i in range(6)]
rclpy.init()
n = rclpy.create_node('park_streams')
pub = n.create_publisher(String, '/drink/carry', 10)
for rnd in range(3):
    for i, name in enumerate(NAMES):
        z = -1.0
        msg = 'place %s 0 0 %.3f 0 0 0 0 1' % (name, z)
        pub.publish(String(data=msg))
        rclpy.spin_once(n, timeout_sec=0.05)
    time.sleep(0.4)
print('parked')
n.destroy_node()
rclpy.shutdown()
