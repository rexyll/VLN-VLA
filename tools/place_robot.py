#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""place_robot.py —— 把机器人经 GraspCarry place 通道摆到服务位(桌面抓取位)。

位置 = scene_layout.SERVE_POSE, 朝向 +x(面向桌子): 距桌沿 0.38m(> 0.28m
深度硬闸, 不触发避障), 臂展覆盖壶/杯。
"""
import json
import math
import os
import time

import rclpy
from std_msgs.msg import String

SERVE = (1.75, 0.35)          # L.SERVE_POSE
YAW = 0.0

rclpy.init()
n = rclpy.create_node('place_robot')
pub = n.create_publisher(String, '/drink/carry', 10)
x, y = SERVE
qz, qw = math.sin(YAW / 2), math.cos(YAW / 2)
msg = 'place robot %.6f %.6f 0.000000 0 0 %.6f %.6f' % (x, y, qz, qw)
for _ in range(6):
    pub.publish(String(data=msg))
    rclpy.spin_once(n, timeout_sec=0.1)
    time.sleep(0.3)
print('place robot sent: (%.2f, %.2f, yaw %.2f)' % (x, y, YAW))
n.destroy_node()
rclpy.shutdown()
