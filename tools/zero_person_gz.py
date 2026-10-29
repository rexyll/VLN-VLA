#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""zero_person_gz.py —— 直发 gz /model/person/cmd_vel 零速(绕开 ROS 桥)。"""
import time

import gz.transport13 as gz_transport
import gz.msgs10.twist_pb2 as twist_pb2

node = gz_transport.Node()
pub = node.advertise('/model/person/cmd_vel', twist_pb2.Twist)
t0 = time.time()
n = 0
while time.time() - t0 < 6.0:
    m = twist_pb2.Twist()
    m.linear.x = 0.0
    m.angular.z = 0.0
    pub.publish(m)
    n += 1
    time.sleep(0.1)
print('gz-side zero published x%d' % n)
