#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""zero_person.py —— 向 /person/cmd_vel 发零速并锁存(person_walk 已停时用)。"""
import rclpy
import time
from geometry_msgs.msg import Twist

rclpy.init()
n = rclpy.create_node('zero_person')
pub = n.create_publisher(Twist, '/person/cmd_vel', 10)
for _ in range(12):
    pub.publish(Twist())
    rclpy.spin_once(n, timeout_sec=0.05)
    time.sleep(0.15)
print('person cmd_vel zeroed')
n.destroy_node()
rclpy.shutdown()
