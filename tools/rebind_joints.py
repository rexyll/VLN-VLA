#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""rebind_joints.py —— 应急: 把插件 joints 表的 water/jet 登记换绑到活实体,
止住 "component ... does not exist" 每 tick 洪水(纯 ROS 通道, 可靠)。"""
import rclpy
import time
from std_msgs.msg import String

rclpy.init()
n = rclpy.create_node('rebind')
pub = n.create_publisher(String, '/drink/carry', 10)
for txt in ('joint cup water_joint 0.000000',
            'joint kettle jet_joint 0.000000'):
    for _ in range(4):
        pub.publish(String(data=txt))
        rclpy.spin_once(n, timeout_sec=0.1)
        time.sleep(0.2)
print('rebind sent')
n.destroy_node()
rclpy.shutdown()
