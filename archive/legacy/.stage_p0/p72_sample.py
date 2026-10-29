# -*- coding: utf-8 -*-
"""真值位姿采样: 订阅 gt_tf 发布的 /odom(= gz 物理位姿), 按 sim 时间记点。"""
import math
import sys

import rclpy
from nav_msgs.msg import Odometry

rclpy.init()
node = rclpy.create_node('p72_sampler')
f = open(sys.argv[1], 'w')


def cb(m):
    p = m.pose.pose.position
    q = m.pose.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    f.write('%.4f %.5f %.5f %.5f\n' % (t, p.x, p.y, yaw))
    f.flush()


node.create_subscription(Odometry, '/odom', cb, 20)
rclpy.spin(node)
