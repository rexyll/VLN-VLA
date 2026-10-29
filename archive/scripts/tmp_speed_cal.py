#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_speed_cal.py —— 机器人实际线速度标定(指令 vs 真实位移)

直发 /robot/cmd_vel, 用 /robot/odom 测真实位移速度。
跑之前必须 pkill follower, 否则两个发布者抢同一个话题。
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class Cal(Node):

    def __init__(self):
        super().__init__('speed_cal')
        self.pub = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.x = self.y = None
        self.create_subscription(Odometry, '/robot/odom', self._cb, 20)

    def _cb(self, m):
        p = m.pose.pose.position
        self.x, self.y = p.x, p.y

    def spin_for(self, sec, v, w=0.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < sec:
            m = Twist()
            m.linear.x = float(v)
            m.angular.z = float(w)
            self.pub.publish(m)
            rclpy.spin_once(self, timeout_sec=0.02)


def main():
    rclpy.init()
    n = Cal()
    print('等待 odom...', flush=True)
    t0 = time.monotonic()
    while n.x is None and time.monotonic() - t0 < 10:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.x is None:
        print('!! 无 odom')
        return
    print('%8s %8s %8s %8s' % ('指令v', '实测v', '位移m', '用时s'), flush=True)
    for v in (0.20, 0.35, 0.52, 0.62, 0.75, 0.90, 1.10):
        # 先停稳
        n.spin_for(1.0, 0.0)
        x0, y0 = n.x, n.y
        dur = 2.5
        n.spin_for(dur, v)
        x1, y1 = n.x, n.y
        n.spin_for(0.8, 0.0)
        dist = math.hypot(x1 - x0, y1 - y0)
        print('%8.2f %8.3f %8.3f %8.2f'
              % (v, dist / dur, dist, dur), flush=True)
        time.sleep(0.3)
    n.spin_for(0.5, 0.0)
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
