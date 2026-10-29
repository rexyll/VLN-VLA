#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_vis_check.py —— 视觉测距/方位 vs Gazebo 真值 逐帧核对"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class Chk(Node):

    def __init__(self):
        super().__init__('vis_check')
        self.rx = self.ry = self.ryaw = None
        self.px = self.py = None
        self.vis = {}
        self.rows = []
        self.create_subscription(Odometry, '/robot/odom', self._rb, 20)
        self.create_subscription(Odometry, '/person/odom', self._pb, 20)
        self.create_subscription(String, '/perception/person', self._vb, 20)
        self.create_timer(0.5, self._tick)

    def _rb(self, m):
        p = m.pose.pose.position
        self.rx, self.ry, self.ryaw = p.x, p.y, yaw_of(m.pose.pose.orientation)

    def _pb(self, m):
        p = m.pose.pose.position
        self.px, self.py = p.x, p.y

    def _vb(self, m):
        try:
            self.vis = json.loads(m.data)
        except Exception:
            pass

    def _tick(self):
        if None in (self.rx, self.ry, self.px, self.py):
            return
        v = self.vis or {}
        d_true = math.hypot(self.px - self.rx, self.py - self.ry)
        brg_true = math.degrees(wrap(
            math.atan2(self.py - self.ry, self.px - self.rx) - self.ryaw))
        det = bool(v.get('detected'))
        row = (time.monotonic() % 100, d_true, brg_true,
               v.get('cam', ''), float(v.get('dist', -1)) if det else -1,
               math.degrees(float(v.get('bearing', 0))) if det else 0,
               bool(v.get('area', 0)),
               int(v.get('area', 0)) if det else 0)
        self.rows.append(row)
        print('t=%5.1f 真值d=%.2f 方位=%+6.1f | 视觉 %s d=%6.2f 方位=%+6.1f '
              '面积=%5d'
              % (len(self.rows) * 0.5, row[1], row[2],
                 row[3] or '-', row[4], row[5], row[7]), flush=True)


def main():
    rclpy.init()
    n = Chk()
    print('=== 视觉 vs 真值 核对 20s ===')
    t0 = time.monotonic()
    while rclpy.ok() and time.monotonic() - t0 < 20.0:
        rclpy.spin_once(n, timeout_sec=0.1)
    rows = [r for r in n.rows if r[4] > 0]
    if rows:
        err = [r[4] - r[1] for r in rows]
        err.sort()
        print('\n距离误差(视觉-真值): n=%d mean=%+.2f 中位=%+.2f '
              '范围[%+.2f,%+.2f]'
              % (len(err), sum(err) / len(err), err[len(err) // 2],
                 err[0], err[-1]))
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
