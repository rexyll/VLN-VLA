#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_rate.py —— 用仿真时间(header.stamp)计算真实角速度/线速度, 并给出实时因子
用法: tmp_rate.py v w sec
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

import gz.transport13 as gzt
from gz.msgs10.pose_v_pb2 import Pose_V


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.x * q.x))


class Cap:
    def __init__(self):
        self.node = gzt.Node()
        self.robot = None
        self.node.subscribe(Pose_V, '/world/follow_world/dynamic_pose/info',
                            self.cb)

    def cb(self, m):
        for p in m.pose:
            if p.name == 'robot':
                st = m.header.stamp.sec + m.header.stamp.nsec * 1e-9
                self.robot = (p.position.x, p.position.y, yaw_of(p.orientation),
                              st, time.monotonic())


def main():
    rclpy.init()
    n = Node('tmp_rate')
    pub = n.create_publisher(Twist, '/robot/cmd_vel', 10)
    cap = Cap()
    t0 = time.time()
    while cap.robot is None and time.time() - t0 < 10:
        time.sleep(0.05)
    v = float(sys.argv[1])
    w = float(sys.argv[2])
    sec = float(sys.argv[3])
    frames = []

    def rec():
        if cap.robot is not None:
            frames.append(cap.robot)

    t = time.time()
    while time.time() - t < sec:
        m = Twist()
        m.linear.x = v
        m.angular.z = w
        pub.publish(m)
        rec()
        time.sleep(0.02)
    pub.publish(Twist())
    time.sleep(1.0)
    rec()
    a, b = frames[len(frames) // 3], frames[-1]
    ds = b[3] - a[3]
    dw = b[4] - a[4]
    dy = b[2] - a[2]
    while dy > math.pi:
        dy -= 2 * math.pi
    while dy < -math.pi:
        dy += 2 * math.pi
    dx, dyy = b[0] - a[0], b[1] - a[1]
    proj = dx * math.cos(a[2]) + dyy * math.sin(a[2])
    print('cmd v=%+.2f w=%+.2f | 采样 %d 帧 | 仿真时长 %.2fs 墙钟 %.2fs '
          'RTF=%.2f' % (v, w, len(frames), ds, dw, ds / max(dw, 1e-6)))
    print('  实测: w=%+.3f rad/s (指令 %+.2f, 比 %+.2f)  '
          'v=%+.3f m/s (指令 %+.2f, 比 %+.2f)' % (
              dy / max(ds, 1e-6), w,
              (dy / max(ds, 1e-6)) / w if abs(w) > 1e-6 else 0,
              proj / max(ds, 1e-6), v,
              (proj / max(ds, 1e-6)) / v if abs(v) > 1e-6 else 0))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
