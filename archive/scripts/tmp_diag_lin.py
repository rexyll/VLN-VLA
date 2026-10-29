#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_diag_lin.py —— 验证 linear.x 符号: 前进指令是否真的让机器人朝机头方向前进"""
import math
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
                self.robot = (p.position.x, p.position.y, yaw_of(p.orientation))


def main():
    rclpy.init()
    n = Node('tmp_diag_lin')
    pub = n.create_publisher(Twist, '/robot/cmd_vel', 10)
    cap = Cap()
    t0 = time.time()
    while cap.robot is None and time.time() - t0 < 10:
        time.sleep(0.05)

    def drive(v, sec, tag):
        a = cap.robot
        t = time.time()
        while time.time() - t < sec:
            m = Twist()
            m.linear.x = float(v)
            pub.publish(m)
            time.sleep(0.05)
        pub.publish(Twist())
        time.sleep(0.5)
        b = cap.robot
        dx, dy = b[0] - a[0], b[1] - a[1]
        # 位移在初始机头方向上的投影
        proj = dx * math.cos(a[2]) + dy * math.sin(a[2])
        print('%s: 指令 v=%+.2f %.1fs → 位移投影(机头方向)=%+.2fm '
              '(期望 %s)' % (tag, v, sec, proj,
                            '正=前进' if v > 0 else '负=后退'))

    print('robot: x=%.2f y=%.2f yaw=%+.1f°' % (
        cap.robot[0], cap.robot[1], math.degrees(cap.robot[2])))
    drive(+0.35, 2.0, '测试前进')
    drive(-0.35, 2.0, '测试后退')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
