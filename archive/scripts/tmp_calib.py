#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_calib.py —— 标定 cmd_vel → 实际运动增益 (真值 yaw / 位移)
对多组 (v, w) 指令分别测量稳态 1s 内的 yaw 角速度与线速度。
"""
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
                self.robot = (p.position.x, p.position.y, yaw_of(p.orientation),
                              time.monotonic())


def main():
    rclpy.init()
    n = Node('tmp_calib')
    pub = n.create_publisher(Twist, '/robot/cmd_vel', 10)
    cap = Cap()
    t0 = time.time()
    while cap.robot is None and time.time() - t0 < 10:
        time.sleep(0.05)

    def measure(v, w, sec=3.0):
        t = time.time()
        while time.time() - t < sec:
            m = Twist()
            m.linear.x = float(v)
            m.angular.z = float(w)
            pub.publish(m)
            time.sleep(0.05)
        # 最后 1s 的稳态
        a = cap.robot
        time.sleep(0.9)
        b = cap.robot
        pub.publish(Twist())
        time.sleep(0.6)
        dt = b[3] - a[3]
        dy = b[2] - a[2]
        while dy > math.pi:
            dy -= 2 * math.pi
        while dy < -math.pi:
            dy += 2 * math.pi
        dist = math.hypot(b[0] - a[0], b[1] - a[1])
        # 位移投影到机头(取中点航向)
        hd = a[2]
        proj = (b[0] - a[0]) * math.cos(hd) + (b[1] - a[1]) * math.sin(hd)
        return dy / dt, proj / dt, dist / dt

    print('%-22s %10s %10s %8s' % ('指令', '实测w', '实测v(投影)', 'v比'))
    for (v, w) in [(0, 0.4), (0, 0.8), (0, 1.5), (0.3, 0), (0.35, 0)]:
        wr, vr, _ = measure(v, w)
        print('v=%+.2f w=%+.2f        %+9.3f %+9.3f %8s' % (
            v, w, wr, vr,
            '-' if abs(v) < 1e-3 else '%.2f' % (vr / v if v else 0)))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
