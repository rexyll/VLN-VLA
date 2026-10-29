#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_wheel_rate.py —— 从轮子连杆姿态反推关节角速度, 判断打滑还是电机不给力
用法: tmp_wheel_rate.py v w sec
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

import gz.transport13 as gzt
from gz.msgs10.pose_v_pb2 import Pose_V


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def qconj(a):
    return (-a[0], -a[1], -a[2], a[3])


def qang(a):
    return 2.0 * math.acos(max(-1.0, min(1.0, a[3])))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.x * q.x))


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class Cap:
    def __init__(self):
        self.node = gzt.Node()
        self.prev = None          # (t, [(x,y), q_l, q_r], yaw)
        self.accL = self.accR = self.accYaw = 0.0
        self.t0 = self.t1 = None
        self.n = 0
        self.node.subscribe(Pose_V, '/world/follow_world/dynamic_pose/info',
                            self.cb)

    def cb(self, m):
        st = m.header.stamp.sec + m.header.stamp.nsec * 1e-9
        robot = None
        wheels = {}
        for p in m.pose:
            if p.name == 'robot':
                robot = p
            elif p.name in ('left_wheel', 'right_wheel'):
                q = p.orientation
                wheels.setdefault(p.name, (q.x, q.y, q.z, q.w))
        if robot is None or len(wheels) < 2:
            return
        y = yaw_of(robot.orientation)
        cur = (st, wheels['left_wheel'], wheels['right_wheel'], y)
        if self.prev is not None:
            ds = st - self.prev[0]
            if ds > 1e-6:
                dL = qmul(qconj(self.prev[1]), cur[1])
                dR = qmul(qconj(self.prev[2]), cur[2])
                self.accL += qang(dL) / ds
                self.accR += qang(dR) / ds
                self.accYaw += wrap(y - self.prev[3])
                self.n += 1
        self.prev = cur
        if self.t0 is None:
            self.t0 = st
        self.t1 = st


def main():
    rclpy.init()
    n = Node('tmp_wheel_rate')
    pub = n.create_publisher(Twist, '/robot/cmd_vel', 10)
    cap = Cap()
    t = time.time()
    while cap.prev is None and time.time() - t < 10:
        time.sleep(0.05)
    v, w, sec = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
    cap.accL = cap.accR = cap.accYaw = 0.0
    cap.n = 0
    cap.t0 = None
    t = time.time()
    while time.time() - t < sec:
        m = Twist()
        m.linear.x = v
        m.angular.z = w
        pub.publish(m)
        time.sleep(0.02)
    pub.publish(Twist())
    time.sleep(0.8)
    if cap.n == 0:
        print('无样本')
        return
    rL = cap.accL / cap.n
    rR = cap.accR / cap.n
    wy = cap.accYaw / ((cap.t1 - cap.t0) or 1.0)
    # 期望: 左轮 = (v - w*sep/2)/r , 右轮 = (v + w*sep/2)/r ; sep=0.26 r=0.06
    eL = (v - w * 0.13) / 0.06
    eR = (v + w * 0.13) / 0.06
    print('cmd v=%+.2f w=%+.2f | 轮速(实测|dθ|/dt): L=%+.2f R=%+.2f rad/s '
          '| 期望 L=%+.2f R=%+.2f' % (v, w, rL, rR, eL, eR))
    print('  车身 yaw 速率 = %+.3f rad/s (指令 %+.2f)' % (wy, w))


if __name__ == '__main__':
    main()
