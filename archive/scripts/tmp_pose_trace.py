#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_pose_trace.py —— 打印机器人完整位姿(z/roll/pitch) + 指令响应时序列,
用于判断是否发生侧翻/打滑导致 yaw 测量异常"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

import gz.transport13 as gzt
from gz.msgs10.pose_v_pb2 import Pose_V


def rpy(q):
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.x * q.x))
    return roll, pitch, yaw


class Cap:
    def __init__(self):
        self.node = gzt.Node()
        self.robot = None
        self.person = None
        self.node.subscribe(Pose_V, '/world/follow_world/dynamic_pose/info',
                            self.cb)

    def cb(self, m):
        for p in m.pose:
            if p.name == 'robot':
                r, pi, y = rpy(p.orientation)
                self.robot = (p.position.x, p.position.y, p.position.z, r, pi, y)
            elif p.name == 'person':
                self.person = (p.position.x, p.position.y)


def main():
    rclpy.init()
    n = Node('tmp_pose_trace')
    pub = n.create_publisher(Twist, '/robot/cmd_vel', 10)
    cap = Cap()
    t0 = time.time()
    while cap.robot is None and time.time() - t0 < 10:
        time.sleep(0.05)
    v = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    w = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    sec = float(sys.argv[3]) if len(sys.argv) > 3 else 6.0
    print('cmd v=%.2f w=%.2f for %.1fs' % (v, w, sec))
    t = time.time()
    nextp = 0.0
    while time.time() - t < sec:
        m = Twist()
        m.linear.x = v
        m.angular.z = w
        pub.publish(m)
        if time.time() - t >= nextp:
            nextp += 0.5
            r = cap.robot
            p = cap.person
            print('t=%.1f pos=(%.2f,%.2f,%.3f) rpy=(%.0f,%.0f,%.0f)° '
                  'person=(%.1f,%.1f)' % (
                      time.time() - t, r[0], r[1], r[2],
                      math.degrees(r[3]), math.degrees(r[4]),
                      math.degrees(r[5]), p[0], p[1]))
        time.sleep(0.05)
    pub.publish(Twist())
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
