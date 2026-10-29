#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_odom_vs_truth.py —— 对比 轮式里程计(编码器) 与 真值 的角速度,
判断"指令→轮子转是否正确"(odom) 与 "轮子→车身是否打滑"(truth)。
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

import gz.transport13 as gzt
from gz.msgs10.pose_v_pb2 import Pose_V


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
        self.gzn = gzt.Node()
        self.truth = None
        self.odom = None
        self.node.subscribe(Pose_V, '/world/follow_world/dynamic_pose/info',
                            self.cb)

    def cb(self, m):
        for p in m.pose:
            if p.name == 'robot':
                st = m.header.stamp.sec + m.header.stamp.nsec * 1e-9
                self.truth = (p.position.x, p.position.y,
                              yaw_of(p.orientation), st)


def main():
    rclpy.init()
    n = Node('tmp_odom_vs_truth')
    pub = n.create_publisher(Twig := Twist, '/robot/cmd_vel', 10)
    cap = Cap()

    def ocb(msg):
        p = msg.pose.pose
        st = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        cap.odom = (p.position.x, p.position.y,
                    yaw_of(p.orientation), st)
        cap.odom_tw = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
    n.create_subscription(Odometry, '/robot/odom', ocb, 10)
    t = time.time()
    while (cap.truth is None or cap.odom is None) and time.time() - t < 10:
        rclpy.spin_once(n, timeout_sec=0.05)
    a_t = cap.truth
    a_o = cap.odom
    cmd_w = 1.0
    t = time.time()
    while time.time() - t < 3.0:
        m = Twist()
        m.angular.z = cmd_w
        pub.publish(m)
        rclpy.spin_once(n, timeout_sec=0.0)
        time.sleep(0.02)
    pub.publish(Twist())
    time.sleep(0.5)
    rclpy.spin_once(n, timeout_sec=0.2)
    b_t, b_o = cap.truth, cap.odom
    dt = b_t[3] - a_t[3]
    do = b_o[3] - a_o[3]
    print('cmd w=%+.2f' % cmd_w)
    print('真值: dyaw=%+.3f rad / %.2fs → %+.3f rad/s' % (
        wrap(b_t[2] - a_t[2]), dt, wrap(b_t[2] - a_t[2]) / dt))
    print('编码器: dyaw=%+.3f rad / %.2fs → %+.3f rad/s' % (
        wrap(b_o[2] - a_o[2]), do, wrap(b_o[2] - a_o[2]) / do))
    print('odom twist: v=%+.3f w=%+.3f' % (cap.odom_tw[0], cap.odom_tw[1]))
    print('>>> 判定: %s' % (
        '轮子按指令转, 车身打滑(摩擦/支撑问题)'
        if abs(wrap(b_o[2] - a_o[2]) / do - cmd_w) < 0.3
        else '轮子实际转速与指令不符(joint/motor 问题)'))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
