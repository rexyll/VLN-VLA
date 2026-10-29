#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_calib2.py —— 稳态标定 + 打滑判据

只统计指令窗的"中段"(40%~100%), 避开加速斜坡与松手后的减速尾巴, 因此
测出来的是真正的稳态速率。同时读取 DiffDrive 插件自己积分的里程计:
  - 若 odom 速率 ≈ 指令, 但 truth 速率明显偏小 → 轮子打滑(摩擦/承重问题)
  - 若 odom 速率也偏小 → 插件侧速度指令没到位(加速度限幅/JointVelocityCmd)

用法: tmp_calib2.py v w sec
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

import gz.transport13 as gzt
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.odometry_pb2 import Odometry


def yaw_of(q):
    # 分母必须是 1-2*(y²+z²): 写成 (y²+x²) 会把 yaw 压缩, 90° 读成 45°。
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class Cap:
    """truth: 逐帧累加车体航向/里程; odom: 直接取插件积分结果"""

    def __init__(self):
        self.n = gzt.Node()
        self.have = False
        self.t0 = None            # 首帧仿真时刻
        self.last = None          # (x, y, yaw, stamp)
        self.acc_yaw = 0.0
        self.acc_dist = 0.0
        self.mark = None          # 中段起点 (t, acc_yaw, acc_dist)
        self.end = None           # 中段终点
        self.odom = None          # (lin_x, ang_z, stamp)
        self.n.subscribe(Pose_V, '/world/follow_world/dynamic_pose/info',
                         self.cb_pose)
        self.n.subscribe(Odometry, '/model/robot/odometry', self.cb_odom)

    def cb_odom(self, m):
        self.odom = (m.twist.linear.x, m.twist.angular.z,
                     m.header.stamp.sec + m.header.stamp.nsec * 1e-9)

    def cb_pose(self, m):
        for p in m.pose:
            if p.name != 'robot':
                continue
            st = m.header.stamp.sec + m.header.stamp.nsec * 1e-9
            y = yaw_of(p.orientation)
            x, yy = p.position.x, p.position.y
            if self.last is not None:
                ds = st - self.last[3]
                if ds > 1e-6:
                    self.acc_yaw += wrap(y - self.last[2])
                    self.acc_dist += ((x - self.last[0]) * math.cos(self.last[2])
                                      + (yy - self.last[1]) * math.sin(self.last[2]))
            self.last = (x, yy, y, st)
            if self.t0 is None:
                self.t0 = st
            self.have = True
            return


def main():
    rclpy.init()
    node = Node('tmp_calib2')
    pub = node.create_publisher(Twist, '/robot/cmd_vel', 10)
    cap = Cap()
    t = time.time()
    while not cap.have and time.time() - t < 10:
        time.sleep(0.05)
    if not cap.have:
        print('!! 收不到 /world/follow_world/dynamic_pose/info')
        return
    v, w, sec = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
    cap.acc_yaw = cap.acc_dist = 0.0

    t_start = time.time()
    while True:
        el = time.time() - t_start
        m = Twist()
        m.linear.x = v
        m.angular.z = w
        pub.publish(m)
        # 中段锚点: 指令已持续 40%(且至少 1.2s) 时记一次
        if cap.mark is None and el >= max(1.2, sec * 0.4):
            cap.mark = (cap.last[3], cap.acc_yaw, cap.acc_dist)
        if el >= sec:
            break
        time.sleep(0.02)
    cap.end = (cap.last[3], cap.acc_yaw, cap.acc_dist, cap.odom)
    pub.publish(Twist())
    time.sleep(0.5)

    if cap.mark is None:
        print('!! 采样失败')
        return
    dt = cap.end[0] - cap.mark[0]
    dyaw = cap.end[1] - cap.mark[1]
    dds = cap.end[2] - cap.mark[2]
    if dt <= 0:
        print('!! 仿真时间未推进')
        return
    wr, vr = dyaw / dt, dds / dt
    o = cap.end[3]
    print('指令 v=%+.2f w=%+.2f  稳态窗口 %.2fs' % (v, w, dt))
    print('  真值  v=%+.3f (比 %+.2f)  w=%+.3f (比 %+.2f)'
          % (vr, (vr / v if abs(v) > 1e-6 else 0.0),
             wr, (wr / w if abs(w) > 1e-6 else 0.0)))
    if o:
        print('  里程计 v=%+.3f  w=%+.3f   ← 插件自认的速度' % (o[0], o[1]))
        print('  判据: 里程计≈指令 且 真值偏小 ⇒ 打滑; 两者都偏小 ⇒ 指令没到位')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
