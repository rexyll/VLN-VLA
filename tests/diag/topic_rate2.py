#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_rate2.py —— 逐帧累加测真实角速度/线速度(避免 wrap 折算错误)
用法: tmp_rate2.py v w sec
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
    # 分母必须是 1-2*(y²+z²)。写成 1-2*(y²+x²) 会让 atan2(x, ·) 在 yaw≈90°
    # 处把角速度压到 0，长距离纯旋转积分会系统性低估(实测 90° 读成 45°)。
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class Cap:
    def __init__(self):
        self.node = gzt.Node()
        self.last = None
        self.acc_yaw = 0.0
        self.acc_v = 0.0
        self.t0 = None
        self.t1 = None
        self.n = 0
        self.node.subscribe(Pose_V, '/world/follow_world/dynamic_pose/info',
                            self.cb)

    def cb(self, m):
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
                    self.acc_v += ((x - self.last[0]) * math.cos(self.last[2])
                                   + (yy - self.last[1]) * math.sin(self.last[2]))
                    self.n += 1
            self.last = (x, yy, y, st)
            if self.t0 is None:
                self.t0 = st
            self.t1 = st


def main():
    rclpy.init()
    n = Node('tmp_rate2')
    pub = n.create_publisher(Twist, '/robot/cmd_vel', 10)
    cap = Cap()
    t = time.time()
    while cap.last is None and time.time() - t < 10:
        time.sleep(0.05)
    v, w, sec = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
    # 复位累加器
    cap.acc_yaw = cap.acc_v = 0.0
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
    ds = (cap.t1 - cap.t0) if (cap.t0 and cap.t1) else 1.0
    wr = cap.acc_yaw / ds
    vr = cap.acc_v / ds
    print('cmd v=%+.2f w=%+.2f | 仿真 %.2fs 帧 %d | 实测 w=%+.3f (比 %+.2f)'
          ' v=%+.3f (比 %+.2f)' % (
              v, w, ds, cap.n, wr,
              wr / w if abs(w) > 1e-6 else 0.0,
              vr, vr / v if abs(v) > 1e-6 else 0.0))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
