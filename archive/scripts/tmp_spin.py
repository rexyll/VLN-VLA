#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_spin.py —— 判定"速度损失"是打滑还是有效半径偏小

原理: 轮子是绕自身局部 z 轴转的(revolute 关节轴 = child 0 0 -1)。
从 dynamic_pose 拿到 chassis 与 wheel 的世界朝向, 抠出相对旋转
    R_rel = R_chassis^T * R_wheel = R_x(90°) * R_spin(θ)
再左乘 R_x(90°)^T 得到纯自转 R_spin, θ = atan2(R[1,0], R[0,0]),
逐帧累加即轮子真实转角。轮子转过的弧长 r*Δθ 与车体实际位移比较:
  - 两者接近 → 轮子没打滑, 损失来自插件标定(有效半径/轮距)
  - 弧长明显大于位移 → 真打滑(摩擦/承重不足)
用法: tmp_spin.py v w sec
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

import gz.transport13 as gzt
from gz.msgs10.pose_v_pb2 import Pose_V

R_WHEEL = 0.06


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def qconj(q):
    return (q[0], -q[1], -q[2], -q[3])


def qrot(q, v):
    """用四元数旋转向量"""
    qv = (0.0, v[0], v[1], v[2])
    r = qmul(qmul(q, qv), qconj(q))
    return (r[1], r[2], r[3])


def yaw_of(q):
    return math.atan2(2.0 * (q[0] * q[3] + q[1] * q[2]),
                      1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]))


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


S = {'chassis': None, 'wheels': {}, 'names': set()}
R_X90T = (math.cos(-math.pi / 4), math.sin(-math.pi / 4), 0.0, 0.0)  # 反向 roll


def cb(m):
    for p in m.pose:
        S['names'].add(p.name)
        q = (p.orientation.w, p.orientation.x, p.orientation.y,
             p.orientation.z)
        if p.name == 'robot':
            S['chassis'] = (p.position.x, p.position.y, q)
        elif 'wheel' in p.name:
            S['wheels'][p.name] = q


def spin_theta(q_ch, q_w):
    """抠出轮子相对底盘的纯自转角"""
    rel = qmul(qconj(q_ch), q_w)          # R_x90 * R_spin
    spin = qmul(R_X90T, rel)              # R_spin
    w, x, y, z = spin
    # 绕 z 轴: 矩阵 [0,0] = 1-2(x²+y²), [0,1] = 2(xy - wz) → atan2 用 (x,y,z)
    m00 = 1.0 - 2.0 * (x * x + y * y)
    m10 = 2.0 * (x * y + w * z)
    return math.atan2(m10, m00)


def main():
    rclpy.init()
    node = Node('tmp_spin')
    pub = node.create_publisher(Twist, '/robot/cmd_vel', 10)
    gz = gzt.Node()
    gz.subscribe(Pose_V, '/world/follow_world/dynamic_pose/info', cb)
    t = time.time()
    while S['chassis'] is None and time.time() - t < 10:
        time.sleep(0.05)
    if S['chassis'] is None:
        print('!! 无 chassis 位姿')
        return
    if not S['wheels']:
        print('!! 找不到轮子 link, 现有条目: %s' % sorted(S['names']))
        return

    v, w, sec = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
    # 初始化上一拍
    q_ch = S['chassis'][2]
    prev = {'pos': (S['chassis'][0], S['chassis'][1]),
            'ch_theta': None, 'spin': {}, 'yaw': yaw_of(q_ch)}
    for k, qw in S['wheels'].items():
        prev['spin'][k] = spin_theta(q_ch, qw)
    acc_dist = 0.0
    acc_spin = {k: 0.0 for k in S['wheels']}
    acc_yaw = 0.0

    t0 = time.time()
    mark = None
    while True:
        el = time.time() - t0
        m = Twist()
        m.linear.x = v
        m.angular.z = w
        pub.publish(m)
        # 逐帧累加(与发布无关, 用最新快照做差分)
        ch = S['chassis']
        if ch is not None:
            x, y, qc = ch
            yaw = yaw_of(qc)
            px, py = prev['pos']
            acc_dist += ((x - px) * math.cos(prev['yaw'])
                         + (y - py) * math.sin(prev['yaw']))
            acc_yaw += wrap(yaw - prev['yaw'])
            for k, qw in S['wheels'].items():
                if k in prev['spin']:
                    th = spin_theta(qc, qw)
                    acc_spin[k] = acc_spin.get(k, 0.0) + wrap(
                        th - prev['spin'][k])
                    prev['spin'][k] = th
            prev = {'pos': (x, y), 'ch_theta': None, 'spin': prev['spin'],
                    'yaw': yaw}
        if el >= sec:
            break
        time.sleep(0.01)
    pub.publish(Twist())

    print('指令 v=%+.2f w=%+.2f  时长 %.1fs' % (v, w, sec))
    print('  车体位移 = %+.3f m   (等效 v=%+.3f)'
          % (acc_dist, acc_dist / sec))
    print('  车体转角 = %+.3f rad (等效 w=%+.3f)' % (acc_yaw, acc_yaw / sec))
    for k, th in acc_spin.items():
        print('  %s 自转 = %+.3f rad → 弧长 %.3f m'
              % (k, th, abs(th) * R_WHEEL))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
