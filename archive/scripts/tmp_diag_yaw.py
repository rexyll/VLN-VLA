#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_diag_yaw.py —— 验证 cmd_vel.angular.z 符号与真值 yaw 变化方向的一致性
以及 vision /perception/person 的 bearing 与真值方位的一致性
"""
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

import gz.transport13 as gzt
from gz.msgs10.pose_v_pb2 import Pose_V


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.x * q.x))


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
                self.robot = (p.position.x, p.position.y, yaw_of(p.orientation))
            elif p.name == 'person':
                self.person = (p.position.x, p.position.y)


def main():
    rclpy.init()
    n = Node('tmp_diag_yaw')
    pub = n.create_publisher(Twist, '/robot/cmd_vel', 10)
    cap = Cap()
    t0 = time.time()
    while cap.robot is None and time.time() - t0 < 10:
        time.sleep(0.05)
    if cap.robot is None:
        print('NO POSE')
        return
    print('robot 初始: x=%.2f y=%.2f yaw=%+.1f°' % (
        cap.robot[0], cap.robot[1], math.degrees(cap.robot[2])))

    def drive(w, sec, tag):
        a = cap.robot
        t = time.time()
        while time.time() - t < sec:
            m = Twist()
            m.angular.z = float(w)
            pub.publish(m)
            time.sleep(0.05)
        m = Twist()
        pub.publish(m)
        time.sleep(0.4)
        b = cap.robot
        dy = math.degrees(b[2] - a[2])
        while dy > 180:
            dy -= 360
        while dy < -180:
            dy += 360
        print('%s: 指令 w=%+.1f 持续%.1fs → yaw 变化 %+.1f° (期望 %s)' % (
            tag, w, sec, dy, '正/逆时针(左)' if w > 0 else '负/顺时针(右)'))

    drive(+0.8, 2.0, '测试A')
    drive(-0.8, 2.0, '测试B')

    # 视觉 bearing 与真值方位对比
    vision = {}

    def vcb(msg):
        try:
            vision.update(json.loads(msg.data))
            vision['_t'] = time.time()
        except Exception:
            pass
    n.create_subscription(String, '/perception/person', vcb, 10)
    for _ in range(60):
        rclpy.spin_once(n, timeout_sec=0.05)
    r = cap.robot
    p = cap.person
    if p is not None and vision.get('detected'):
        true_b = math.degrees(math.atan2(p[1] - r[1], p[0] - r[0]) - r[2])
        while true_b > 180:
            true_b -= 360
        while true_b < -180:
            true_b += 360
        td = math.hypot(p[0] - r[0], p[1] - r[1])
        print('真值: 人相对机器人方位 %+.1f° 距离 %.2fm (cam=%s)' % (
            true_b, td, vision.get('cam')))
        print('视觉: bearing %+.1f° bearing_cam %+.1f° dist %.2fm u=%s' % (
            math.degrees(vision.get('bearing', 0)),
            math.degrees(vision.get('bearing_cam', 0)),
            vision.get('dist', -1), vision.get('u')))
        print('>>> bearing 符号一致: %s' % (
            'YES' if abs(math.degrees(vision['bearing']) - true_b) < 25
            else 'NO(镜像/偏差过大)'))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
