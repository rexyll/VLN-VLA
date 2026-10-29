#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P60d: ROS→bridge→gz 路径的方向约定测试。

背景: 直连 gz 话题实测 cmd.x=+0.5 时机器人沿车头**反方向**走
(Δpos 与 yaw 差 180°)。但 follower 走的是 ROS /robot/cmd_vel →
ros_gz_bridge → gz /model/robot/cmd_vel 这条路, 中间可能有符号转换。
必须直接测 ROS 路径, 才能判断 follower 到底往哪走。

测试:
  1) ROS Twist linear.x=+0.5  3s  -> 看车往哪走
  2) ROS Twist linear.x=-0.5  3s  -> 看车往哪走
  3) ROS Twist angular.z=+1.5 3s  -> 看 yaw 往哪转
同时订阅 gz 端 /model/robot/cmd_vel, 打印桥接后的实际数值。
"""
import math
import subprocess
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import gz.transport13 as gt
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist as GzTwist

st = {}
gzcmd = {}
gn = gt.Node()


def cb(m):
    for p in m.pose:
        if p.name == 'robot':
            q = p.orientation
            st['robot'] = (p.position.x, p.position.y,
                           math.degrees(math.atan2(
                               2 * (q.w * q.z + q.x * q.y),
                               1 - 2 * (q.y * q.y + q.z * q.z))))


def cbcmd(m):
    gzcmd['v'] = m.linear.x
    gzcmd['w'] = m.angular.z


gn.subscribe(Pose_V, '/world/follow_world/dynamic_pose/info', cb)
gn.subscribe(GzTwist, '/model/robot/cmd_vel', cbcmd)


class R(Node):
    def __init__(self):
        super().__init__('p60_dir_ros')
        self.pub = self.create_publisher(Twist, '/robot/cmd_vel', 10)


def main():
    # 杀掉 follower(否则它 20Hz 发零速覆盖), 但**保留** ros_gz_bridge
    subprocess.run(['pkill', '-f', 'vla_follo[w]/lib'], capture_output=True)
    subprocess.run(['pkill', '-f', 'person_wal[k]'], capture_output=True)
    time.sleep(1.2)

    rclpy.init()
    node = R()
    time.sleep(1.5)

    def drive(v, w, secs, tag):
        t = Twist()
        t.linear.x = v
        t.angular.z = w
        p0 = st.get('robot')
        t0 = time.time()
        while time.time() - t0 < secs:
            node.pub.publish(t)
            rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(0.08)
        z = Twist()
        for _ in range(8):
            node.pub.publish(z)
            rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(0.05)
        time.sleep(0.6)
        p = st.get('robot')
        if p and p0:
            print('  >>> %-16s Δpos=(%+.3f,%+.3f) Δyaw=%+.1f  gz收到 v=%s w=%s'
                  % (tag, p[0] - p0[0], p[1] - p0[1], p[2] - p0[2],
                     gzcmd.get('v'), gzcmd.get('w')), flush=True)
        return p

    print('初始: robot=%s' % (st.get('robot'),))
    print()
    drive(0.5, 0.0, 3.0, 'ROS x=+0.5')
    drive(-0.5, 0.0, 3.0, 'ROS x=-0.5')
    drive(0.0, 1.5, 3.0, 'ROS z=+1.5')
    print('\n结束: robot=%s' % (st.get('robot'),))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
