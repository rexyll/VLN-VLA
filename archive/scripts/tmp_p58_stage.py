#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P58 定点场景测试 —— 人钉住不动, 机器人从任意位形出发。

用途: 逐条验证需求 1/2/5/6 的行为, 不受 person_walk 随机性的干扰。

用法:
  python3 tmp_p58_stage.py --rx -2.6 --ry 0 --ryaw 0 --px 1.0 --py 0 \
      --pd 3.14 --dur 40 [--pin 1] [--tag 场景名]

输出: 每 0.5s 一行 (t, 真值距离, 指令v/w, 基元, note), 末尾汇总。
"""
import argparse
import json
import math
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import gz.transport13 as gt
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist as GzTwist

WORLD = 'follow_world'

ap = argparse.ArgumentParser()
ap.add_argument('--rx', type=float, default=-3.0)
ap.add_argument('--ry', type=float, default=0.0)
ap.add_argument('--ryaw', type=float, default=0.0)
ap.add_argument('--px', type=float, default=1.0)
ap.add_argument('--py', type=float, default=0.0)
ap.add_argument('--pd', type=float, default=math.pi)
ap.add_argument('--dur', type=float, default=40.0)
ap.add_argument('--pin', type=int, default=1, help='1=钉住人 0=让人自己走')
ap.add_argument('--tag', default='场景')
A = ap.parse_args()


def set_pose(name, x, y, z, yaw=0.0):
    req = ('name: "%s", position: {x: %s, y: %s, z: %s}, '
           'orientation: {z: %s, w: %s}'
           % (name, x, y, z, math.sin(yaw / 2), math.cos(yaw / 2)))
    for _ in range(3):
        r = subprocess.run(
            ['gz', 'service', '-s', '/world/%s/set_pose' % WORLD,
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '2000', '--req', req],
            capture_output=True, text=True)
        if 'true' in (r.stdout or '').lower():
            return True
        time.sleep(0.2)
    return False


class R(Node):

    def __init__(self):
        super().__init__('p58_stage')
        self.st = {}
        self.pos = {}
        self.create_subscription(String, '/follower/status', self._st, 10)
        self._gz = gt.Node()
        self._gz.subscribe(Pose_V, '/world/%s/dynamic_pose/info' % WORLD,
                           self._pose)

    def _st(self, m):
        try:
            self.st = json.loads(m.data)
        except Exception:
            pass

    def _pose(self, m):
        for p in m.pose:
            if p.name in ('robot', 'person'):
                self.pos[p.name] = (p.position.x, p.position.y)


def main():
    rclpy.init()
    node = R()
    time.sleep(0.8)

    # 零速走 gz 侧的 /model/person/cmd_vel。原来用 rclpy publisher 发
    # /person/cmd_vel, 跑到第 3 个 tick 就报 "publisher's context is
    # invalid", 人于是继续按最后一条 cmd_vel 漂移(0.26m/s), 距离越测越大。
    gzp = gt.Node()
    _pubp = gzp.advertise('/model/person/cmd_vel', GzTwist)
    zero = GzTwist()

    def pub_zero():
        _pubp.publish(zero)

    # P60: 杀 follower ≠ 停转 —— gz diff_drive 保持最后一条 cmd_vel,
    # 上次测试残留的 w=±1.5 会让机器人在 set_pose 之后继续原地旋转,
    # 视觉自然乱报。先 STOP(follower 会发零速), 再从 gz 侧再清一次,
    # 确认 /robot/cmd_vel 真的为 0 之后才摆位形。
    pub = node.create_publisher(String, '/robot/command', 10)
    stop = String()
    stop.data = json.dumps({'action': 'STOP', 'text': 'stop'})
    for _ in range(3):
        pub.publish(stop)
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.15)
    _gzr = gt.Node()
    _pubr = _gzr.advertise('/model/robot/cmd_vel', GzTwist)
    for _ in range(10):
        _pubr.publish(zero)
        time.sleep(0.05)

    if A.pin:
        for sig in ([], ['-9']):
            subprocess.run(['pkill'] + sig + ['-f', 'person_wal[k]'],
                           capture_output=True)
            time.sleep(0.8)
        for _ in range(10):
            pub_zero()
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)

    set_pose('robot', A.rx, A.ry, 0.15, A.ryaw)
    set_pose('person', A.px, A.py, 0.02, A.pd)
    time.sleep(1.2)
    print('[%s] 机器人(%.2f,%.2f,%.0f°) 人(%.2f,%.2f) 真值距 %.2fm  pin=%d'
          % (A.tag, A.rx, A.ry, math.degrees(A.ryaw), A.px, A.py,
             math.hypot(A.px - A.rx, A.py - A.ry), A.pin), flush=True)

    cmd = String()
    cmd.data = json.dumps({'action': 'FOLLOW', 'distance': 1.0,
                           'text': 'follow'})
    for _ in range(3):
        pub.publish(cmd)
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.15)

    t0 = time.monotonic()
    t_pin = 0.0
    last = 0.0
    hist = []
    while time.monotonic() - t0 < A.dur:
        rclpy.spin_once(node, timeout_sec=0.05)
        t = time.monotonic() - t0
        if A.pin and t - t_pin > 0.5:
            t_pin = t
            set_pose('person', A.px, A.py, 0.02, A.pd)
            pub_zero()
        if t - last > 0.5:
            last = t
            st = node.st
            rp = node.pos.get('robot')
            pp = node.pos.get('person')
            d = (math.hypot(pp[0] - rp[0], pp[1] - rp[1])
                 if rp and pp else -1.0)
            hist.append(d)
            print('%5.1fs d=%.2f v=%+.2f w=%+.2f prim=%-5s cam=%-5s '
                  'plan{%s} | %s'
                  % (t, d, st.get('cmd_v', 0.0), st.get('cmd_w', 0.0),
                     st.get('prim', ''), st.get('cam', '-'),
                     (lambda p: '%d点/%.2f/%s' % (p['n'], p['lv'],
                                                  '到' if p['reached'] else '近')
                      if p else '-')(st.get('plan')),
                     (st.get('note') or '')[:46]), flush=True)
        if node.st.get('track') == 'SCAN':
            pass

    if hist:
        good = [x for x in hist if 0.85 <= x <= 1.15]
        print('\n[%s] 采样 %d: 带内 %.0f%% 中位 %.2fm 最小 %.2fm 最大 %.2fm'
              % (A.tag, len(hist), 100.0 * len(good) / len(hist),
                 sorted(hist)[len(hist) // 2], min(hist), max(hist)),
              flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
