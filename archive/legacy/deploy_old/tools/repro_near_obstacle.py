#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_p53_repro.py —— 定点复现 eval14 那次 60s 卡死。

eval14 实证的现场(truth):
  机器人 (-2.24, 2.84, yaw 39°)  人 (-1.13, 4.33)  真值距 1.86m
  车心到 ob2 (-3.0,3.2, 1.2x0.6, yaw0.5) 表面 0.38m, ob2 的角在车头
  左前 64°、0.395m。沿 39° 航向法向间隙 0.355m > 车半径 0.18 —— 过得去。
  但旧判据 d_head = 0.34*0.72 = 0.245 < stop_d 0.66 → 急停 + 强制绕右,
  与 ALIGN"转向目标"互掐: w 在 +1.1/-0.9 每 0.5s 翻一次, 60s 没动 1mm。

用法: python3 tmp_p53_repro.py [robot_x] [robot_y] [robot_yaw_deg]
                              [person_x] [person_y] [持续秒]
"""
import json
import math
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

import gz.transport13 as gz_transport
from gz.msgs10.pose_v_pb2 import Pose_V

WORLD = 'follow_world'

argv = sys.argv[1:]
RX = float(argv[0]) if len(argv) > 0 else -2.24
RY = float(argv[1]) if len(argv) > 1 else 2.84
RYAW = math.radians(float(argv[2]) if len(argv) > 2 else 25.0)
PX = float(argv[3]) if len(argv) > 3 else -1.13
PY = float(argv[4]) if len(argv) > 4 else 4.33
DUR = float(argv[5]) if len(argv) > 5 else 25.0


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


def shoot_follow():
    r = subprocess.run(['bash', '/home/earl/workspace/VLA/tmp_follow.sh'],
                       capture_output=True, text=True, timeout=30)
    return r.stdout.strip().splitlines()[-1] if r.stdout else ''


class R(Node):
    def __init__(self):
        super().__init__('p53_repro')
        self.st = {}
        self.pos = {}
        self.create_subscription(String, '/follower/status', self._st, 10)
        self._gz = gz_transport.Node()
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

    # kill 掉 person_walk 不会清零速度: gz 的 diff_drive 保持最后一条 cmd_vel
    zero = Twist()
    pub_p = node.create_publisher(Twist, '/person/cmd_vel', 10)
    for sig in ([], ['-9']):
        subprocess.run(['pkill'] + sig + ['-f', 'person_wal[k]'],
                       capture_output=True)
        time.sleep(0.8)
    for _ in range(8):
        pub_p.publish(zero)
        rclpy.spin_once(node, timeout_sec=0.08)
        time.sleep(0.05)

    set_pose('robot', RX, RY, 0.15, RYAW)
    set_pose('person', PX, PY, 0.02, math.pi)
    time.sleep(1.5)
    print('[REPRO] 机器人(%.2f,%.2f,%.0f°) 人(%.2f,%.2f) 真值距 %.2fm'
          % (RX, RY, math.degrees(RYAW), PX, PY,
             math.hypot(PX - RX, PY - RY)), flush=True)

    for _ in range(3):
        print('[REPRO] 下指令:', shoot_follow(), flush=True)
        time.sleep(0.6)
        rclpy.spin_once(node, timeout_sec=0.1)

    time.sleep(0.3)
    t0 = time.monotonic()
    print('\n%6s %7s %7s %7s %6s %6s %6s %5s %s'
          % ('t', 'robot_x', 'robot_y', 'dist', 'cmd_v', 'cmd_w',
             'vis_d', 'mode', '备注'))
    print('-' * 96, flush=True)
    last = 0.0
    moved = 0.0
    p0 = None
    _pin_t = 0.0
    dmin = 99.0
    while time.monotonic() - t0 < DUR:
        rclpy.spin_once(node, timeout_sec=0.05)
        t = time.monotonic() - t0
        if t - _pin_t > 0.4:            # 钉住人, 否则 gz 里刚体会缓慢漂
            _pin_t = t
            set_pose('person', PX, PY, 0.02, math.pi)
            pub_p.publish(zero)
        if t - last < 1.0:
            continue
        last = t
        rp = node.pos.get('robot')
        pp = node.pos.get('person')
        if not rp or not pp:
            continue
        if p0 is None:
            p0 = rp
        moved = math.hypot(rp[0] - p0[0], rp[1] - p0[1])
        d = math.hypot(pp[0] - rp[0], pp[1] - rp[1])
        dmin = min(dmin, d)
        st = node.st
        print('%6.1f %7.2f %7.2f %7.2f %6.2f %6.2f %6s %5s %s'
              % (t, rp[0], rp[1], d, st.get('cmd_v', 0.0),
                 st.get('cmd_w', 0.0),
                 ('%.2f' % st.get('vis_dist', -1)) if st.get('detected') else '-',
                 st.get('track', '?'), st.get('note', '') or st.get('src', '')),
              flush=True)

    print('\n[REPRO] 累计位移 %.2fm  最近接近 %.2fm' % (moved, dmin), flush=True)
    print('[REPRO] 结论: %s'
          % ('不再卡死 → P53 生效' if moved > 0.5 and dmin < 1.35
             else '仍然卡死 → 另有约束'), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
