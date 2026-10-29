#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_repro_ob4.py —— 复现"机器人撞蓝色圆柱 ob4"并定位是哪一层失守

场景: ob4 是 (-4.2,-2.5) r0.50 的蓝色圆柱。
把"人"固定在 ob4 正后方, 机器人从 ob4 正前方 3m 处跟随 → 会直冲 ob4。

同时记录每一帧:
  - 机器人到 ob4 表面的真实间隙 (负=已进入圆柱)
  - /perception/proximity 的 8 扇区(深度感知层看到了什么)
  - /follower/status 的 track/src/lost/cmd_v (决策层在干什么)
  - 从 follower 日志里捞"全向急停/全向限速"标记(安全层有没有生效)

用法:
  python3 tmp_repro_ob4.py --dur 40
"""
import argparse
import json
import math
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import gz.transport13 as gz_transport
from gz.msgs10.pose_v_pb2 import Pose_V as GzPose_V

WORLD = 'follow_world'
ROBOT_R = 0.18
OB4 = (-4.2, -2.5, 0.50)      # x, y, r
PERSON = (-5.55, -2.5)        # ob4 正后方(远侧)


def set_pose(name, x, y, z, yaw=0.0):
    req = ('name: "%s", position: {x: %s, y: %s, z: %s}, '
           'orientation: {z: %s, w: %s}'
           % (name, x, y, z, math.sin(yaw / 2), math.cos(yaw / 2)))
    for _ in range(3):
        r = subprocess.run(
            ['gz', 'service', '-s', '/world/%s/set_pose' % WORLD,
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '3000', '--req', req],
            capture_output=True, text=True)
        if 'true' in (r.stdout or '').lower():
            return True
        time.sleep(0.3)
    return False


def gap_ob4(x, y):
    return math.hypot(x - OB4[0], y - OB4[1]) - OB4[2] - ROBOT_R


class Repro(Node):
    def __init__(self, dur):
        super().__init__('repro_ob4')
        self.dur = dur
        self.t0 = time.monotonic()
        self.done = False
        self.rec = []
        self.rx = self.ry = None
        self.prox = {}
        self.st = {}
        self.pub_cmd = self.create_publisher(String, '/robot/command', 10)
        self._gz = gz_transport.Node()
        self._gz.subscribe(GzPose_V, '/world/%s/dynamic_pose/info' % WORLD,
                           self._gt)
        self.create_subscription(String, '/perception/proximity', self._p, 10)
        self.create_subscription(String, '/follower/status', self._s, 10)
        self.hold_n = 0
        self.create_timer(0.05, self._tick)
        self.create_timer(0.20, self._hold_person)

    def _gt(self, msg):
        for p in msg.pose:
            if p.name == 'robot':
                self.rx, self.ry = p.position.x, p.position.y

    def _p(self, m):
        try:
            self.prox = json.loads(m.data)
        except Exception:
            pass

    def _s(self, m):
        try:
            self.st = json.loads(m.data)
        except Exception:
            pass

    def _hold_person(self):
        # person_walk 已停, 这里每 0.2s 把"人"钉在原地当静止目标
        set_pose('person', PERSON[0], PERSON[1], 0.02, math.pi)
        self.hold_n += 1

    def _tick(self):
        t = time.monotonic() - self.t0
        if t > self.dur:
            self.done = True
            return
        if self.rx is None:
            return
        pr = self.prox or {}
        self.rec.append({
            't': round(t, 2), 'rx': round(self.rx, 3), 'ry': round(self.ry, 3),
            'gap': round(gap_ob4(self.rx, self.ry), 3),
            'front': round(float(pr.get('front', -1) or -1), 2),
            'fl': round(float(pr.get('fl', -1) or -1), 2),
            'fr': round(float(pr.get('fr', -1) or -1), 2),
            'track': self.st.get('track', ''), 'src': self.st.get('src', ''),
            'cmd_v': self.st.get('cmd_v', 0.0),
        })


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dur', type=float, default=40.0)
    a = ap.parse_args()

    print('[REPRO] 停 person_walk(改由脚本钉住"人")', flush=True)
    subprocess.run(['pkill', '-f', 'person_wal[k]'], capture_output=True)
    time.sleep(1.0)

    print('[REPRO] 人 -> (%.2f,%.2f) 机器人 -> (-1.20,-2.50) 朝 -x'
          % PERSON, flush=True)
    set_pose('person', PERSON[0], PERSON[1], 0.02, math.pi)
    set_pose('robot', -1.20, -2.50, 0.15, math.pi)

    rclpy.init()
    n = Repro(a.dur)
    t = time.monotonic() + 2.0
    while time.monotonic() < t:
        rclpy.spin_once(n, timeout_sec=0.1)
    subprocess.run(['bash', '/home/earl/workspace/VLA/tmp_follow.sh'],
                   capture_output=True, text=True, timeout=30)
    print('[REPRO] FOLLOW 已下发', flush=True)
    n.t0 = time.monotonic()

    while rclpy.ok() and not n.done:
        rclpy.spin_once(n, timeout_sec=0.05)

    rs = n.rec
    print('\n================ 复现结果 ================')
    if not rs:
        print('无数据')
        return
    mn = min(rs, key=lambda r: r['gap'])
    print('采样 %d 点, 出发间隙 %+.2fm' % (len(rs), rs[0]['gap']))
    print('最小间隙 %+.3fm @ t=%.1fs (x=%.2f, y=%.2f)'
          % (mn['gap'], mn['t'], mn['rx'], mn['ry']))
    print('最终位置 x=%.2f y=%.2f 间隙=%+.3f'
          % (rs[-1]['rx'], rs[-1]['ry'], rs[-1]['gap']))
    hits = [r for r in rs if r['gap'] < 0]
    print('进入圆柱的帧数: %d / %d' % (len(hits), len(rs)))
    print('\n  t     x       y      gap    front   fl     fr    trk    src   cmd_v')
    for r in rs[::10]:
        print(' %5.1f %6.2f %6.2f %+6.2f %6.2f %6.2f %6.2f  %-5s %-5s %5.2f'
              % (r['t'], r['rx'], r['ry'], r['gap'], r['front'], r['fl'],
                 r['fr'], r['track'] or '-', r['src'] or '-', r['cmd_v']))
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
