#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_hide_test.py —— "人躲到柱子后面"强制遮挡场景

摆位: 机器人 (5.40,2.05) 朝 -x; 人先在柱侧可见处 (4.50,2.30);
     t=6s 时把人瞬移到柱子正后方 (2.55,2.00) 并静止 —— 机器人视线被
     ob1(3.5,2.0, 0.8x0.8) 完全挡住, 只能靠绕视(PEEK)把视线绕出来。

用法: python3 tests/test_hide.py <tag> [dur=30]
"""
import json
import math
import os

# 项目根: 产物统一输出到 tests/out/, 不再散落在项目根
VLA_HOME = os.environ.get('VLA_HOME', '/home/earl/workspace/VLA')
OUTDIR = os.path.join(VLA_HOME, 'tests', 'out')
os.makedirs(OUTDIR, exist_ok=True)
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import gz.transport13 as gz_transport
from gz.msgs10.pose_v_pb2 import Pose_V as GzPose_V

WORLD = 'follow_world'
OX, OY, HB = 3.5, 2.0, 0.4
RX, RY = 5.40, 2.05
P0 = (4.50, 2.30)          # 可见起点
P1 = (2.55, 2.00)          # 柱后隐藏点
HFOV = 1.8


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def seg_box(x0, y0, x1, y1, cx, cy, h):
    dx, dy = x1 - x0, y1 - y0
    t0, t1 = 0.0, 1.0
    for p, d in ((x0 - cx, dx), (y0 - cy, dy)):
        if abs(d) < 1e-9:
            if p < -h or p > h:
                return False
        else:
            ta, tb = (-h - p) / d, (h - p) / d
            if ta > tb:
                ta, tb = tb, ta
            t0, t1 = max(t0, ta), min(t1, tb)
            if t0 > t1:
                return False
    return True


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


class Hide(Node):
    def __init__(self, tag, dur):
        super().__init__('hide_test')
        self.dur = dur
        self.f = open(os.path.join(OUTDIR, 'hide_%s.jsonl' % tag),
                      'w', encoding='utf-8')
        self.rp = self.pp = None
        self.st = {}
        self.t0 = time.monotonic()
        self.pub_cmd = self.create_publisher(String, '/robot/command', 10)
        self._gz = gz_transport.Node()
        self._gz.subscribe(GzPose_V, '/world/%s/dynamic_pose/info' % WORLD,
                           self._gt)
        self.create_subscription(String, '/follower/status', self._s, 10)
        self.create_timer(0.1, self._tick)
        print('HIDE 就绪 tag=%s' % tag, flush=True)

    def _gt(self, msg):
        for p in msg.pose:
            if p.name == 'robot':
                self.rp = (p.position.x, p.position.y, yaw_of(p.orientation))
            elif p.name == 'person':
                self.pp = (p.position.x, p.position.y)

    def _s(self, m):
        try:
            self.st = json.loads(m.data)
        except Exception:
            pass

    def _tick(self):
        if self.pp is None or self.rp is None or not self.st:
            return
        px, py = self.pp
        rx, ry, ryaw = self.rp
        d = math.hypot(px - rx, py - ry)
        brg = wrap(math.atan2(py - ry, px - rx) - ryaw)
        self.f.write(json.dumps({
            't': round(time.monotonic() - self.t0, 2),
            'd': round(d, 3), 'brg': round(math.degrees(brg), 1),
            'det': bool(self.st.get('detected')),
            'cam': self.st.get('cam', ''),
            'vd': self.st.get('vis_dist', -1.0),
            'trk': self.st.get('track', ''),
            'occl': seg_box(rx, ry, px, py, OX, OY, HB),
            'rx': round(rx, 2), 'ry': round(ry, 2)}, ensure_ascii=False)
            + '\n')
        self.f.flush()


def main():
    tag = sys.argv[1] if len(sys.argv) > 1 else 'h'
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
    rclpy.init()
    n = Hide(tag, dur)
    set_pose('person', P0[0], P0[1], 0.02, 0.0)
    set_pose('robot', RX, RY, 0.15, math.pi)
    time.sleep(1.0)
    for _ in range(3):
        m = String()
        m.data = json.dumps({'action': 'FOLLOW', 'distance': 1.0})
        n.pub_cmd.publish(m)
        time.sleep(0.4)
    t0 = time.monotonic()
    hid = False
    _fix_t = 0.0
    while time.monotonic() - t0 < dur:
        rclpy.spin_once(n, timeout_sec=0.05)
        if not hid and time.monotonic() - t0 > 6.0:
            set_pose('person', P1[0], P1[1], 0.02, 0.0)
            print('HIDE: 人已躲到柱后', flush=True)
            hid = True
        # 差速插件会有残余速度把人推走 —— 每 0.8s 复位一次, 保证"人站着不动"
        if hid and time.monotonic() - _fix_t > 0.8:
            _fix_t = time.monotonic()
            set_pose('person', P1[0], P1[1], 0.02, 0.0)
    n.f.close()
    n.destroy_node()
    rclpy.try_shutdown()
    print('HIDE 完成', flush=True)


if __name__ == '__main__':
    main()
