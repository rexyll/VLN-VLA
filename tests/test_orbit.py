#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_orbit_test.py —— "绕柱跟人"受控复现

人绕 ob1 立柱(3.5,2.0) 以 R 匀速转圈, 机器人从 1m 外跟随。
每圈有 1/3 左右的行程人被动柱挡住 —— 这就是"跟到障碍物附近丢目标"
的最小可复现场景。记录检测状态 + 遮挡几何 + 状态机 + 重捕获延迟。

用法: python3 tests/test_orbit.py <tag> [dur=90] [R=1.0] [spd=0.35]
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
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import gz.transport13 as gz_transport
from gz.msgs10.pose_v_pb2 import Pose_V as GzPose_V

WORLD = 'follow_world'
OX, OY = 3.5, 2.0          # ob1 中心
HB = 0.4                   # ob1 半宽(0.8m 立方)
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


class Orbit(Node):
    def __init__(self, tag, dur, R, spd):
        super().__init__('orbit_test')
        self.dur, self.R, self.spd = dur, R, spd
        self.f = open(os.path.join(OUTDIR, 'orbit_%s.jsonl' % tag),
                      'w', encoding='utf-8')
        self.rp = self.pp = None
        self.st = {}
        self.t0 = time.monotonic()
        self.pub = self.create_publisher(Twist, '/person/cmd_vel', 10)
        self.pub_cmd = self.create_publisher(String, '/robot/command', 10)
        self._gz = gz_transport.Node()
        self._gz.subscribe(GzPose_V, '/world/%s/dynamic_pose/info' % WORLD,
                           self._gt)
        self.create_subscription(String, '/follower/status', self._s, 10)
        self.create_timer(0.05, self._tick)
        print('ORBIT 就绪 tag=%s R=%.2f spd=%.2f' % (tag, R, spd), flush=True)

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
        if self.pp is None or self.rp is None:
            return
        px, py = self.pp
        ang = math.atan2(py - OY, px - OX)
        # 纯追踪: 圆上前方 0.5m 弧长处的点
        ang_t = ang + 0.5 / self.R
        tx = OX + self.R * math.cos(ang_t)
        ty = OY + self.R * math.sin(ang_t)
        goal = math.atan2(ty - py, tx - px)
        err = wrap(goal - (ang + math.pi / 2.0))   # 车头朝向: 切线
        # 用切向速度 + 径向修正代替车头朝向(人对模型是差速, 但这里直接给
        # 世界系速度更稳: 把世界速度转成 body 系)
        vx = -math.sin(ang) * self.spd
        vy = math.cos(ang) * self.spd
        # 径向修正: 当前半径偏大/偏小
        r_now = math.hypot(px - OX, py - OY) or 1e-6
        er = self.R - r_now
        ux, uy = (px - OX) / r_now, (py - OY) / r_now
        vx += ux * 2.0 * er
        vy += uy * 2.0 * er
        # 假设人车头朝着运动方向: 用位置差分得到的实际朝向更可靠
        heading = getattr(self, '_ph', ang + math.pi / 2.0)
        c, s = math.cos(-heading), math.sin(-heading)
        vb = vx * c - vy * s
        w = 1.2 * err
        tw = Twist()
        tw.linear.x = vb
        tw.angular.z = w
        self.pub.publish(tw)
        if not hasattr(self, '_plast'):
            self._plast = (px, py)
        dx, dy = px - self._plast[0], py - self._plast[1]
        if math.hypot(dx, dy) > 0.02:
            self._ph = math.atan2(dy, dx)
        self._plast = (px, py)
        # 采样
        if int((time.monotonic() - self.t0) * 20) % 2 == 0:
            rx, ry, ryaw = self.rp
            d = math.hypot(px - rx, py - ry)
            brg = wrap(math.atan2(py - ry, px - rx) - ryaw)
            occl = seg_box(rx, ry, px, py, OX, OY, HB)
            self.f.write(json.dumps({
                't': round(time.monotonic() - self.t0, 2),
                'd': round(d, 3), 'brg': round(math.degrees(brg), 1),
                'det': bool(self.st.get('detected')),
                'cam': self.st.get('cam', ''),
                'vd': self.st.get('vis_dist', -1.0),
                'trk': self.st.get('track', ''),
                'occl': bool(occl),
                'in_fov': abs(brg) < HFOV / 2.0,
                'ang': round(math.degrees(wrap(ang)), 0),
                'pd': round(math.hypot(px - OX, py - OY) - HB - 0.26, 3),
                'px': round(px, 2), 'py': round(py, 2),
                'rx': round(rx, 2), 'ry': round(ry, 2)}, ensure_ascii=False)
                + '\n')
            self.f.flush()


def main():
    tag = sys.argv[1] if len(sys.argv) > 1 else 'o'
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 90.0
    R = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
    spd = float(sys.argv[4]) if len(sys.argv) > 4 else 0.35
    rclpy.init()
    n = Orbit(tag, dur, R, spd)
    # 摆位: 人在圆上 ang=0, 机器人在其正后方 1.0m 朝 -x
    set_pose('person', OX + R, OY, 0.02, math.pi / 2)
    set_pose('robot', OX + R + 1.0, OY, 0.15, math.pi)
    time.sleep(1.0)
    for _ in range(3):
        m = String()
        m.data = json.dumps({'action': 'FOLLOW', 'distance': 1.0})
        n.pub_cmd.publish(m)
        time.sleep(0.4)
    t0 = time.monotonic()
    while time.monotonic() - t0 < dur + 2.0:
        rclpy.spin_once(n, timeout_sec=0.02)
    n.f.close()
    n.pub.publish(Twist())
    n.destroy_node()
    rclpy.try_shutdown()
    print('ORBIT 完成', flush=True)


if __name__ == '__main__':
    main()
