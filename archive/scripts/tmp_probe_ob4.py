#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_probe_ob4.py —— 定点探测某个障碍是否被深度感知层看到

用法:
  python3 tmp_probe_ob4.py --x -2.2 --y -2.5 --yaw 3.14159 --tag ob4

把机器人瞬移到 (x,y) 朝 yaw, 读 /perception/proximity 的 8 扇区距离,
同时按 depth_percept 的取带规则直接复算前方深度相机的最小距离, 交叉验证。
"""
import argparse
import json
import math
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

WORLD = 'follow_world'
D_MIN, D_MAX = 0.30, 11.5
# depth_percept 的取带: 仰角 +1.5° ~ +18°
r0f, r1f = 0.5 - 18.0 / 83.58, 0.5 - 1.5 / 83.58


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


class Probe(Node):
    def __init__(self):
        super().__init__('probe')
        self.prox = {}
        self.depth = {}
        self.create_subscription(String, '/perception/proximity', self._p, 10)
        for d in ('front', 'left', 'back', 'right'):
            self.create_subscription(
                Image, '/depth_%s/image_depth' % d,
                lambda m, dd=d: self._d(m, dd), 5)

    def _p(self, m):
        try:
            self.prox = json.loads(m.data)
        except Exception:
            pass

    def _d(self, m, name):
        h, w = m.height, m.width
        if h < 10 or len(m.data) < h * w * 4:
            return
        a = np.frombuffer(m.data[:h * w * 4], dtype='<f4').reshape(h, w)
        r0, r1 = max(0, int(h * r0f)), min(h, int(h * r1f) + 1)
        b = a[r0:r1, :]
        ok = np.isfinite(b) & (b > D_MIN) & (b < D_MAX)
        if ok.sum() < 60:
            self.depth[name] = ('无效帧', int(ok.sum()))
            return
        v = b[ok]
        cols = np.where(ok.any(axis=0))[0]
        self.depth[name] = (round(float(np.quantile(v, 0.05)), 3),
                            '%d~%d 列' % (cols.min(), cols.max()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--x', type=float, required=True)
    ap.add_argument('--y', type=float, required=True)
    ap.add_argument('--yaw', type=float, default=0.0)
    ap.add_argument('--tag', default='probe')
    ap.add_argument('--settle', type=float, default=3.0)
    a = ap.parse_args()

    print('[PROBE] set_pose robot=(%.2f,%.2f,%.0f°) -> %s'
          % (a.x, a.y, math.degrees(a.yaw), set_pose('robot', a.x, a.y, 0.15, a.yaw)),
          flush=True)
    rclpy.init()
    n = Probe()
    t_end = time.monotonic() + a.settle
    while time.monotonic() < t_end:
        rclpy.spin_once(n, timeout_sec=0.1)
    print('\n=== %s  8 扇区(深度感知层输出) ===' % a.tag)
    for k in sorted(n.prox.keys()):
        print('  %-8s %s' % (k, n.prox[k]))
    print('\n=== 相机原始深度(带内 5% 分位 / 有效列范围) ===')
    for k in ('front', 'left', 'back', 'right'):
        print('  %-6s %s' % (k, n.depth.get(k)))
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
