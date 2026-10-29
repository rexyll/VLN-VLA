#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_calib_dist.py —— 视觉测距 vs 真值距离 标定

机器人固定在 (0,0) 朝 +x, 人依次放到 +x 方向 1.0~5.0m, 读 /perception/person。
输出 视觉距离 dist_vis 与真值(模型中心距)的偏差, 用来给 follower 做补偿。

注意: 运行前请先 pkill person_walk, 否则人的位姿会被它改掉。
"""
import json
import math
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

WORLD = 'follow_world'
POS = [1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 5.0]


def set_pose(name, x, y, z, yaw=0.0):
    req = ('name: "%s", position: {x: %s, y: %s, z: %s}, '
           'orientation: {z: %s, w: %s}'
           % (name, x, y, z, math.sin(yaw / 2), math.cos(yaw / 2)))
    return subprocess.run(
        ['gz', 'service', '-s', '/world/%s/set_pose' % WORLD,
         '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
         '--timeout', '3000', '--req', req],
        capture_output=True, text=True).stdout.strip()


class Calib(Node):

    def __init__(self):
        super().__init__('calib_dist')
        self.vis = {}
        self.create_subscription(String, '/perception/person', self._vb, 10)

    def _vb(self, m):
        try:
            self.vis = json.loads(m.data)
        except Exception:
            pass

    def read(self, sec=2.5):
        """采 sec 秒, 返回检出帧的 dist/bearing 中位数"""
        t0 = time.monotonic()
        ds, bs, cams = [], [], []
        while time.monotonic() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.05)
            v = self.vis or {}
            if v.get('detected') and float(v.get('dist', -1)) > 0:
                ds.append(float(v['dist']))
                bs.append(math.degrees(float(v['bearing'])))
                cams.append(v.get('cam', ''))
        if not ds:
            return None
        ds.sort()
        bs.sort()
        return (ds[len(ds) // 2], bs[len(bs) // 2],
                max(set(cams), key=cams.count), len(ds))


def main():
    print('[CALIB] 机器人固定到 (0,0) 朝 +x', flush=True)
    print(set_pose('robot', 0.0, 0.0, 0.15, 0.0), flush=True)
    rclpy.init()
    n = Calib()
    time.sleep(1.0)
    rows = []
    print('%6s %8s %8s %8s %6s %6s' %
          ('真值d', '视觉d', '偏差', '方位°/视', 'cam', '检出帧'), flush=True)
    for x in POS:
        set_pose('person', x, 0.0, 0.02, math.pi)
        time.sleep(1.2)
        r = n.read(2.5)
        if r is None:
            print('%6.2f %8s %8s %8s %6s %6d' % (x, '-', '-', '-', '-', 0),
                  flush=True)
            rows.append((x, None, None, None))
            continue
        dv, bg, cam, cnt = r
        print('%6.2f %8.2f %+8.2f %+8.1f %6s %6d'
              % (x, dv, dv - x, bg, cam, cnt), flush=True)
        rows.append((x, dv, dv - x, bg))
    ok = [(t, d, e) for (t, d, e, _) in rows if d is not None]
    if ok:
        err = sorted(e for _, _, e in ok)
        print('\n偏差(视觉-真值): n=%d 中位=%+.3f 范围[%+.3f,%+.3f]'
              % (len(err), err[len(err) // 2], err[0], err[-1]), flush=True)
        # 只统计 1.0~4.0m(实际跟随工作区)的偏差
        near = sorted(e for t, _, e in ok if 1.0 <= t <= 4.0)
        if near:
            print('工作区 1~4m 偏差中位=%+.3f' % near[len(near) // 2],
                  flush=True)
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
