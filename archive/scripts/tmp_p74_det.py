#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_p74_det.py —— 记录人的真值轨迹, 用于验证评测可重复性

v2: 检测仿真时间基重置(gz 重启导致 stamp 回跳) —— 回跳超过 0.5s 就丢弃旧数据、
    以新时间基重启窗口, 并在日志打 [RESET]。上一版因此把两轮数据混在一起。
按 /person/odom 的 header.stamp(仿真时间)对齐。
"""
import argparse
import json
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class Rec(Node):
    def __init__(self, dur):
        super().__init__('p74_det_rec')
        self.dur = dur
        self.t0 = None
        self.last = None
        self.recs = []
        self.resets = 0
        self.done = False
        self.create_subscription(Odometry, '/person/odom', self._cb, 30)

    def _cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if st <= 0.0:
            return
        if self.last is not None and st < self.last - 0.5:
            print('[RESET] stamp %.2f -> %.2f, 丢弃旧窗口 %d 帧'
                  % (self.last, st, len(self.recs)), flush=True)
            self.recs = []
            self.t0 = None
            self.resets += 1
        self.last = st
        if self.t0 is None:
            self.t0 = st
            print('[START] t0=%.3f' % st, flush=True)
        self.recs.append([round(st - self.t0, 4), round(p.x, 4),
                          round(p.y, 4), round(yaw, 4)])
        if self.recs[-1][0] >= self.dur:
            self.done = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dur', type=float, default=120.0)
    ap.add_argument('--out', default='det.json')
    a = ap.parse_args()
    rclpy.init()
    n = Rec(a.dur)
    print('[WAIT] 等 /person/odom', flush=True)
    while rclpy.ok() and n.t0 is None:
        rclpy.spin_once(n, timeout_sec=0.2)
    print('[REC] 开始记录', flush=True)
    while rclpy.ok() and not n.done:
        rclpy.spin_once(n, timeout_sec=0.2)
    d = {'n': len(n.recs), 'dur': n.recs[-1][0] if n.recs else 0.0,
         'resets': n.resets, 'recs': n.recs}
    with open(a.out, 'w') as f:
        json.dump(d, f)
    print('wrote %s  n=%d  dur=%.1fs  resets=%d'
          % (a.out, d['n'], d['dur'], d['resets']), flush=True)
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
