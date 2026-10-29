#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""让"人"沿一条平滑圆周行走（用于跟随效果演示）。

直接用 gz 的 set_pose 服务按 10Hz 插值移动目标，避免 person_walk 的随机
转向把人带到墙边或柱子后面，导致演示被绕障/搜索逻辑打断。
"""
import argparse
import math
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--r', type=float, default=2.0, help='圆周半径 m')
    ap.add_argument('--w', type=float, default=0.18, help='角速度 rad/s')
    ap.add_argument('--cx', type=float, default=0.0)
    ap.add_argument('--cy', type=float, default=0.2)
    ap.add_argument('--dur', type=float, default=110.0)
    ap.add_argument('--theta0', type=float, default=-1.5708)
    ap.add_argument('--world', default='follow_world')
    a = ap.parse_args()

    node = Node()
    svc = '/world/%s/set_pose' % a.world
    t0 = time.monotonic()
    step = 0
    while True:
        t = time.monotonic() - t0
        if t > a.dur:
            break
        th = a.theta0 + a.w * t
        x = a.cx + a.r * math.cos(th)
        y = a.cy + a.r * math.sin(th)
        yaw = th + math.pi / 2          # 朝切线方向前进
        req = Pose()
        req.name = 'person'
        req.position.x = x
        req.position.y = y
        req.position.z = 0.02
        req.orientation.z = math.sin(yaw / 2)
        req.orientation.w = math.cos(yaw / 2)
        node.request(svc, req, Pose, Boolean, 300)
        step += 1
        if step % 50 == 0:
            print('[CIRCLE] t=%5.1fs 人=(%+.2f,%+.2f) 朝向=%+.0f° 线速度=%.2fm/s'
                  % (t, x, y, math.degrees(yaw), a.w * a.r), flush=True)
        time.sleep(0.1)
    print('[CIRCLE] 结束, 共 %d 步' % step, flush=True)


if __name__ == '__main__':
    main()
