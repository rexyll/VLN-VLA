#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""p84_watch.py —— 记录真值位姿轨迹, 定位"物件出生即被掀飞"

为什么需要它:
    P84 阶段水壶/水杯一启动就被弹到几米外(壶 -6.1m, 杯 +2.6m)。肉眼只能看到
    "结果", 看不到"哪一帧开始动、往哪个方向动、加速度多大"。直接看轨迹就能
    区分三类原因:
      a) 与桌面/相邻物件初始穿透 → 第一帧就有巨大位移, 方向沿接触法向(向上/外)
      b) 被别的物体撞飞           → 位移起点滞后几十毫秒, 方向沿撞击方向
      c) 自身关节(jet/water 滑动副)发散 → 容器本体位移小但抖动, 姿态剧烈
    所以本脚本按 50Hz 采样, 输出"首次位移 > 阈值"的时刻与随后 1s 的轨迹。

用法:
    python3 tests/diag/p84_watch.py [秒数] [--ent a,b,c] [--all]
默认采样 /world/follow_world/dynamic_pose/info, 关注
    kettle / cup / table / person / robot / chassis
"""
import math
import sys
import time

import gz.transport13 as gz_transport
from gz.msgs10.pose_v_pb2 import Pose_V

TOPIC = '/world/follow_world/dynamic_pose/info'
DUR = 20.0
WANT = ['kettle', 'cup', 'table', 'person', 'robot', 'chassis',
        'R_palm', 'L_palm']

args = sys.argv[1:]
if args and not args[0].startswith('--'):
    DUR = float(args[0])
if '--ent' in args:
    WANT = args[args.index('--ent') + 1].split(',')

# name -> [(t_rel, x, y, z)]
traj = {}
t0 = [None]
nmsg = [0]


def cb(msg):
    now = time.time()
    if t0[0] is None:
        t0[0] = now
    tr = now - t0[0]
    nmsg[0] += 1
    for p in msg.pose:
        n = p.name
        if n in WANT:
            traj.setdefault(n, []).append(
                (tr, p.position.x, p.position.y, p.position.z))


def fmt(v):
    return '(%9.4f, %9.4f, %9.4f)' % v


def main():
    node = gz_transport.Node()
    ok = node.subscribe(Pose_V, TOPIC, cb)
    print('subscribe -> %s, 采样 %.0fs' % (ok, DUR))
    while t0[0] is None and True:
        time.sleep(0.05)
        if time.time() > time.time():
            break
    tend = (t0[0] or time.time()) + DUR
    while time.time() < tend:
        time.sleep(0.1)

    print('收到 %d 条 pose_v' % nmsg[0])
    for name in WANT:
        seq = traj.get(name)
        if not seq:
            print('\n%-10s 无数据' % name)
            continue
        p0 = seq[0]
        pl = seq[-1]
        d = math.sqrt(sum((pl[i + 1] - p0[i + 1]) ** 2 for i in range(3)))
        # 首次超过 5mm 的时刻
        tmove = None
        pmove = None
        for s in seq:
            dd = math.sqrt(sum((s[i + 1] - p0[i + 1]) ** 2 for i in range(3)))
            if dd > 0.005:
                tmove, pmove = s[0], s
                break
        # 本段内的最大单步速度 (m/s), 用相邻样本差分
        vmax = 0.0
        for a, b in zip(seq, seq[1:]):
            dt = b[0] - a[0]
            if dt <= 1e-6:
                continue
            v = math.sqrt(sum((b[i] - a[i]) ** 2 for i in range(1, 4))) / dt
            vmax = max(vmax, v)
        print('\n%-10s 样本 %4d' % (name, len(seq)))
        print('   首帧 %s' % fmt(p0[1:]))
        print('   末帧 %s   漂移 %.1f mm   峰值速度 %.2f m/s'
              % (fmt(pl[1:]), d * 1000.0, vmax))
        if tmove is None:
            print('   全程位移 < 5mm  ✅ 稳定')
        else:
            print('   ⚠ 首次超 5mm: t=%.3fs  %s' % (tmove, fmt(pmove[1:])))
            print('   --- 之后 12 帧 ---')
            i0 = seq.index(pmove)
            for s in seq[i0:i0 + 12]:
                print('     t=%6.3f  %s' % (s[0], fmt(s[1:])))
    return 0


if __name__ == '__main__':
    sys.exit(main())
