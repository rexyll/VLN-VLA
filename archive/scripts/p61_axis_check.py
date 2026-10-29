#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P61: 差速轮方向约定的**自包含**判定实验(不依赖 ROS/bridge)。

背景:
  P58 起重写 SDF 时把 robot/person 的 4 处差速轮 joint axis 从 `0 0 -1`
  改成了 `0 0 1`, 导致机器人与人都是"倒着走"。但翻转两个轮轴会**同时**
  镜像"前进"与"差速"(yaw)两个自由度, 所以单看一个轴无法判断哪套约定
  才是对的 —— 必须同时看两个量, 并与历史可用版逐位对照。

本脚本:
  1. 直连 gz transport, 发 /model/robot/cmd_vel
  2. cmd.x=+0.5  3s -> 记录 Δpos 与车头方向的点积(>0 = 沿车头前进)
  3. cmd.z=+1.5  3s -> 记录 Δyaw(>0 = 逆时针/左转)
  4. cmd.x=-0.5 / cmd.z=-1.5 反向复核
输出用 "VERDICT" 行给出四元组结论, 便于两个 SDF 版本逐位比对。

用法: python3 p61_axis_check.py [标签]
"""
import math
import sys
import time

import gz.transport13 as gt
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist as GzTwist

TAG = sys.argv[1] if len(sys.argv) > 1 else 'CURRENT'
st = {}
gn = gt.Node()


def cb(m):
    for p in m.pose:
        if p.name in ('robot', 'person'):
            q = p.orientation
            st[p.name] = (p.position.x, p.position.y,
                          math.degrees(math.atan2(
                              2 * (q.w * q.z + q.x * q.y),
                              1 - 2 * (q.y * q.y + q.z * q.z))))


gn.subscribe(Pose_V, '/world/follow_world/dynamic_pose/info', cb)
pub = gn.advertise('/model/robot/cmd_vel', GzTwist)
zero = GzTwist()


def zero_out(n=8):
    for _ in range(n):
        pub.publish(zero)
        time.sleep(0.05)


def drive(v, w, secs):
    tw = GzTwist()
    tw.linear.x = v
    tw.angular.z = w
    p0 = st.get('robot')
    t0 = time.time()
    while time.time() - t0 < secs:
        pub.publish(tw)
        time.sleep(0.1)
    zero_out(4)
    time.sleep(0.6)
    p = st.get('robot')
    if not (p and p0):
        return None
    return p, p0


def report(label, v, w):
    r = drive(v, w, 3.0)
    if r is None:
        print('  [%s] 无位姿数据' % label)
        return None
    (x1, y1, yaw1), (x0, y0, yaw0) = r
    dx, dy = x1 - x0, y1 - y0
    n = math.hypot(dx, dy)
    fwd = (dx * math.cos(yaw0) + dy * math.sin(yaw0)) / n if n > 1e-6 else 0.0
    dyaw = yaw1 - yaw0
    print('  %-12s Δpos=(%+.3f,%+.3f) 前向点积=%+.3f  Δyaw=%+7.1f'
          % (label, dx, dy, fwd, dyaw), flush=True)
    return fwd, dyaw


print('===== 差速轮方向判定 [%s] =====' % TAG)
time.sleep(2.0)
zero_out(10)
time.sleep(1.0)
for k in ('robot', 'person'):
    if k in st:
        print('  初始 %-7s pos=(%+.3f,%+.3f) yaw=%+.1f' % (k, st[k][0], st[k][1], st[k][2]))

r1 = report('cmd.x=+0.5', 0.5, 0.0)
r2 = report('cmd.x=-0.5', -0.5, 0.0)
r3 = report('cmd.z=+1.5', 0.0, 1.5)
r4 = report('cmd.z=-1.5', 0.0, -1.5)

print()
if all(x is not None for x in (r1, r2, r3, r4)):
    fwd_ok = (r1[0] > 0.5) and (r2[0] < -0.5)
    yaw_ok = (r3[1] > 30) and (r4[1] < -30)
    print('VERDICT [%s] 前进=%s  左转(w>0)=%s'
          % (TAG, 'OK' if fwd_ok else 'BAD', 'OK' if yaw_ok else 'BAD'))
    print('  -> %s' % ('可用: v>0 前进 且 w>0 左转' if (fwd_ok and yaw_ok)
                        else '不可用: 必须修正后再测'))
