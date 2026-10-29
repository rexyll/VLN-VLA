#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P60c: 方向约定决定性实验。

问题: S1 场景里 follower 下发 v=+0.45, 机器人却**远离**目标
(4.28m -> 6.8m, 撞 -x 墙)。P60a 的直线前进测试也显示 pos.x 从 0 变到
-3.95。也就是说 gz diff_drive 的 cmd_vel.x>0 让机器人朝 chassis 的 **-x**
方向走, 而 follower's 几何约定(前向 = (cos yaw, sin yaw))假设前向是 +x。

本实验(与 follower 完全解耦, 直接驱动 gz 端):
  1) 发 cmd_vel.x = +0.5  3s  -> 看 pos 往哪走
  2) 发 cmd_vel.x = -0.5  3s  -> 看 pos 往哪走
  3) 发 cmd_vel.z = +1.5  3s  -> 看 yaw 往哪转(判断左右轮镜像是否正确)
并同时记录 chassis 的世界位姿与 front 相机朝向, 以确定
"front 相机" 在 chassis 系里到底是 +x 还是 -x。
"""
import math
import subprocess
import time

import gz.transport13 as gt
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist as GzTwist

st = {}
gn = gt.Node()


def cb(m):
    for p in m.pose:
        if p.name in ('robot', 'chassis', 'person'):
            q = p.orientation
            st[p.name] = (p.position.x, p.position.y, p.position.z,
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


def drive(v, w, secs, tag):
    tw = GzTwist()
    tw.linear.x = v
    tw.angular.z = w
    p0 = st.get('robot')
    t0 = time.time()
    while time.time() - t0 < secs:
        pub.publish(tw)
        time.sleep(0.1)
        p = st.get('robot')
        if p:
            print('  %-16s t=%4.1f pos=(%+6.2f,%+6.2f,%+6.2f) yaw=%+7.1f'
                  % (tag, time.time() - t0, p[0], p[1], p[2], p[3]), flush=True)
    zero_out(4)
    time.sleep(0.6)
    p = st.get('robot')
    if p and p0:
        print('  >>> %-14s Δpos=(%+.3f,%+.3f) Δyaw=%+.1f'
              % (tag, p[0] - p0[0], p[1] - p0[1], p[3] - p0[3]), flush=True)
    return st.get('robot')


# 先杀掉 follower, 否则它 20Hz 发零速会覆盖我们的指令
subprocess.run(['pkill', '-f', 'vla_follo[w]/lib'], capture_output=True)
subprocess.run(['pkill', '-f', 'person_wal[k]'], capture_output=True)
time.sleep(1.2)
zero_out(10)
time.sleep(1.0)

print('初始位姿:')
for k in ('robot', 'chassis', 'person'):
    if k in st:
        print('  %-8s pos=(%+.3f,%+.3f,%+.3f) yaw=%+.1f'
              % (k, st[k][0], st[k][1], st[k][2], st[k][3]))
print()
drive(0.5, 0.0, 3.0, 'cmd.x=+0.5')
drive(-0.5, 0.0, 3.0, 'cmd.x=-0.5')
drive(0.0, 1.5, 3.0, 'cmd.z=+1.5')
drive(0.0, -1.5, 3.0, 'cmd.z=-1.5')
print('\n结束位姿:')
for k in ('robot', 'chassis'):
    if k in st:
        print('  %-8s pos=(%+.3f,%+.3f,%+.3f) yaw=%+.1f'
              % (k, st[k][0], st[k][1], st[k][2], st[k][3]))
