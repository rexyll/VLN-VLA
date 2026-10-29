#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""避障稳定性专项测试 (v2: 按障碍真实形状判定)

场景: 把"人"放在障碍正后方, 机器人必须穿过障碍区域才能靠近 —— 这正是
"有时能绕开、有时直接撞"的场景。

判定依据改成按真实几何算距离, 而不是一律用外接圆:
  · 圆柱 ob3/ob4/ob7 : 到圆心的距离 - 半径
  · 方块 ob1/ob2/ob5/ob6: 把车体中心变换到方块的本地系, 得到到矩形的距离
                          (方块带 yaw, 必须考虑)
再把"车体"近似成半径 0.18 的外接圆与半径 0.10 的内切圆:
  gap_out = d - 0.18   > 0 说明怎么转都不会碰到
  gap_in  = d - 0.10   < 0 说明必然接触
所以:
  gap_in  < 0        → 撞上
  gap_out < 0 <= gap_in → 擦碰(可能蹭到角)
  其他               → 安全
"""
import math
import subprocess
import time

import gz.transport13 as gz
from gz.msgs10.pose_v_pb2 import Pose_V
import rclpy
from rclpy.node import Node as RNode
from std_msgs.msg import String

R_IN = 0.10      # 车体 0.30x0.20 → 内切半径(短边一半)
R_OUT = 0.18     # 外接半径
# (名称, x, y, 形状, 尺寸, yaw)  —— 与 work_area.sdf 严格一致
OBST = [
    ('ob1', 3.5, 2.0, 'box', (0.80, 0.80), 0.0),
    ('ob2', -3.0, 3.2, 'box', (1.20, 0.60), 0.5),
    ('ob3', 0.5, -3.2, 'cyl', (0.40,), 0.0),
    ('ob4', -4.2, -2.5, 'cyl', (0.50,), 0.0),
    ('ob5', 4.2, -3.0, 'box', (0.60, 0.60), 0.0),
    ('ob6', -1.5, -4.5, 'box', (1.00, 0.50), -0.4),
    ('ob7', 2.2, 4.6, 'cyl', (0.35,), 0.0),
]
POSE = {}


def shape_dist(px, py, o):
    """车体中心到障碍表面的最短距离(负值=已进入)"""
    _, ox, oy, kind, sz, oyaw = o
    if kind == 'cyl':
        return math.hypot(px - ox, py - oy) - sz[0]
    hx, hy = sz[0] / 2.0, sz[1] / 2.0
    dx, dy = px - ox, py - oy
    c, s = math.cos(-oyaw), math.sin(-oyaw)
    lx = dx * c - dy * s
    ly = dx * s + dy * c
    ddx = abs(lx) - hx
    ddy = abs(ly) - hy
    if ddx > 0 or ddy > 0:
        return math.hypot(max(ddx, 0.0), max(ddy, 0.0))
    return max(ddx, ddy)          # 在方块内部 → 负值


def pose_cb(msg):
    for p in msg.pose:
        if p.name in ('robot', 'person'):
            POSE[p.name] = (p.position.x, p.position.y)


def set_pose(name, x, y, yaw, z):
    qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    req = ('name: "%s", position: {x: %.4f, y: %.4f, z: %.3f}, '
           'orientation: {z: %.6f, w: %.6f}' % (name, x, y, z, qz, qw))
    r = subprocess.run(
        ['gz', 'service', '-s', '/world/follow_world/set_pose',
         '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
         '--timeout', '3000', '--req', req],
        capture_output=True, timeout=10)
    return b'true' in r.stdout.lower()


def main():
    cases = [
        ('A 正面直冲 ob3(r0.40)', 0.5, -1.00, -90.0, 0.5, -4.30),
        ('B 斜 35° 冲 ob3', 2.20, -1.20, -125.0, 0.5, -4.30),
        ('C 正面直冲 ob1(0.8x0.8)', 3.5, 0.20, 90.0, 3.5, 3.60),
        ('D 冲墙角 ob4(r0.50)', -4.2, -0.60, -90.0, -4.2, -3.90),
        ('E 斜 40° 冲 ob5(0.6x0.6)', 2.60, -1.20, -108.0, 4.2, -3.90),
    ]

    gznode = gz.Node()
    gznode.subscribe(Pose_V, '/world/follow_world/dynamic_pose/info', pose_cb)
    rclpy.init()
    ros = RNode('avoid_test')
    pub = ros.create_publisher(String, '/robot/command', 10)

    print('=' * 80, flush=True)
    print('避障稳定性专项测试 v2  (车体内切 %.2fm / 外接 %.2fm)' % (R_IN, R_OUT),
          flush=True)
    print('gap_in<0 撞上 | gap_out<0 擦碰 | 否则安全', flush=True)
    print('=' * 80, flush=True)

    # 预热: 等位姿流稳定, 先把 set_pose 通一次(首轮常因服务未就绪而静默失败)
    t0 = time.time()
    while 'robot' not in POSE and time.time() - t0 < 15:
        time.sleep(0.2)
    set_pose('robot', 0.0, 0.0, 0.0, 0.15)
    time.sleep(2.5)
    print('预热完成, 机器人在 (%+.2f,%+.2f)'
          % POSE.get('robot', (9, 9)), flush=True)

    results = []
    for desc, rx, ry, ryaw_deg, px, py in cases:
        ok = False
        for attempt in range(3):
            ok = set_pose('robot', rx, ry, math.radians(ryaw_deg), 0.15)
            ok = set_pose('person', px, py, 0.0, 0.02) and ok
            time.sleep(0.3)
            if ok and abs(POSE.get('robot', (9, 9))[0] - rx) < 0.3:
                break
        time.sleep(2.0)
        if abs(POSE.get('robot', (9, 9))[0] - rx) > 0.4:
            print('%-26s  !! 复位失败, 跳过(得到 %s)'
                  % (desc, POSE.get('robot')), flush=True)
            continue
        POSE.clear()
        msg = String()
        msg.data = '{"action":"FOLLOW","distance":1.0,"text":"请跟随我"}'
        pub.publish(msg)

        t0 = time.time()
        g_in, g_out, w_name = 1e9, 1e9, '-'
        path = 0.0
        prev = None
        while time.time() - t0 < 30.0:
            time.sleep(0.05)
            r = POSE.get('robot')
            if r is None:
                continue
            if prev is not None:
                path += math.hypot(r[0] - prev[0], r[1] - prev[1])
            prev = r
            for o in OBST:
                d = shape_dist(r[0], r[1], o)
                if d - R_OUT < g_out:
                    g_out, w_name = d - R_OUT, o[0]
                if d - R_IN < g_in:
                    g_in = d - R_IN
        r = POSE.get('robot')
        p = POSE.get('person')
        d_end = math.hypot(r[0] - p[0], r[1] - p[1]) if (r and p) else -1
        verdict = '撞上' if g_in < 0 else ('擦碰' if g_out < 0 else '安全')
        results.append((desc, g_in, g_out, w_name, d_end, path, verdict))
        print('%-26s gap_in=%+.3f gap_out=%+.3f(%s) 距人=%.2f 行程=%.2f  → %s'
              % (desc, g_in, g_out, w_name, d_end, path, verdict), flush=True)
        stop = String()
        stop.data = '{"action":"STOP","text":"停"}'
        pub.publish(stop)
        time.sleep(1.0)

    print('=' * 80, flush=True)
    bad = [r for r in results if r[6] != '安全']
    print('结论: %d/%d 场景安全' % (len(results) - len(bad), len(results)),
          flush=True)
    for r in bad:
        print('  !! %s → %s (gap_in=%+.3f gap_out=%+.3f, %s)'
              % (r[0], r[6], r[1], r[2], r[3]), flush=True)
    trace = [r for r in results if r[6] == '安全' and r[2] < 0.08]
    if trace:
        print('  (另有 %d 个场景属于"外接圆相交但没碰到"的贴靠级)'
              % len(trace), flush=True)
    ros.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
