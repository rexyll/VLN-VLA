#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""probe_setpose.py —— 验证 `/world/<w>/set_pose` 能否做"位姿跟随"抓取附着

P86 抓取附着方案的前置验证。

背景: P85 已确认 gz 的 `DetachableJoint` 插件**用不了**(见 TaskPlan §6.4 #19):
它的 `child_model` 只在本模型内部找嵌套模型, 世界级的 kettle/cup 找不到就报
`[Wrn] Child Model kettle could not be found.`, 而且即使从不 attach 也会破坏
机械臂的关节约束求解(R_pan 硬锁 0.175、速度反馈恒 0)。

于是抓取附着只剩"位姿跟随": 抓握期间每帧把壶/杯设成"掌心位姿 × 抓握偏置"。
本探针验证三件事:
  [1] 服务 `/world/follow_world/set_pose` 存在
  [2] 水平 teleport 精确到位(排除"设定被物理拽回")
  [3] **20Hz 悬空保持的下坠量** —— 这是跟随方案唯一的精度代价:
      set_pose 每帧把物体钉在目标位姿, 帧与帧之间物理照常积分(重力), 所以
      会有"锯齿状"下坠。20Hz 下理论单帧自由落体 0.5*9.8*0.05² = 12mm。
      1ms 步长 + 每帧重置速度的话实际远小于此, 但要实测。
  [4] 连续 20 次 5mm 步进能否跟上(模拟跟随轨迹)
  [5] 结束后还原(±2mm)

用法: python3 tests/diag/probe_setpose.py
"""
import math
import sys
import time

import gz.transport13 as gz_transport
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.boolean_pb2 import Boolean

WORLD = 'follow_world'
TOPIC = '/world/%s/dynamic_pose/info' % WORLD
SVC = '/world/%s/set_pose' % WORLD
MODEL = 'kettle'
FOLLOW_HZ = 20.0

link = {}
hits = [0]


def cb(msg):
    hits[0] += 1
    for p in msg.pose:
        link[p.name] = (p.position.x, p.position.y, p.position.z,
                        p.orientation.x, p.orientation.y,
                        p.orientation.z, p.orientation.w)


def pos(name=None):
    v = link.get(name or MODEL)
    return None if v is None else (v[0], v[1], v[2])


def fmt(p):
    return '(%.4f, %.4f, %.4f)' % p


def spin(sec):
    t0 = time.time()
    while time.time() - t0 < sec:
        time.sleep(0.01)


def make_pose(p, quat):
    m = Pose()
    m.name = MODEL
    m.position.x, m.position.y, m.position.z = p
    (m.orientation.x, m.orientation.y,
     m.orientation.z, m.orientation.w) = quat
    return m


def main():
    node = gz_transport.Node()
    node.subscribe(Pose_V, TOPIC, cb)
    spin(2.0)

    print('=' * 74)
    print('P86 前置探针 —— set_pose 能否做位姿跟随')
    print('=' * 74)
    if hits[0] == 0:
        print('[!] 收不到 %s, 先确认 gz 世界在跑' % TOPIC)
        return 2

    p0 = pos()
    if p0 is None:
        print('[!] 真值里没有 %s; 现有实体样例: %s'
              % (MODEL, sorted(link)[:12]))
        return 2
    quat = link[MODEL][3:]
    print('初始 %s = %s   (真值帧 %d)' % (MODEL, fmt(p0), hits[0]))

    # ---- [1] 服务可达
    r = node.request(SVC, make_pose(p0, quat), Pose, Boolean, 3000)
    print('[1] set_pose 服务        %s' % ('可达' if r is not None else '不可达'))
    if r is None:
        return 2

    # ---- [2] 水平 teleport (z 不变, 排除重力干扰)
    tgt = (p0[0] + 0.10, p0[1] + 0.05, p0[2])
    node.request(SVC, make_pose(tgt, quat), Pose, Boolean, 3000)
    spin(0.6)
    p1 = pos()
    d1 = math.hypot(p1[0] - tgt[0], p1[1] - tgt[1])
    ok2 = d1 < 0.003
    print('[2] 水平 teleport +10/+5cm   实测 %s  误差 %.1f mm  %s'
          % (fmt(p1), d1 * 1000.0, 'OK' if ok2 else 'NG'))

    # ---- [3] 20Hz 悬空保持: 抬 10cm, 保持 1s, 量下坠
    hold = (p1[0], p1[1], p0[2] + 0.10)
    dt = 1.0 / FOLLOW_HZ
    samples = []
    t0 = time.time()
    while time.time() - t0 < 1.0:
        node.request(SVC, make_pose(hold, quat), Pose, Boolean, 500)
        time.sleep(dt)
        q = pos()
        if q:
            samples.append(q[2] - hold[2])
    sag_min = min(samples) if samples else float('nan')
    sag_max = max(samples) if samples else float('nan')
    sag_rms = (math.sqrt(sum(s * s for s in samples) / len(samples))
               if samples else float('nan'))
    ok3 = abs(sag_rms) < 0.010
    print('[3] 20Hz 悬空 1s           下坠 %.1f ~ %.1f mm (RMS %.1f mm) %s'
          % (sag_min * 1000.0, sag_max * 1000.0, sag_rms * 1000.0,
             'OK' if ok3 else 'NG'))

    # ---- [4] 20 次 5mm 步进(模拟跟随轨迹)
    tgt = list(hold)
    errs = []
    for _ in range(20):
        tgt[0] += 0.005
        node.request(SVC, make_pose(tgt, quat), Pose, Boolean, 500)
        time.sleep(dt)
        q = pos()
        if q:
            errs.append(math.hypot(q[0] - tgt[0], q[1] - tgt[1]))
    p2 = pos()
    dx = p2[0] - hold[0]
    ok4 = abs(dx - 0.100) < 0.010 and (not errs or max(errs) < 0.015)
    print('[4] 20×5mm 步进            前进 %.4f m (应 0.1000), 水平残差最大 %.1f mm %s'
          % (dx, (max(errs) * 1000.0 if errs else 0.0),
             'OK' if ok4 else 'NG'))

    # ---- [5] 还原
    node.request(SVC, make_pose(p0, quat), Pose, Boolean, 3000)
    spin(1.2)                      # 等它落回桌面坐稳
    p3 = pos()
    d3 = math.dist(p3, p0)
    ok5 = d3 < 0.003
    print('[5] 还原原位              %s  偏差 %.1f mm  %s'
          % (fmt(p3), d3 * 1000.0, 'OK' if ok5 else 'NG'))

    print('-' * 74)
    good = ok2 and ok3 and ok4 and ok5
    print('结论: %s' % ('通过 —— 位姿跟随方案可用' if good else '不通过'))
    print('=' * 74)
    return 0 if good else 2


if __name__ == '__main__':
    sys.exit(main())
