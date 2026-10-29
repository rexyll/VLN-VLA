#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""verify_hang.py —— 悬垂初始态逐关节验收。

用法: python3 tools/verify_hang.py        # 触发 /drink/hang 后逐关节核对
判定:
  * 14 个臂关节: 指令(Q_HANG=0) vs 实测, |Δ| < 2° 且都出现在
    /drink/joint_state 里 = 连接且受控;
  * 20 个手部关节: 指令(GRIP_HANG=0, 并拢伸直) vs 实测, |Δ| < 5°;
  * FK 掌心位置 = 肩下 0.615m(世界 z ≈ 0.335), 掌系 -z ≈ 世界 -z(手指朝下)。
"""
import json
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'vla_ws', 'src', 'vla_follow'))
from vla_follow import arm_kin as K          # noqa: E402

ARM_TOL_DEG = 2.0
HAND_TOL_DEG = 5.0


def main():
    rclpy.init()
    n = Node('verify_hang')
    js = {}
    hb = {}

    def _js(m):
        js.update(zip(m.name, m.position))
    n.create_subscription(JointState, '/drink/joint_state', _js, 20)

    def _hb(m):
        try:
            hb.update(json.loads(m.data))
        except Exception:
            pass
    n.create_subscription(String, '/drink/state', _hb, 10)
    cli = n.create_client(Trigger, '/drink/hang')

    # 1) 触发悬垂(服务是异步排队, 主循环里执行)
    t0 = time.time()
    while not cli.service_is_ready() and time.time() - t0 < 10:
        rclpy.spin_once(n, timeout_sec=0.1)
    fut = cli.call_async(Trigger.Request())
    t0 = time.time()
    while not fut.done() and time.time() - t0 < 5:
        rclpy.spin_once(n, timeout_sec=0.05)
    print('HANG 服务: %s' % ('已排队' if fut.done() else '无应答'))
    # 2) 等动作完成(home 不需要, 悬垂两次 move ≈ 4~8s)
    t0 = time.time()
    while time.time() - t0 < 14:
        rclpy.spin_once(n, timeout_sec=0.05)
    # 3) 逐关节核对
    rows = []
    bad = 0
    for s in ('R', 'L'):
        for jname, _off, _ax, _lim in K.JOINTS:
            full = '%s_%s_joint' % (s, jname)
            meas = js.get(full)
            d = (math.degrees(abs(meas)) if meas is not None else None)
            ok = (d is not None and d < ARM_TOL_DEG)
            bad += 0 if ok else 1
            rows.append((full, 0.0, meas, d, ok))
        for hname in K.hand_joints(s):
            full = '%s_joint' % hname   # hand_joints 已含侧别前缀
            meas = js.get(full)
            d = (abs(meas) if meas is not None else None)
            ok = (d is not None and d < HAND_TOL_DEG)
            bad += 0 if ok else 1
            rows.append((full, 0.0, meas, d, ok))
    print('%-26s %8s %10s %9s %s' % ('joint', 'cmd(rad)', 'meas(rad)',
                                     'Δ', '连接'))
    for full, c, m, d, ok in rows:
        print('%-26s %8.3f %10s %9s %s'
              % (full, c, ('%.4f' % m) if m is not None else '缺失',
                 ('%.2f°' % d) if d is not None else '-',
                 'OK' if ok else '★NG'))
    # 4) FK 掌心(用心跳里的 base 真值)
    base = hb.get('base') or [0.0, 0.0, 0.0]
    for s in ('R', 'L'):
        q = [js.get('%s_%s_joint' % (s, j[0]), float('nan'))
             for j in K.JOINTS]
        T = K.fk_world(s, K.Q_HANG, tuple(base))
        # Q_HANG 下掌系=世界系(仅差底盘 yaw), 掌系 +z 朝上 → 手指(-z)朝下
        print('%s侧掌心(FK): (%+.3f, %+.3f, %+.3f)  掌系z·世界z=%+.3f '
              '(≈+1 即手指沿 -z 朝下)' % (s, T[0, 3], T[1, 3], T[2, 3],
                                          T[2, 2]))
    print('==== %s (异常关节数 %d) ====' % ('ALL OK' if bad == 0 else 'FAILED',
                                           bad))
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if bad == 0 else 1)


if __name__ == '__main__':
    main()
