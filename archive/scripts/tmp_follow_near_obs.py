#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_follow_near_obs.py —— 复现"人站在障碍附近, 机器人卡死不前进"。

场景固定为 eval13 日志里那次 20s 死锁的几何关系:
  机器人 (-2.60, -5.03) 朝 +x, 人 (-0.43, -4.95)  距离 2.18m
  ob6 = box(-1.5, -4.5) 1.0x0.5 yaw-0.4 恰在人机连线上(侧向仅 0.53m)
  到 ob6 表面真实间隙 0.733m —— 完全能过

P45 之前: 第 6 段"真值地图硬保护"用外接圆判 1.22 < 1.27 → v=0 且 w=0,
          机器人 20s 一动不动。
P45 之后: 用 OBB 真实间隙 0.73 > 0.28 → 放行, 应当正常跟到 1.0m。

用法: python3 tmp_follow_near_obs.py [持续秒数]
"""
import json
import math
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import gz.transport13 as gz_transport
from gz.msgs10.pose_v_pb2 import Pose_V

WORLD = 'follow_world'
RX, RY, RYAW = -2.60, -5.03, 0.0
PX, PY = -0.43, -4.95


def set_pose(name, x, y, z, yaw=0.0):
    req = ('name: "%s", position: {x: %s, y: %s, z: %s}, '
           'orientation: {z: %s, w: %s}'
           % (name, x, y, z, math.sin(yaw / 2), math.cos(yaw / 2)))
    for _ in range(3):
        r = subprocess.run(
            ['gz', 'service', '-s', '/world/%s/set_pose' % WORLD,
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '2000', '--req', req],
            capture_output=True, text=True)
        if 'true' in (r.stdout or '').lower():
            return True
        time.sleep(0.2)
    return False


def shoot_follow():
    r = subprocess.run(['bash', '/home/earl/workspace/VLA/tmp_follow.sh'],
                       capture_output=True, text=True, timeout=30)
    return r.stdout.strip().splitlines()[-1] if r.stdout else ''


class R(Node):
    def __init__(self):
        super().__init__('near_obs_follow')
        self.st = {}
        self.pos = {}
        self.create_subscription(String, '/follower/status', self._st, 10)
        self._gz = gz_transport.Node()
        self._gz.subscribe(Pose_V, '/world/%s/dynamic_pose/info' % WORLD,
                           self._pose)

    def _st(self, m):
        try:
            self.st = json.loads(m.data)
        except Exception:
            pass

    def _pose(self, m):
        for p in m.pose:
            if p.name in ('robot', 'person'):
                self.pos[p.name] = (p.position.x, p.position.y)


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    rclpy.init()
    node = R()
    time.sleep(0.8)

    # 强力停掉人的随机行走。注意: kill 掉节点**不会**清零速度 ——
    # gz 的 diff_drive 会一直保持最后一条 cmd_vel, 人会继续漂。
    # 必须显式注入 0 速度。
    from geometry_msgs.msg import Twist
    zero = Twist()
    pub_p = node.create_publisher(Twist, '/person/cmd_vel', 10)
    for sig in ([], ['-9']):
        subprocess.run(['pkill'] + sig + ['-f', 'person_wal[k]'],
                       capture_output=True)
        time.sleep(0.8)
    for _ in range(8):
        pub_p.publish(zero)
        rclpy.spin_once(node, timeout_sec=0.08)
        time.sleep(0.05)
    print('[REPRO] 已向 /person/cmd_vel 注入零速', flush=True)
    print('[REPRO] 复位: 机器人(%.2f,%.2f,0°) 人(%.2f,%.2f) 距%.2fm'
          % (RX, RY, PX, PY, math.hypot(PX - RX, PY - RY)), flush=True)
    set_pose('robot', RX, RY, 0.15, RYAW)
    set_pose('person', PX, PY, 0.02, math.pi)
    time.sleep(1.5)

    for k in range(3):
        print('[REPRO] 下指令:', shoot_follow(), flush=True)
        time.sleep(0.6)
        rclpy.spin_once(node, timeout_sec=0.1)

    time.sleep(0.3)
    rclpy.spin_once(node, timeout_sec=0.1)
    _rp, _pp = node.pos.get('robot'), node.pos.get('person')
    print('[REPRO] 实测: 机器人%s 人%s 距%.2fm'
          % (('(%.2f,%.2f)' % _rp) if _rp else '?',
             ('(%.2f,%.2f)' % _pp) if _pp else '?',
             math.hypot(_pp[0]-_rp[0], _pp[1]-_rp[1]) if (_rp and _pp) else -1),
          flush=True)

    t0 = time.monotonic()
    print('\n%6s %7s %7s %7s %6s %6s %6s %5s %s'
          % ('t', 'robot_x', 'robot_y', 'dist', 'cmd_v', 'cmd_w',
             'vis_d', 'mode', '备注'))
    print('-' * 88, flush=True)
    last = 0.0
    moved = 0.0
    p0 = None
    _pin_t = 0.0
    while time.monotonic() - t0 < dur:
        rclpy.spin_once(node, timeout_sec=0.05)
        t = time.monotonic() - t0
        if t - _pin_t > 0.4:            # 钉住人: gz 里刚体会缓慢沉降漂移
            _pin_t = t
            set_pose('person', PX, PY, 0.02, math.pi)
            pub_p.publish(zero)
        if t - last < 1.0:
            continue
        last = t
        rp = node.pos.get('robot')
        pp = node.pos.get('person')
        if not rp or not pp:
            continue
        if p0 is None:
            p0 = rp
        moved = math.hypot(rp[0] - p0[0], rp[1] - p0[1])
        d = math.hypot(pp[0] - rp[0], pp[1] - rp[1])
        st = node.st
        print('%6.1f %7.2f %7.2f %7.2f %6.2f %6.2f %6s %5s %s'
              % (t, rp[0], rp[1], d, st.get('cmd_v', 0.0),
                 st.get('cmd_w', 0.0),
                 ('%.2f' % st.get('vis_dist', -1)) if st.get('detected') else '-',
                 st.get('track', '?'), st.get('src', '')), flush=True)

    print('\n[REPRO] 累计位移 %.2fm' % moved, flush=True)
    print('[REPRO] 结论: %s'
          % ('能正常前进 → P45 已解除死锁' if moved > 0.6
             else '仍然卡死 → 还有别的约束'), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
