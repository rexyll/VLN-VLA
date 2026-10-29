#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_los_probe.py —— 丢目标的"几何 vs 感知"归因探针

对每一拍记录:
  - 机器人与人的真值位姿 (只用于分析, 不进控制)
  - /follower/status 的 detected / cam / vis_dist / track / note
  - 机器人→人 的连线是否被障碍物遮挡 (OBB 相交), 被谁遮挡, 遮挡余量
  - 人离最近障碍物表面的距离
输出: los_<tag>.jsonl
"""
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import gz.transport13 as gz_transport
from gz.msgs10.pose_v_pb2 import Pose_V as GzPose_V

WORLD = 'follow_world'


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))

# (x, y, yaw, 半长, 半宽)  与 work_area.sdf 一致
OBS = [
    ('ob1', 3.5, 2.0, 0.0, 0.4, 0.4),
    ('ob2', -3.0, 3.2, 0.5, 0.6, 0.3),
    ('ob3', 0.5, -3.2, 0.0, 0.4, 0.4),
    ('ob4', -4.2, -2.5, 0.0, 0.5, 0.5),
    ('ob5', 4.2, -3.0, 0.0, 0.3, 0.3),
    ('ob6', -1.5, -4.5, -0.4, 0.5, 0.25),
    ('ob7', 2.2, 4.6, 0.0, 0.35, 0.35),
]
HFOV = 1.8


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def seg_obb(x0, y0, x1, y1, cx, cy, yaw, ha, hb):
    """线段与有向包围盒是否相交(Slab 法)。返回 True=相交(被遮挡)。"""
    c, s = math.cos(-yaw), math.sin(-yaw)
    ax, ay = x0 - cx, y0 - cy
    bx, by = x1 - cx, y1 - cy
    ax, ay = ax * c - ay * s, ax * s + ay * c
    bx, by = bx * c - by * s, bx * s + by * c
    dx, dy = bx - ax, by - ay
    t0, t1 = 0.0, 1.0
    for p, d, h in ((ax, dx, ha), (ay, dy, hb)):
        if abs(d) < 1e-9:
            if p < -h or p > h:
                return False
        else:
            ta, tb = (-h - p) / d, (h - p) / d
            if ta > tb:
                ta, tb = tb, ta
            t0, t1 = max(t0, ta), min(t1, tb)
            if t0 > t1:
                return False
    return True


def dist_obb(x, y, cx, cy, yaw, ha, hb):
    """点到 OBB 表面的距离(盒内为 0)"""
    c, s = math.cos(-yaw), math.sin(-yaw)
    ax, ay = x - cx, y - cy
    lx, ly = ax * c - ay * s, ax * s + ay * c
    dx = max(abs(lx) - ha, 0.0)
    dy = max(abs(ly) - hb, 0.0)
    return math.hypot(dx, dy)


class Probe(Node):
    def __init__(self, tag):
        super().__init__('los_probe')
        self.f = open('los_%s.jsonl' % tag, 'w', encoding='utf-8')
        self.rp = None
        self.pp = None
        self.st = {}
        self.t0 = time.monotonic()
        self._gz = gz_transport.Node()
        self._gz.subscribe(GzPose_V, '/world/%s/dynamic_pose/info' % WORLD,
                           self._gt)
        self.create_subscription(String, '/follower/status', self._s, 10)
        self.create_timer(0.1, self._tick)
        print('LOS-PROBE 就绪 -> los_%s.jsonl' % tag, flush=True)

    def _gt(self, msg):
        for p in msg.pose:
            if p.name == 'robot':
                self.rp = (p.position.x, p.position.y, yaw_of(p.orientation))
            elif p.name == 'person':
                self.pp = (p.position.x, p.position.y)

    def _s(self, m):
        try:
            self.st = json.loads(m.data)
        except Exception:
            pass

    def _tick(self):
        if self.rp is None or self.pp is None or not self.st:
            return
        rx, ry, ryaw = self.rp
        px, py = self.pp
        dx, dy = px - rx, py - ry
        d = math.hypot(dx, dy)
        brg = wrap(math.atan2(dy, dx) - ryaw)
        occl = []
        for (n, ox, oy, oyaw, ha, hb) in OBS:
            if seg_obb(rx, ry, px, py, ox, oy, oyaw, ha, hb):
                occl.append(n)
        dmin = min(dist_obb(px, py, o[1], o[2], o[3], o[4], o[5])
                   for o in OBS)
        rec = {'t': round(time.monotonic() - self.t0, 2),
               'd': round(d, 3), 'brg': round(math.degrees(brg), 1),
               'det': bool(self.st.get('detected')),
               'cam': self.st.get('cam', ''),
               'vd': self.st.get('vis_dist', -1.0),
               'trk': self.st.get('track', ''),
               'note': self.st.get('note', ''),
               'occl': occl,
               'pd': round(dmin, 3),
               'in_fov': abs(brg) < HFOV / 2.0,
               'rx': round(rx, 2), 'ry': round(ry, 2),
               'px': round(px, 2), 'py': round(py, 2)}
        self.f.write(json.dumps(rec, ensure_ascii=False) + '\n')
        self.f.flush()

    def done(self):
        try:
            self.f.close()
        except Exception:
            pass


def main():
    tag = sys.argv[1] if len(sys.argv) > 1 else 'x'
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 120.0
    rclpy.init()
    n = Probe(tag)
    t0 = time.monotonic()
    while time.monotonic() - t0 < dur:
        rclpy.spin_once(n, timeout_sec=0.05)
    n.done()
    n.destroy_node()
    rclpy.try_shutdown()
    print('LOS-PROBE 完成', flush=True)


if __name__ == '__main__':
    main()
