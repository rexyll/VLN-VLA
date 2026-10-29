#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""traj_viz.py —— 在 Gazebo 窗口里实时画出机器人与目标的轨迹

原理
----
Gazebo Harmonic 的 GUI 自带 MarkerManager 插件，它把 marker 渲染成场景里的几何体。
注意它是**服务式**接口而不是话题式（这是最容易踩的坑）：

    /marker        Marker   -> Empty      单条（action=DELETE_ALL 可清空全部）
    /marker_array  Marker_V -> Boolean    批量
    /marker/list   Empty    -> Marker_V   查询已注册的 marker

并且必须在 ~/.gz/sim/8/gui.config 的 MarkerManager 节点里写明
<topic_name>/marker</topic_name>，否则服务名是空的，插件根本不会 advertise。

本节点订阅 /world/<world>/dynamic_pose/info 的真值位姿，**采集与渲染分离**：
位姿回调只做无阻塞的记录，独立线程按固定频率把 marker 批量推给 GUI。
（早期版本在回调里同步 request，服务端一慢就把采集也拖成 2 秒一帧。）

    id=1 机器人轨迹线(青)  id=2 目标轨迹线(橙红)
    id=3 机器人当前位置    id=4 目标当前位置
    id=5 目标朝向箭头      id=6 机器人朝向箭头
    id=7 机器人起点        id=8 目标起点

用法
----
    python3 traj_viz.py
    python3 traj_viz.py --world follow_world --out /tmp/traj.csv --rate 5
"""

import argparse
import math
import os
import signal
import sys
import threading
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.marker_pb2 import Marker
from gz.msgs10.marker_v_pb2 import Marker_V
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node

C_ROBOT_PATH = (0.00, 0.85, 1.00)
C_PERSON_PATH = (1.00, 0.35, 0.05)
C_ROBOT_NOW = (1.00, 0.95, 0.10)
C_PERSON_NOW = (1.00, 0.15, 0.60)
C_START = (0.60, 0.60, 0.60)

Z_RPATH = 0.045
Z_PPATH = 0.058
LINE_W = 0.12          # 注意: gz-gui 的 MarkerManager 用 ogre 后端时线宽不生效,
                       # 实际永远是 1px, 所以另外叠一层"面包屑"小球保证可见
BEAD_STEP = 0.40       # 每走 0.40m 撒一颗珠子
BEAD_MAX = 520         # 轨迹珠子总数上限, 防止 marker 过多拖垮 GUI
BEAD_R = 0.22


def yaw_of(q):
    """四元数 → yaw（分母 1-2(y²+z²)，本项目曾因写错导致全局航向失真）"""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def paint(mat, color):
    for slot in ('ambient', 'diffuse'):
        c = getattr(mat, slot)
        c.r, c.g, c.b = color
        c.a = 1.0
    mat.specular.r = mat.specular.g = mat.specular.b = 0.35
    mat.specular.a = 1.0


def new_marker(ns, mid, mtype, color, scale):
    mk = Marker()
    mk.ns = ns
    mk.id = mid
    mk.action = Marker.ADD_MODIFY
    mk.type = mtype
    mk.scale.x, mk.scale.y, mk.scale.z = scale
    paint(mk.material, color)
    return mk


class TrajViz:
    def __init__(self, args):
        self.args = args
        self.node = Node()
        self.lock = threading.Lock()

        self.r_pts = []
        self.p_pts = []
        self.r_pose = None
        self.p_pose = None
        # 面包屑: 每 BEAD_STEP 记一颗, 每个用唯一 id 只发一次, 避免重复重建几何
        self.r_beads = []
        self.p_beads = []
        self._r_last_bead = None
        self._p_last_bead = None
        self._bead_id = 1000
        self._bead_total = 0
        self.pose_msgs = 0
        self.rows = 0
        self.running = True
        self.started = time.monotonic()
        self.pub_fail = 0

        ok, _ = self.node.request('/marker/list', Empty(), Empty, Marker_V, 3000)
        if not ok:
            print('[TRAJVIZ] 致命: /marker/list 不可用，检查 gui.config 的 <topic_name>',
                  flush=True)
            raise SystemExit(2)
        print('[TRAJVIZ] marker 服务就绪', flush=True)

        self.csv = open(args.out, 'w', encoding='utf-8', buffering=1)
        self.csv.write('t,rx,ry,ryaw,px,py,pyaw\n')

        topic = '/world/%s/dynamic_pose/info' % args.world
        sub_ok = self.node.subscribe(Pose_V, topic, self._on_pose)
        print('[TRAJVIZ] subscribe %s -> %s' % (topic, sub_ok), flush=True)

        clr = Marker()
        clr.action = Marker.DELETE_ALL
        self.node.request('/marker', clr, Marker, Empty, 3000)
        print('[TRAJVIZ] 已清空旧 marker', flush=True)

    # ---------------- 采集：唯一职责是把数值抄下来 ----------------
    def _on_pose(self, msg):
        r = p = None
        for e in msg.pose:
            if e.name == 'robot':
                r = e
            elif e.name == 'person':
                p = e

        # 在消息仍有效时立刻把值读成 float，避免与传输层复用缓冲区竞争
        r_val = None
        if r is not None:
            r_val = (r.position.x, r.position.y, r.position.z,
                     yaw_of(r.orientation))
        p_val = None
        if p is not None:
            p_val = (p.position.x, p.position.y, p.position.z,
                     yaw_of(p.orientation))

        now = time.monotonic()
        self.pose_msgs += 1
        with self.lock:
            if r_val:
                self.r_pose = r_val
                if (not self.r_pts
                        or math.hypot(r_val[0] - self.r_pts[-1][0],
                                      r_val[1] - self.r_pts[-1][1])
                        >= self.args.min_step):
                    self.r_pts.append((r_val[0], r_val[1]))
                    if len(self.r_pts) > self.args.max_pts:
                        del self.r_pts[0]
                if (self._bead_total < BEAD_MAX
                        and (self._r_last_bead is None
                             or math.hypot(r_val[0] - self._r_last_bead[0],
                                           r_val[1] - self._r_last_bead[1]) >= BEAD_STEP)):
                    self._r_last_bead = (r_val[0], r_val[1])
                    self.r_beads.append((r_val[0], r_val[1]))
                    self._bead_total += 1
            if p_val:
                self.p_pose = p_val
                if (not self.p_pts
                        or math.hypot(p_val[0] - self.p_pts[-1][0],
                                      p_val[1] - self.p_pts[-1][1])
                        >= self.args.min_step):
                    self.p_pts.append((p_val[0], p_val[1]))
                    if len(self.p_pts) > self.args.max_pts:
                        del self.p_pts[0]
                if (self._bead_total < BEAD_MAX
                        and (self._p_last_bead is None
                             or math.hypot(p_val[0] - self._p_last_bead[0],
                                           p_val[1] - self._p_last_bead[1]) >= BEAD_STEP)):
                    self._p_last_bead = (p_val[0], p_val[1])
                    self.p_beads.append((p_val[0], p_val[1]))
                    self._bead_total += 1

            if self.r_pose or self.p_pose:
                rp = self.r_pose or (float('nan'),) * 4
                pp = self.p_pose or (float('nan'),) * 4
                self.csv.write('%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n'
                               % (now - self.started, rp[0], rp[1], rp[3],
                                  pp[0], pp[1], pp[3]))
                self.rows += 1

    # ---------------- 渲染：独立线程，慢也不影响采集 ----------------
    def _build(self):
        with self.lock:
            r_pts, p_pts = list(self.r_pts), list(self.p_pts)
            r_pose, p_pose = self.r_pose, self.p_pose

        out = []
        if len(r_pts) >= 2:
            mk = new_marker('vla', 1, Marker.LINE_STRIP, C_ROBOT_PATH,
                            (LINE_W, LINE_W, LINE_W))
            for x, y in r_pts:
                pt = mk.point.add()
                pt.x, pt.y, pt.z = x, y, Z_RPATH
            out.append(mk)
        if len(p_pts) >= 2:
            mk = new_marker('vla', 2, Marker.LINE_STRIP, C_PERSON_PATH,
                            (LINE_W, LINE_W, LINE_W))
            for x, y in p_pts:
                pt = mk.point.add()
                pt.x, pt.y, pt.z = x, y, Z_PPATH
            out.append(mk)

        if r_pose:
            mk = new_marker('vla', 3, Marker.SPHERE, C_ROBOT_NOW, (0.3, 0.3, 0.3))
            mk.pose.position.x, mk.pose.position.y = r_pose[0], r_pose[1]
            mk.pose.position.z = 0.30
            out.append(mk)
            mk = new_marker('vla', 6, Marker.BOX, C_ROBOT_PATH, (0.46, 0.08, 0.08))
            mk.pose.position.x = r_pose[0] + 0.26 * math.cos(r_pose[3])
            mk.pose.position.y = r_pose[1] + 0.26 * math.sin(r_pose[3])
            mk.pose.position.z = 0.14
            mk.pose.orientation.z = math.sin(r_pose[3] / 2)
            mk.pose.orientation.w = math.cos(r_pose[3] / 2)
            out.append(mk)
        if p_pose:
            mk = new_marker('vla', 4, Marker.SPHERE, C_PERSON_NOW, (0.34, 0.34, 0.34))
            mk.pose.position.x, mk.pose.position.y = p_pose[0], p_pose[1]
            mk.pose.position.z = 1.15
            out.append(mk)
            mk = new_marker('vla', 5, Marker.BOX, C_PERSON_PATH, (0.46, 0.08, 0.08))
            mk.pose.position.x = p_pose[0] + 0.26 * math.cos(p_pose[3])
            mk.pose.position.y = p_pose[1] + 0.26 * math.sin(p_pose[3])
            mk.pose.position.z = 1.15
            mk.pose.orientation.z = math.sin(p_pose[3] / 2)
            mk.pose.orientation.w = math.cos(p_pose[3] / 2)
            out.append(mk)
        if r_pts:
            mk = new_marker('vla', 7, Marker.SPHERE, C_START, (0.16, 0.16, 0.16))
            mk.pose.position.x, mk.pose.position.y = r_pts[0]
            mk.pose.position.z = 0.09
            out.append(mk)
        if p_pts:
            mk = new_marker('vla', 8, Marker.SPHERE, C_START, (0.20, 0.20, 0.20))
            mk.pose.position.x, mk.pose.position.y = p_pts[0]
            mk.pose.position.z = 0.11
            out.append(mk)
        return out

    def _pub_loop(self):
        period = 1.0 / max(0.5, self.args.rate)
        while self.running:
            time.sleep(period)
            try:
                out = self._build()
                # 新珠子只发一次, 用唯一 id 追加, 不重建已有几何
                with self.lock:
                    new_r, self.r_beads = self.r_beads, []
                    new_p, self.p_beads = self.p_beads, []
                for x, y in new_r:
                    mk = new_marker('bead', self._bead_id, Marker.SPHERE,
                                    C_ROBOT_PATH, (BEAD_R, BEAD_R, 0.05))
                    mk.pose.position.x = x
                    mk.pose.position.y = y
                    mk.pose.position.z = 0.030
                    self._bead_id += 1
                    out.append(mk)
                for x, y in new_p:
                    mk = new_marker('bead', self._bead_id, Marker.SPHERE,
                                    C_PERSON_PATH, (BEAD_R, BEAD_R, 0.05))
                    mk.pose.position.x = x
                    mk.pose.position.y = y
                    mk.pose.position.z = 0.042
                    self._bead_id += 1
                    out.append(mk)
                if not out:
                    continue
                batch = Marker_V()
                batch.marker.extend(out)
                self.node.request('/marker_array', batch, Marker_V, Boolean, 800)

                with self.lock:
                    rp, pp = self.r_pose, self.p_pose
                    nr, np = len(self.r_pts), len(self.p_pts)
                if rp and pp:
                    print('[TRAJVIZ] t=%5.1fs 机器人=(%+.2f,%+.2f,%+.0f°) '
                          '目标=(%+.2f,%+.2f,%+.0f°) 距离=%.2fm 点=%d/%d'
                          % (time.monotonic() - self.started,
                             rp[0], rp[1], math.degrees(rp[3]),
                             pp[0], pp[1], math.degrees(pp[3]),
                             math.hypot(rp[0] - pp[0], rp[1] - pp[1]), nr, np),
                          flush=True)
            except Exception as exc:
                self.pub_fail += 1
                if self.pub_fail < 5:
                    print('[TRAJVIZ] 发布异常: %r' % (exc,), flush=True)

    def spin(self):
        def _stop(*_):
            self.running = False
        signal.signal(signal.SIGTERM, _stop)
        signal.signal(signal.SIGINT, _stop)
        th = threading.Thread(target=self._pub_loop, daemon=True)
        th.start()
        last = 0.0
        while self.running:
            time.sleep(0.2)
            if time.monotonic() - last > 10.0:
                last = time.monotonic()
                if self.pose_msgs == 0:
                    print('[TRAJVIZ] 警告: 10s 无位姿，检查 world 名/仿真状态',
                          flush=True)
        self.csv.close()
        print('[TRAJVIZ] 退出: 采样 %d 行, 机器人点 %d, 目标点 %d, 发布异常 %d'
              % (self.rows, len(self.r_pts), len(self.p_pts), self.pub_fail),
              flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', default=os.environ.get('VLA_WORLD', 'follow_world'))
    ap.add_argument('--out', default='/home/earl/workspace/VLA/traj_log.csv')
    ap.add_argument('--rate', type=float, default=5.0)
    ap.add_argument('--min-step', type=float, default=0.02)
    ap.add_argument('--max-pts', type=int, default=4000)
    return TrajViz(ap.parse_args()).spin()


if __name__ == '__main__':
    sys.exit(main())
