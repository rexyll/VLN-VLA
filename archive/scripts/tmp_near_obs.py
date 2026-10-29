#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_near_obs.py —— 定点诊断"人在障碍物附近时的视觉检出率"。

机器人冻结在 (RX,RY) 朝 +x, 依次把人摆到障碍 ob1(3.5,2.0 方 0.8x0.8) 的
各个相对位置, 每个位置采样 /perception/person, 再用 4 台相机原始帧离线
复现 person_vision 的全部门限, 指出到底是哪一道把人干掉的。

用法: python3 tmp_near_obs.py
"""
import json
import math
import subprocess
import sys
import time

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import gz.transport13 as gz_transport
from gz.msgs10.image_pb2 import Image as GzImage

WORLD = 'follow_world'
RX, RY, RYAW = 1.5, 2.0, 0.0

# ---- 与 person_vision.py 完全一致的参数 ----
DARK_V_MAX, SAT_MAX = 70, 100
AREA_MIN, AREA_MAX = 20, 22000
ASPECT_MIN, ASPECT_MAX = 0.2, 2.8
HFOV = 1.7453293
DEPTH_MIN, DEPTH_MAX = 0.28, 7.0
W_REAL, W_TOL = 0.36, 0.62

CAMS = (('front', 0.0), ('left', 1.5707963),
        ('right', -1.5707963), ('back', 3.1415926))

OBS = [('box', 3.5, 2.0, 0.40, 0.40, 0.0),
       ('box', -3.0, 3.2, 0.60, 0.30, 0.5),
       ('cyl', 0.5, -3.2, 0.40, 0.0, 0.0),
       ('cyl', -4.2, -2.5, 0.50, 0.0, 0.0),
       ('box', 4.2, -3.0, 0.30, 0.30, 0.0),
       ('box', -1.5, -4.5, 0.50, 0.25, -0.4),
       ('cyl', 2.2, 4.6, 0.35, 0.0, 0.0)]


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


def obj_dist(x, y, ob):
    k, ox, oy, a, b, ya = ob
    dx, dy = x - ox, y - oy
    if ya:
        c, s = math.cos(-ya), math.sin(-ya)
        dx, dy = dx * c - dy * s, dx * s + dy * c
    if k == 'cyl':
        return max(0.0, math.hypot(dx, dy) - a)
    qx, qy = abs(dx) - a, abs(dy) - b
    return math.hypot(max(qx, 0.0), max(qy, 0.0))


class Probe(Node):
    def __init__(self):
        super().__init__('near_obs_probe')
        self.vis = {}
        self.frames = {}
        self.create_subscription(String, '/perception/person', self._on_vis, 10)
        self._gz = gz_transport.Node()
        for name, _ in CAMS:
            self._gz.subscribe(
                GzImage, '/depth_%s/image' % name,
                lambda m, n=name: self._on_rgb(m, n))
            self._gz.subscribe(
                GzImage, '/depth_%s/depth_image' % name,
                lambda m, n=name: self._on_dep(m, n))

    def _on_vis(self, msg):
        try:
            self.vis = json.loads(msg.data)
        except Exception:
            pass

    def _on_rgb(self, msg, name):
        w, h = msg.width, msg.height
        n = len(msg.data)
        if n == w * h * 3:
            a = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
        elif n == w * h * 4:
            a = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 4)[:, :, :3]
        else:
            return
        self.frames.setdefault(name, {})['rgb'] = a.copy()

    def _on_dep(self, msg, name):
        w, h = msg.width, msg.height
        if len(msg.data) < w * h * 4:
            return
        a = np.frombuffer(msg.data[:w * h * 4], dtype='<f4').reshape(h, w)
        self.frames.setdefault(name, {})['dep'] = a.copy()


def analyze_frame(rgb, dep):
    """复现 person_vision._detect_cam, 返回候选表与最终判定"""
    if rgb is None or dep is None:
        return None, '无数据', []
    if rgb.shape[:2] != dep.shape:
        dep = cv2.resize(dep, (rgb.shape[1], rgb.shape[0]))
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, (0, 0, 0), (179, SAT_MAX, DARK_V_MAX))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                               cv2.CHAIN_APPROX_SIMPLE)
    img_w = rgb.shape[1]
    tan_h = math.tan(HFOV / 2.0)
    rows, best = [], None
    for c in cnts:
        a = cv2.contourArea(c)
        if a < 12:
            continue
        x, y, w, h = cv2.boundingRect(c)
        asp = w / max(h, 1)
        y0, y1 = y + int(h * 0.15), y + int(h * 0.60)
        sub = dep[y0:max(y1, y0 + 1), x:x + w]
        valid = sub[np.isfinite(sub) & (sub > DEPTH_MIN) & (sub < DEPTH_MAX)]
        p30 = float(np.quantile(valid, 0.30)) if valid.size else -1.0
        d_geo = W_REAL * img_w / max(2.0 * w * tan_h, 1e-6)
        ok_a = AREA_MIN <= a <= AREA_MAX
        ok_s = ASPECT_MIN <= asp <= ASPECT_MAX
        ok_v = valid.size >= 10
        ok_g = (p30 > 0 and d_geo * (1.0 - W_TOL) <= p30 <= d_geo * (1.0 + W_TOL))
        rows.append({'area': int(a), 'x': x, 'y': y, 'w': w, 'h': h,
                     'asp': round(asp, 2), 'p30': round(p30, 2),
                     'nvalid': int(valid.size), 'dgeo': round(d_geo, 2),
                     'A': ok_a, 'S': ok_s, 'V': ok_v, 'G': ok_g,
                     'ok': ok_a and ok_s and ok_v and ok_g})
        if ok_a and ok_s and ok_v and ok_g:
            if best is None or a > best[0]:
                best = (a, p30, x + w / 2.0, w)
    rows.sort(key=lambda r: -r['area'])
    if best is None:
        return None, '未检出', rows
    a, dist, u, w = best
    bc = (img_w / 2.0 - u) * HFOV / img_w
    return {'dist': dist, 'bc': math.degrees(bc), 'area': int(a), 'w': w}, '检出', rows


def main():
    rclpy.init()
    node = Probe()
    time.sleep(1.0)

    # ob1 参考: box(3.5,2.0) 半尺寸 0.40 -> 近面 x=3.10
    # case = (标签, 机器人x, 机器人y, 机器人yaw, 人x, 人y)
    cases = [
        ('对照-空场正前1.05m', 1.5, -0.5, 0.0, 2.55, -0.50),
        ('障碍正前0.55m(人挡障)', 1.5, 2.0, 0.0, 2.55, 2.00),
        ('障碍左前(偏0.45m)',     1.5, 2.0, 0.0, 2.55, 1.55),
        ('障碍右前(偏0.45m)',     1.5, 2.0, 0.0, 2.55, 2.45),
        ('障碍正左0.85m',         1.5, 2.0, 0.0, 3.50, 1.15),
        ('障碍正左1.40m',         1.5, 2.0, 0.0, 3.50, 0.60),
        ('障碍正后(完全遮挡)',     1.5, 2.0, 0.0, 4.40, 2.00),
        ('紧贴障碍左面',           1.5, 2.0, 0.0, 3.10, 1.15),
    ]
    print('\n机器人固定 (%.1f, %.1f) yaw=%.0f°, ob1 方障中心(3.5,2.0) '
          '近面 x=3.1' % (RX, RY, math.degrees(RYAW)), flush=True)
    print('%-24s %8s %5s %6s %6s %6s %5s %s'
          % ('人的位置', '到ob距离', '检出率', 'cam', 'dist', '真值', '误差', '判定'))
    print('-' * 96, flush=True)

    for tag, rx, ry, ryaw, px, py in cases:
        ok = set_pose('robot', rx, ry, 0.15, ryaw)
        ok &= set_pose('person', px, py, 0.02, math.pi)
        time.sleep(1.0)
        node.vis = {}
        hits, cams, dists = 0, {}, []
        N = 22
        for _ in range(N):
            rclpy.spin_once(node, timeout_sec=0.045)
            v = node.vis
            if v.get('detected'):
                hits += 1
                cams[v.get('cam', '')] = cams.get(v.get('cam', ''), 0) + 1
                dists.append(v.get('dist', -1))
            time.sleep(0.012)
        truth = math.hypot(px - rx, py - ry)
        gob = min(obj_dist(px, py, ob) for ob in OBS)
        rate = 100.0 * hits / N
        cam = max(cams, key=cams.get) if cams else '-'
        dv = sum(dists) / len(dists) if dists else -1
        err = (dv - truth) if dv > 0 else float('nan')
        verdict = 'OK' if rate >= 80 else ('抖动' if rate >= 30 else '丢失')
        print('%-24s %8.2f %5.0f%% %6s %6.2f %6.2f %+6.2f %s'
              % (tag, gob, rate, cam, dv, truth, err, verdict), flush=True)

        # 离线复现: 只对 front 相机做详细门限分析
        for cn in ('front', 'left', 'right', 'back'):
            fr = node.frames.get(cn) or {}
            res, why, rows = analyze_frame(fr.get('rgb'), fr.get('dep'))
            if cn != 'front':
                continue
            if rows:
                print('   [front 离线] %s' % why, flush=True)
                for r in rows[:4]:
                    print('     area=%6d bbox=(%3d,%3d,%3d,%3d) asp=%4.2f '
                          'p30=%5.2f nvalid=%4d dgeo=%5.2f  A%d S%d V%d G%d'
                          % (r['area'], r['x'], r['y'], r['w'], r['h'],
                             r['asp'], r['p30'], r['nvalid'], r['dgeo'],
                             r['A'], r['S'], r['V'], r['G']), flush=True)
            else:
                print('   [front 离线] 无候选(暗色掩码为空)', flush=True)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
