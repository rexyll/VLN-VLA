#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P60 定点探针: 机器人(-3,0,yaw0) 人(1,0) —— 人应在 front 正前 4m。
逐相机抓 RGB+深度, 离线复现 person_vision 全部门限, 报出每台相机
检不检得出 / 被哪道门限挡住。同时读 /perception/person 实际输出。
"""
import math
import subprocess
import sys
import time

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import gz.transport13 as gt
from gz.msgs10.image_pb2 import Image as GzImage
from gz.msgs10.twist_pb2 import Twist as GzTwist

WORLD = 'follow_world'
RX, RY, RYAW = -3.0, 0.0, 0.0
PX, PY = 1.0, 0.0

DARK_V_MAX, SAT_MAX = 70, 100
AREA_MIN, AREA_MAX = 20, 22000
ASPECT_MIN, ASPECT_MAX = 0.2, 2.8
HFOV = 1.7453293          # person_vision 里用的
HFOV_SDF = {'front': 1.6, 'left': 1.8, 'right': 1.8, 'back': 1.8}
DEPTH_MIN, DEPTH_MAX = 0.28, 7.0
W_REAL, W_TOL = 0.36, 0.62
CAMS = (('front', 0.0), ('left', 1.5707963),
        ('right', -1.5707963), ('back', 3.1415926))
FMT = {1: 'gray', 3: 'rgb', 4: 'rgba', 5: 'bgra'}


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


def grab(topic, want=4):
    got = []
    n = gt.Node()
    n.subscribe(GzImage, topic, lambda m: got.append(m) if len(got) < want
                else None)
    t0 = time.time()
    while time.time() - t0 < 4.0 and len(got) < want:
        time.sleep(0.05)
    return got[-1] if got else None


def decode(m):
    h, w = m.height, m.width
    k = m.pixel_format_type
    buf = np.frombuffer(m.data, dtype=np.uint8)
    if k == 3:                       # RGB_INT8
        return buf.reshape(h, w, 3)
    if k == 13:                      # L_FLOAT32 深度
        return np.frombuffer(m.data, dtype=np.float32).reshape(h, w)
    if k == 1:                       # L_INT8 灰度
        return buf.reshape(h, w).astype(np.float32)
    print('  (未知像素格式 %d, step=%d)' % (k, m.step))
    return None


def analyze(name, rgb, dep):
    """离线复现 person_vision 的门限链, 返回 (结论, 细节)."""
    if rgb is None:
        return '无RGB帧', ''
    if dep is None:
        return '无深度帧', ''
    if rgb.shape[:2] != dep.shape:
        dep = cv2.resize(dep, (rgb.shape[1], rgb.shape[0]))
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, (0, 0, 0), (179, SAT_MAX, DARK_V_MAX))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                               cv2.CHAIN_APPROX_SIMPLE)
    img_w = rgb.shape[1]
    tan_h = math.tan(HFOV / 2.0)
    info = ['contours=%d' % len(cnts)]
    best = None
    for c in cnts:
        a = cv2.contourArea(c)
        if a < AREA_MIN or a > AREA_MAX:
            continue
        x, y, w, h = cv2.boundingRect(c)
        asp = w / max(h, 1)
        if not (ASPECT_MIN <= asp <= ASPECT_MAX):
            continue
        y0 = y + int(h * 0.15)
        y1 = y + int(h * 0.60)
        sub = dep[y0:max(y1, y0 + 1), x:x + w]
        valid = sub[np.isfinite(sub) & (sub > DEPTH_MIN) & (sub < DEPTH_MAX)]
        if valid.size < 10:
            info.append('cand(a=%d)无深度' % a)
            continue
        dist = float(np.quantile(valid, 0.30))
        d_geo = W_REAL * img_w / max(2.0 * w * tan_h, 1e-6)
        ok = d_geo * (1 - W_TOL) <= dist <= d_geo * (1 + W_TOL)
        info.append('cand(a=%d,w=%d) d=%.2f d_geo=%.2f %s'
                    % (a, w, dist, d_geo, 'OK' if ok else '几何不符'))
        if ok and (best is None or a > best[0]):
            best = (a, x, y, w, h, dist)
    if best is None:
        return '未检出', ' '.join(info)
    a, x, y, w, h, dist = best
    u = x + w / 2.0
    bc = (img_w / 2.0 - u) * HFOV / img_w
    return '检出 d=%.2f bc=%+.1f°' % (dist, math.degrees(bc)), ' '.join(info)


def main():
    rclpy.init()
    node = Node('p60_probe')
    per = {}

    def cb(m):
        try:
            per.update(__import__('json').loads(m.data))
        except Exception:
            pass
    node.create_subscription(String, '/perception/person', cb, 10)

    # 停掉可能移动机器人的东西: 先 STOP(follower 发零速), 再 gz 侧清一次
    # —— diff_drive 会保持最后一条 cmd_vel, 不清零机器人就一直原地转。
    stop = String()
    stop.data = '{"action": "STOP", "text": "stop"}'
    pub = node.create_publisher(String, '/robot/command', 10)
    for _ in range(3):
        pub.publish(stop)
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.15)
    gzr = gt.Node()
    pubr = gzr.advertise('/model/robot/cmd_vel', GzTwist)
    z = GzTwist()
    for _ in range(10):
        pubr.publish(z)
        time.sleep(0.05)
    subprocess.run(['pkill', '-f', 'person_wal[k]'], capture_output=True)
    time.sleep(0.8)
    set_pose('robot', RX, RY, 0.15, RYAW)
    set_pose('person', PX, PY, 0.02, math.pi)
    time.sleep(1.5)

    print('机器人(%.1f,%.1f,yaw0) 人(%.1f,%.1f) —— 应在 front 正前 %.1fm'
          % (RX, RY, PX, PY, PX - RX))
    for cname, cang in CAMS:
        rgb = grab('/depth_%s/image' % cname)
        dep = grab('/depth_%s/depth_image' % cname)
        ri = analyze(cname, decode(rgb) if rgb else None,
                     decode(dep) if dep else None)
        print('[%s] %s  %s' % (cname, ri[0], ri[1][:110]))
    # perception 实际输出
    t0 = time.time()
    while time.time() - t0 < 3.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    print('perception:', {k: per.get(k) for k in
                          ('detected', 'cam', 'dist', 'bearing')})


if __name__ == '__main__':
    main()
