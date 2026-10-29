#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_v_sweep.py —— 绕人一圈测"裙摆"的实测亮度(V)与可用性

对 12 个方位把机器人放到人 1.2m 外, 抓 4 相机, 统计
  - 低饱和暗斑(V∈[40,180], S<=80) 的最大连通块: 中位 V / 面积 / bbox
  - 当前阈值(V<=70)能否检出
用来给"人的外观先验"定阈值, 而不是拍脑袋。
"""
import json
import math
import subprocess
import time

import cv2
import numpy as np

import gz.transport13 as gz_transport
from gz.msgs10.image_pb2 import Image as GzImage

WORLD = 'follow_world'
CAMS = ('front', 'left', 'right', 'back')
PX, PY = 0.0, 0.0

node = gz_transport.Node()
state = {}


def mk(n):
    def cb(msg):
        h, w = msg.height, msg.width
        if h < 10 or w < 10 or len(msg.data) != w * h * 3:
            return
        state[n] = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
    return cb


for c in CAMS:
    node.subscribe(GzImage, '/depth_%s/image' % c, mk(c))


def set_pose(name, x, y, z, yaw=0.0):
    req = ('name: "%s", position: {x: %s, y: %s, z: %s}, '
           'orientation: {z: %s, w: %s}'
           % (name, x, y, z, math.sin(yaw / 2), math.cos(yaw / 2)))
    r = subprocess.run(
        ['gz', 'service', '-s', '/world/%s/set_pose' % WORLD,
         '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
         '--timeout', '3000', '--req'], capture_output=True, text=True)
    return 'true' in (r.stdout or '').lower()


def best_blob(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    m = cv2.inRange(hsv, (0, 0, 40), (179, 80, 180))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    a = cv2.contourArea(c)
    x, y, w, h = cv2.boundingRect(c)
    if a < 150:
        return None
    sub = hsv[y:y + h, x:x + w]
    mm = m[y:y + h, x:x + w] > 0
    vs = sub[:, :, 2][mm]
    ss = sub[:, :, 1][mm]
    return {'a': int(a), 'w': w, 'h': h, 'y': y,
            'v50': int(np.median(vs)), 'v90': int(np.percentile(vs, 90)),
            'v10': int(np.percentile(vs, 10)),
            's50': int(np.median(ss)),
            'dark70': int((vs <= 70).sum())}


def main():
    set_pose('person', PX, PY, 0.02, 0.0)
    print('%-6s %-6s %-6s %5s %4s %4s %4s %4s %4s %6s'
          % ('ang', 'cam', 'area', 'w', 'h', 'y', 'V10', 'V50', 'V90', 'V<=70'))
    worst = []
    for i in range(12):
        ang = i * math.pi / 6.0
        rx, ry = PX + 1.2 * math.cos(ang), PY + 1.2 * math.sin(ang)
        set_pose('robot', rx, ry, 0.15, ang + math.pi)
        time.sleep(1.4)
        for c in CAMS:
            if c not in state:
                continue
            b = best_blob(state[c])
            if b is None:
                continue
            print('%-6.0f %-6s %-6d %5d %4d %4d %4d %4d %4d %6d'
                  % (math.degrees(ang), c, b['a'], b['w'], b['h'], b['y'],
                     b['v10'], b['v50'], b['v90'], b['dark70']))
            worst.append(b['v90'])
    if worst:
        print('全部样本 V90: 中位 %d 最大 %d' %
              (int(np.median(worst)), max(worst)))


main()
