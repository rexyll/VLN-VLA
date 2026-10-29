#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_vis_dump.py —— 抓一帧 front 相机, 复现 person_vision 的检测,
把 RGB / 深度 / mask / bbox 存图, 并列出所有候选轮廓的几何与深度。

用法: python3 tmp_vis_dump.py [front|left|right|back] [tag]
"""
import math
import sys
import time

import numpy as np
import cv2

from gz.msgs10.image_pb2 import Image as GzImage
import gz.transport13 as gz_transport

DARK_V_MAX, SAT_MAX = 70, 100
AREA_MIN, AREA_MAX = 45, 22000
ASPECT_MIN, ASPECT_MAX = 0.2, 2.8
HFOV = 1.7453293
DEPTH_MIN, DEPTH_MAX = 0.45, 11.5


def main():
    cam = sys.argv[1] if len(sys.argv) > 1 else 'front'
    tag = sys.argv[2] if len(sys.argv) > 2 else cam
    st = {'rgb': None, 'dep': None, 't': 0.0}

    def cb_rgb(msg):
        h, w = msg.height, msg.width
        n = len(msg.data)
        if n == w * h * 3:
            st['rgb'] = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
        elif n == w * h * 4:
            st['rgb'] = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                h, w, 4)[:, :, :3]
        st['t'] = time.monotonic()

    def cb_dep(msg):
        h, w = msg.height, msg.width
        if len(msg.data) < w * h * 4:
            return
        st['dep'] = np.frombuffer(msg.data[:w * h * 4],
                                  dtype='<f4').reshape(h, w)

    node = gz_transport.Node()
    node.subscribe(GzImage, '/depth_%s/image' % cam, cb_rgb)
    node.subscribe(GzImage, '/depth_%s/depth_image' % cam, cb_dep)
    print('[DUMP] 等待 %s 相机数据...' % cam, flush=True)
    t0 = time.monotonic()
    while st['rgb'] is None or st['dep'] is None:
        if time.monotonic() - t0 > 15:
            print('!! 超时'); return
        time.sleep(0.1)
    time.sleep(0.4)
    rgb, dep = st['rgb'], st['dep']
    h, w = rgb.shape[:2]
    print('[DUMP] rgb=%s dep=%s' % (rgb.shape, dep.shape), flush=True)

    dshow = np.clip(np.nan_to_num(dep, nan=0.0, posinf=0.0), 0, 8) / 8.0
    cv2.imwrite('/home/earl/workspace/VLA/shots/dump_%s_depth.png' % tag,
                (dshow * 255).astype(np.uint8))

    vis = rgb.copy()
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, (0, 0, 0), (179, SAT_MAX, DARK_V_MAX))
    mask_o = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    cv2.imwrite('/home/earl/workspace/VLA/shots/dump_%s_mask.png' % tag, mask_o)

    cnts, _ = cv2.findContours(mask_o, cv2.RETR_EXTERNAL,
                               cv2.CHAIN_APPROX_SIMPLE)
    print('轮廓数: %d' % len(cnts), flush=True)
    rows = []
    for c in cnts:
        a = cv2.contourArea(c)
        if a < 20:
            continue
        x, y, ww, hh = cv2.boundingRect(c)
        asp = ww / max(hh, 1)
        u = x + ww / 2.0
        y0 = y + int(hh * 0.15)
        y1 = y + int(hh * 0.60)
        sub = dep[y0:max(y1, y0 + 1), x:x + ww]
        v = sub[np.isfinite(sub) & (sub > 0.02) & (sub < 20.0)]
        med = float(np.median(v)) if v.size else -1
        p30 = float(np.quantile(v, 0.30)) if v.size else -1
        ok_a = AREA_MIN <= a <= AREA_MAX
        ok_s = ASPECT_MIN <= asp <= ASPECT_MAX
        ok_d = p30 > DEPTH_MIN
        rows.append((a, x, y, ww, hh, asp, u, med, p30, ok_a, ok_s, ok_d))
    rows.sort(reverse=True)
    print('%6s %4s %4s %4s %4s %5s %6s %7s %7s %4s %4s %4s'
          % ('area', 'x', 'y', 'w', 'h', 'asp', 'u', 'medD', 'p30D',
             'A', 'S', 'D'))
    for r in rows[:12]:
        print('%6.0f %4d %4d %4d %4d %5.2f %6.1f %7.2f %7.2f %4d %4d %4d'
              % (r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8],
                 r[9], r[10], r[11]), flush=True)
        col = (0, 255, 0) if (r[9] and r[10] and r[11]) else (255, 0, 0)
        cv2.rectangle(vis, (r[1], r[2]), (r[1] + r[3], r[2] + r[4]), col, 1)
        cv2.putText(vis, '%.1fm' % r[8], (r[1], max(8, r[2] - 2)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, col, 1)
    cv2.imwrite('/home/earl/workspace/VLA/shots/dump_%s_rgb.png' % tag,
                cv2.cvtColor(vis, cv2.COLOR_RGB2BGR))


if __name__ == '__main__':
    main()
