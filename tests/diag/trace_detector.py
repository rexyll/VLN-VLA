#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_det_trace.py —— 离线复现 person_vision._detect_cam 并打印每个候选的判决

用法: python3 tmp_det_trace.py <cap_dir> [frame]
输出: 每个相机每个候选的 area/aspect/深度/几何一致性 与最终判决;
      并把暗色 mask 叠图存为 trace_<cam>.png
"""
import math
import sys

import cv2
import numpy as np

DARK_V_MAX = 70
SAT_MAX = 100
AREA_MIN, AREA_MAX = 20, 22000
ASPECT_MIN, ASPECT_MAX = 0.2, 2.8
HFOV = 1.8
DEPTH_MIN, DEPTH_MAX = 0.28, 7.0
W_REAL, W_TOL = 0.36, 0.62

d = sys.argv[1]
frame = sys.argv[2] if len(sys.argv) > 2 else '0'


def run(cam):
    rgb = np.load('%s/%s_rgb_%s.npy' % (d, cam, frame))
    dep = np.load('%s/%s_dep_%s.npy' % (d, cam, frame))
    h, w = rgb.shape[:2]
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, (0, 0, 0), (179, SAT_MAX, DARK_V_MAX))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                               cv2.CHAIN_APPROX_SIMPLE)
    print('--- [%s] %dx%d 暗色连通块 %d 个' % (cam, w, h, len(cnts)))
    tan_h = math.tan(HFOV / 2.0)
    vis = rgb.copy()
    ok_any = None
    for c in sorted(cnts, key=cv2.contourArea, reverse=True)[:6]:
        a = cv2.contourArea(c)
        x, y, ww, hh = cv2.boundingRect(c)
        asp = ww / max(hh, 1)
        y0 = y + int(hh * 0.15)
        y1 = y + int(hh * 0.60)
        sub = dep[y0:max(y1, y0 + 1), x:x + ww]
        valid = sub[np.isfinite(sub) & (sub > DEPTH_MIN) & (sub < DEPTH_MAX)]
        dist = float(np.quantile(valid, 0.30)) if valid.size else -1.0
        d_geo = W_REAL * w / max(2.0 * ww * tan_h, 1e-6)
        why = []
        if a < AREA_MIN:
            why.append('面积<min')
        if a > AREA_MAX:
            why.append('面积>max')
        if not (ASPECT_MIN <= asp <= ASPECT_MAX):
            why.append('宽高比%.2f越界' % asp)
        if valid.size < 10:
            why.append('深度有效点%d' % valid.size)
        elif dist < d_geo * (1 - W_TOL) or dist > d_geo * (1 + W_TOL):
            why.append('几何不一致(实测%.2f vs 反推%.2f)' % (dist, d_geo))
        verdict = 'PASS' if not why else 'reject: ' + ','.join(why)
        if not why and ok_any is None:
            ok_any = (a, x, y, ww, hh, dist)
        print('   bbox=(%3d,%3d,%3d,%3d) area=%5d asp=%.2f d=%.2f dgeo=%.2f'
              ' depth_n=%4d -> %s'
              % (x, y, ww, hh, int(a), asp, dist, d_geo, valid.size, verdict))
        col = (0, 255, 0) if not why else (255, 0, 0)
        cv2.rectangle(vis, (x, y), (x + ww, y + hh), col, 1)
    if ok_any is None:
        print('   => 整帧判未检出')
    else:
        a, x, y, ww, hh, dist = ok_any
        u = x + ww / 2.0
        print('   => 检出 dist=%.2f bc=%+.1f°' %
              (dist, math.degrees((w / 2.0 - u) * HFOV / w)))
    cv2.imwrite('%s/trace_%s.png' % (d, cam),
                cv2.cvtColor(vis, cv2.COLOR_RGB2BGR))


for cam in ('front', 'left', 'right', 'back'):
    try:
        run(cam)
    except Exception as e:
        print(cam, 'ERR', e)
