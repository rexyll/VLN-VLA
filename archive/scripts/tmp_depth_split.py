#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_depth_split.py —— 分辨 /depth_<n> 与 /depth_<n>/depth_image 谁是真深度

rgbd_camera 同时发布:
  <topic>              -> RGB  (gazebo 内部常为 RGBA, 4B/px)
  <topic>/depth_image  -> 深度 (32FC1, 4B/px)
两者字节数完全相同(320*240*4=307200), 所以 depth_percept 里
"len(data) < h*w*4 就判为非深度" 的甄别逻辑对 RGBA 失效。

本脚本对每个话题分别统计, 用判据区分:
  * 真深度: 存在大量 >11.5 的远处值; NaN 极少; 分位数随透视平滑变化
  * RGBA 当 float32: 指数位随机 -> 大量 NaN/0/1e-38, 有限值范围怪,
                    且"深度"与像素亮度强相关
"""
import math
import sys
import time

import numpy as np
import gz.transport13 as gz_transport
from gz.msgs10.image_pb2 import Image as GzImage

CAM = sys.argv[1] if len(sys.argv) > 1 else 'front'

seen = {}


def mk(tag):
    def cb(msg):
        if tag in seen:
            return
        h, w = msg.height, msg.width
        n = len(msg.data)
        if h < 10 or n < h * w * 4:
            print(f'[{tag}] 尺寸不足 {w}x{h} n={n} — 跳过')
            return
        a = np.frombuffer(msg.data[:h * w * 4], dtype='<f4')
        a = a.reshape(h, w)
        nan = int(np.isnan(a).sum())
        inf = int(np.isinf(a).sum())
        fin = a[np.isfinite(a)]
        big = int((fin > 11.5).sum())
        tiny = int((fin < 0.05).sum())
        mid = fin[(fin >= 0.05) & (fin <= 11.5)]
        print(f'\n[{tag}] {w}x{h} n={n} 像素格式 {n / (h * w):.2f}B/px')
        print(f'  NaN {nan} ({nan / a.size:.1%})  inf {inf}  '
              f'有限 {fin.size}  其中 >11.5: {big}  <0.05: {tiny}')
        if mid.size:
            q = [np.quantile(mid, x) for x in
                 (0.01, 0.05, 0.25, 0.5, 0.75, 0.95, 0.99)]
            print('  0.05~11.5 区间分位: ' +
                  ' '.join(f'{v:.3f}' for v in q) +
                  f'  max={mid.max():.2f}')
        # 上带(行10%~45%)三扇区 p5, 复现 depth_percept 的算法
        r0, r1 = int(h * 0.10), int(h * 0.45)
        band = a[r0:r1, :]
        okb = np.isfinite(band) & (band > 0.05) & (band < 11.5)
        out = []
        for k in range(5):
            c0, c1 = int(w * k / 5), int(w * (k + 1) / 5)
            v = band[:, c0:c1][okb[:, c0:c1]]
            out.append(f'{np.quantile(v, 0.05):.2f}' if v.size >= 15 else 'NA')
        print('  上带 5 扇区 p5 = ' + ' | '.join(out))
        seen[tag] = True
    return cb


node = gz_transport.Node()
node.subscribe(GzImage, f'/depth_{CAM}', mk('base'))
node.subscribe(GzImage, f'/depth_{CAM}/depth_image', mk('depth_image'))

t0 = time.time()
while time.time() - t0 < 8.0 and len(seen) < 2:
    time.sleep(0.05)
print('\n收到: ' + ', '.join(seen) if seen else '!! 未收到帧')
