#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_depth_diag.py —— 深度相机虚警根因诊断

现象: depth_percept 报 front 测 0.7m / 真值 2.7m, back 测 2.2 / 真值 9.5。
要确认 0.7m 到底是真实回波、边缘伪影, 还是行带裁剪把地面/自身算进来了。

输出:
  1) 帧元信息(尺寸/字节数/推断像素格式)
  2) 逐行(按 10 行一档)的有效点比例与 5% 分位数 —— 看虚警集中在哪几行
  3) 逐列(按 20 列一档)同理 —— 看是否集中在画面边缘
  4) 当前使用行带 10%~45% 的逐扇区结果
  5) 最小 20 个像素的 (行,列,深度) —— 直接指出污染源位置
"""
import math
import sys
import time

import numpy as np
import gz.transport13 as gz_transport
from gz.msgs10.image_pb2 import Image as GzImage

CAM = sys.argv[1] if len(sys.argv) > 1 else 'front'

got = {'ok': False}


def cb(msg):
    if got['ok']:
        return
    h, w = msg.height, msg.width
    n = len(msg.data)
    print('=' * 78)
    print(f'帧: {msg.width}x{msg.height}  data={n}B  '
          f'B/px={n / max(h * w, 1):.3f}')
    if n >= h * w * 4:
        a = np.frombuffer(msg.data[:h * w * 4], dtype='<f4').reshape(h, w)
        tag = '32FC1 深度'
    elif n >= h * w * 3:
        a = np.frombuffer(msg.data[:h * w * 3], dtype='u1').reshape(h, w, 3)
        tag = '8UC3 彩色(非深度)'
        print(f'格式: {tag}')
        got['ok'] = True
        return
    else:
        print(f'未知格式 n={n}')
        got['ok'] = True
        return
    print(f'格式: {tag}')
    ok = np.isfinite(a) & (a > 0.05) & (a < 11.5)
    fin = np.isfinite(a)
    print(f'| 有限值 {fin.sum()}/{a.size} | 有效(0.05~11.5) {ok.sum()} '
          f'| inf {np.isinf(a).sum()} | <0.05 {(fin & (a <= 0.05)).sum()}')
    if ok.sum() < 60:
        print('有效点不足, 非深度帧')
        got['ok'] = True
        return
    v = a[ok]
    print(f'分位数  1%={np.quantile(v,0.01):.3f} 5%={np.quantile(v,0.05):.3f} '
          f'25%={np.quantile(v,0.25):.3f} 50%={np.quantile(v,0.5):.3f} '
          f'max={v.max():.2f}')

    print('\n--- 逐行(10 行一档): 有效点比例 / 5%分位 / 中位 ---')
    for r in range(0, h, 10):
        blk = a[r:r + 10]
        m = ok[r:r + 10]
        if m.sum() < 15:
            continue
        val = blk[m]
        deg = (h / 2.0 - (r + 5)) / h * 83.58   # 仰角≈+, 俯角≈-
        print(f'  行{r:3d}-{r + 9:3d} (仰角{deg:+6.1f}°) '
              f'有效{m.sum():5d}  '
              f'p5={np.quantile(val,0.05):6.3f}  中位={np.median(val):7.3f}')

    print('\n--- 逐列(20 列一档): 有效点比例 / 5%分位 / 中位 ---')
    for c in range(0, w, 20):
        blk = a[:, c:c + 20]
        m = ok[:, c:c + 20]
        if m.sum() < 15:
            continue
        val = blk[m]
        deg = (c + 10 - w / 2.0) / w * 100.0    # 左负右正(图像坐标)
        print(f'  列{c:3d}-{c + 19:3d} ({deg:+6.1f}°) '
              f'有效{m.sum():5d}  '
              f'p5={np.quantile(val,0.05):6.3f}  中位={np.median(val):7.3f}')

    r0, r1 = int(h * 0.10), int(h * 0.45)
    print(f'\n--- 当前行带 10%~45% (行 {r0}~{r1 - 1}) 逐扇区 5%分位 ---')
    for k in range(5):
        c0, c1 = int(w * k / 5), int(w * (k + 1) / 5)
        sub = a[r0:r1, c0:c1]
        m = ok[r0:r1, c0:c1]
        val = sub[m]
        if val.size < 15:
            print(f'  扇区{k}: 有效点不足 {val.size}')
            continue
        ctr = (c0 + c1) / 2.0
        deg = (ctr - w / 2.0) / w * 100.0
        print(f'  扇区{k} 列{c0:3d}-{c1 - 1:3d} 中心{deg:+6.1f}° '
              f'有效{m.sum():5d}  p5={np.quantile(val,0.05):6.3f} '
              f'p10={np.quantile(val,0.10):6.3f} 中位={np.median(val):7.3f}')

    print('\n--- 最小 20 个有效像素 (行,列,深度,方位°) ---')
    idx = np.argwhere(ok)
    vals = a[ok]
    order = np.argsort(vals)[:20]
    for i in order:
        r, c = idx[i]
        deg = (c - w / 2.0) / w * 100.0
        ele = (h / 2.0 - r) / h * 83.58
        print(f'  行{r:3d} 列{c:3d} 深度={vals[i]:.4f} '
              f'方位{deg:+6.1f}° 仰角{ele:+6.1f}°')
    got['ok'] = True


node = gz_transport.Node()
for topic in (f'/depth_{CAM}', f'/depth_{CAM}/depth_image'):
    node.subscribe(GzImage, topic, cb)

t0 = time.time()
while time.time() - t0 < 8.0 and not got['ok']:
    time.sleep(0.05)
if not got['ok']:
    print('!! 8s 内未收到深度帧')
