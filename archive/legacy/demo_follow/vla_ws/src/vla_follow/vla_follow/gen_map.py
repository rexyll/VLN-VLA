#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gen_map.py —— 由已知世界几何生成 Nav2 静态地图 (arena.pgm + arena.yaml)

几何与 worlds/work_area.sdf 一致: 12.3m 地板, 内墙 ±6.0, 7 个障碍物。
用法: python3 gen_map.py <输出目录>   (默认 $VLA_HOME/vla_demo/maps)
"""
import json
import math
import os
import sys

import numpy as np
import cv2

RES = 0.05
WORLD = 12.3           # 地板尺寸
HALF = WORLD / 2.0     # 6.15
N = int(round(WORLD / RES))          # 246
WALL_IN = 6.0          # 墙内侧面

# (类型, x, y, 尺寸, yaw) —— 与 SDF 一致
OBSTACLES = [
    ('box', 3.5, 2.0, (0.8, 0.8), 0.0),
    ('box', -3.0, 3.2, (1.2, 0.6), 0.5),
    ('cyl', 0.5, -3.2, 0.4, 0.0),
    ('cyl', -4.2, -2.5, 0.5, 0.0),
    ('box', 4.2, -3.0, (0.6, 0.6), 0.0),
    ('box', -1.5, -4.5, (1.0, 0.5), -0.4),
    ('cyl', 2.2, 4.6, 0.35, 0.0),
]


def to_px(x, y):
    """世界→像素(col,row); 图像 row0 = y 最大"""
    col = int(round((x + HALF) / RES))
    row = int(round((HALF - y) / RES))
    return col, row


def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else \
        os.path.join(os.environ.get('VLA_HOME',
                                    '/home/earl/workspace/VLA'),
                     'vla_demo', 'maps')
    os.makedirs(out_dir, exist_ok=True)
    img = np.full((N, N), 255, np.uint8)   # 全白=自由
    # 墙: 内侧面 ±6.0 到边缘
    w0, w1 = to_px(-HALF, -HALF), to_px(HALF, HALF)
    wt = int(round(0.15 / RES))            # 墙厚 0.15
    inner = to_px(-WALL_IN, WALL_IN)
    img[inner[1] - 1:, :] = 0              # 南墙带
    img[:inner[1] + wt, :] = 0
    # 精确画四条墙带
    img = np.full((N, N), 255, np.uint8)
    c_in = WALL_IN
    for (x0, y0, x1, y1) in [(-c_in, -c_in, c_in, -c_in),
                             (-c_in, c_in, c_in, c_in),
                             (-c_in, -c_in, -c_in, c_in),
                             (c_in, -c_in, c_in, c_in)]:
        p0, p1 = to_px(x0, y0), to_px(x1, y1)
        cv2.line(img, p0, p1, 0, wt + 2)
    # 障碍物
    for typ, x, y, size, yaw in OBSTACLES:
        c, r = to_px(x, y)
        if typ == 'cyl':
            rr = int(round(size / RES / 2))
            cv2.circle(img, (c, r), rr, 0, -1)
        else:
            w, h = size
            pts = [(-w / 2, -h / 2), (w / 2, -h / 2),
                   (w / 2, h / 2), (-w / 2, h / 2)]
            cs, sn = math.cos(yaw), math.sin(yaw)
            pix = []
            for dx, dy in pts:
                wx = x + dx * cs - dy * sn
                wy = y + dx * sn + dy * cs
                pix.append(to_px(wx, wy))
            cv2.fillPoly(img, [np.array(pix)], 0)
    pgm = os.path.join(out_dir, 'arena.pgm')
    cv2.imwrite(pgm, img)
    yaml = os.path.join(out_dir, 'arena.yaml')
    meta = {'image': 'arena.pgm', 'resolution': RES,
            'origin': [-HALF, -HALF, 0.0], 'negate': 0,
            'occupied_thresh': 0.65, 'free_thresh': 0.196}
    with open(yaml, 'w', encoding='utf-8') as f:
        f.write(json.dumps if False else '')
    with open(yaml, 'w', encoding='utf-8') as f:
        f.write('image: arena.pgm\nresolution: %g\norigin: [%.3f, %.3f, 0.0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n'
                % (RES, -HALF, -HALF))
    occ = int((img == 0).sum())
    print('MAP-GEN: %s (%dx%d, 占用栅格 %d)' % (pgm, N, N, occ))


if __name__ == '__main__':
    main()
