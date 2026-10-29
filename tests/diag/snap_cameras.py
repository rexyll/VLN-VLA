#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_snap4.py —— 抓 4 相机 RGB+深度存盘, 供离线复现检测器

用法: python3 tmp_snap4.py <tag> [frames=6]
"""
import os
import sys
import time

import numpy as np

import gz.transport13 as gz_transport
from gz.msgs10.image_pb2 import Image as GzImage

CAMS = ('front', 'left', 'right', 'back')
tag = sys.argv[1] if len(sys.argv) > 1 else 's'
nf = int(sys.argv[2]) if len(sys.argv) > 2 else 6
os.makedirs('cap_' + tag, exist_ok=True)

node = gz_transport.Node()
state = {}


def mk(n, kind):
    def cb(msg):
        h, w = msg.height, msg.width
        if h < 10 or w < 10:
            return
        d = msg.data
        if kind == 'rgb':
            if len(d) == w * h * 3:
                img = np.frombuffer(d, dtype=np.uint8).reshape(h, w, 3)
            elif len(d) == w * h * 4:
                img = np.frombuffer(d, dtype=np.uint8).reshape(h, w, 4)[:, :, :3]
            else:
                return
        else:
            if len(d) < w * h * 4:
                return
            img = np.frombuffer(d[:w * h * 4], dtype='<f4').reshape(h, w)
        state[(n, kind)] = img
    return cb


for c in CAMS:
    node.subscribe(GzImage, '/depth_%s/image' % c, mk(c, 'rgb'))
    node.subscribe(GzImage, '/depth_%s/depth_image' % c, mk(c, 'dep'))

t0 = time.time()
saved = 0
while time.time() - t0 < 12.0 and saved < nf:
    time.sleep(0.5)
    ok = all((c, k) in state for c in CAMS for k in ('rgb', 'dep'))
    if not ok:
        continue
    for c in CAMS:
        np.save('cap_%s/%s_rgb_%d.npy' % (tag, c, saved), state[(c, 'rgb')])
        np.save('cap_%s/%s_dep_%d.npy' % (tag, c, saved), state[(c, 'dep')])
    saved += 1
    print('saved', saved, flush=True)
print('DONE', saved)
