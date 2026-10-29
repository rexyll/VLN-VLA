#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""dbg_cam_sweep.py —— 一次性抓全部 /dbg_cam_* 画面(机位标定用, gz 原生订阅)。"""
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, 'tools'))

import gz.transport13 as gz_transport                      # noqa: E402
import gz.msgs10.image_pb2 as image_pb2                    # noqa: E402

from wrist_cam_shot import write_png                       # noqa: E402

TAGS = ('id', 'pz', 'mz', 'pz_y90', 'rz')


def main():
    node = gz_transport.Node()
    boxes = {t: {} for t in TAGS}

    def mk(t):
        def cb(m):
            boxes[t].setdefault('m', m)
        return cb

    for t in TAGS:
        node.subscribe(image_pb2.Image, '/dbg_cam_%s/image' % t, mk(t))
    t0 = time.time()
    while time.time() - t0 < 10.0 and any('m' not in b for b in boxes.values()):
        time.sleep(0.1)
    out = os.path.join(ROOT, 'vla_ws', 'logs')
    for t in TAGS:
        m = boxes[t].get('m')
        if m is None:
            print('[%-7s] 无帧' % t)
            continue
        a = np.frombuffer(m.data, dtype=np.uint8)
        ch = a.size // (m.width * m.height)
        a = a.reshape(m.height, m.width, ch)[:, :, :3]
        p = os.path.join(out, 'dbg_%s.png' % t)
        write_png(p, a)
        # 顺带统计"非背景色"像素占比, 便于判断有没有拍到东西
        flat = a.reshape(-1, 3).astype(np.int16)
        uniq = len(np.unique(flat, axis=0))
        print('[%-7s] %dx%d 颜色数 %-5d -> %s' % (t, m.width, m.height, uniq, p))
    return 0


if __name__ == '__main__':
    sys.exit(main())
