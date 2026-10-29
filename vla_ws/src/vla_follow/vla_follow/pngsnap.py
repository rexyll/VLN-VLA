#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pngsnap.py —— 把 `sensor_msgs/Image` 直接落盘成 PNG（P88 提取）

为什么不走 cv_bridge / PIL
=========================
验收脚本要能在**只装了 ROS 基础包**的环境里跑。`cv_bridge` 依赖 OpenCV 的
Python 绑定(本机装的是 ROS 自带的, 版本一换就 import 失败), PIL 干脆没装。
而"存一张 rgb8/bgr8 的图"只需要 zlib + struct —— PNG 的 IDAT 就是 zlib 流,
每行前面加一个 filter 字节 0(不过滤)即可, 三十行写完, 零依赖。

用法:
    from vla_follow.pngsnap import save_png
    save_png('out.png', img.data, img.width, img.height, img.encoding)
"""
import os
import struct
import zlib

import numpy as np


def _chunk(tag, data):
    return (struct.pack('>I', len(data)) + tag + data
            + struct.pack('>I', zlib.crc32(tag + data) & 0xFFFFFFFF))


def save_png(path, buf, w, h, enc='rgb8'):
    """把 ROS Image 的 buffer 存成 PNG, 返回 (h,w,3) 的 ndarray(方便看颜色数)。"""
    a = np.frombuffer(buf, dtype=np.uint8).reshape(h, w, -1)[:, :, :3]
    if enc == 'bgr8':
        a = a[:, :, ::-1]
    raw = b''.join(b'\x00' + a[r].tobytes() for r in range(h))
    png = (b'\x89PNG\r\n\x1a\n'
           + _chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
           + _chunk(b'IDAT', zlib.compress(raw, 6))
           + _chunk(b'IEND', b''))
    d = os.path.dirname(path)
    if d:
        os.makedirs(d, exist_ok=True)
    with open(path, 'wb') as fh:
        fh.write(png)
    return a
