#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""wrist_cam_shot.py —— 腕部 RGB-D 相机抓帧落盘(走 gz 原生话题, 不经 ROS 桥)。

为什么要这个工具
================
`/wrist_cam_R|L` 是挂在**掌 link 内部**的 rgbd_camera(见
`tools/gen_drink_scene.py::_palm_link`)。它的用途是"眼在手"近距观察:
手掌、手指、壶把、杯口都在 0.05~0.5 m 内。要判断机位/朝向对不对, 只能看
实际画面 —— 但腕部图像不一定要进 ROS(摆位调试时不想为了看一眼图就起
ros_gz_bridge 和整套节点)。

本工具直接 `gz.transport13` 订阅 4 路话题, 取一帧存 PNG:
  rgb   -> <name>_R.png / <name>_L.png   (256x192 rgb8)
  depth -> <name>_R_depth.png             (16 位归一化灰度, near/far 线性映射)

用法:
  python3 tools/wrist_cam_shot.py [前缀, 默认 wrist]
  python3 tools/wrist_cam_shot.py wrist --side R     # 只抓右手
"""
import os
import struct
import sys
import time
import zlib

import numpy as np

import gz.transport13 as gz_transport
import gz.msgs10.image_pb2 as image_pb2

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, 'vla_ws', 'logs')


def _chunk(tag, data):
    return (struct.pack('>I', len(data)) + tag + data
            + struct.pack('>I', zlib.crc32(tag + data) & 0xFFFFFFFF))


def write_png(path, arr):
    """arr: (h,w) uint8 或 (h,w,3) uint8 —— 零依赖写 PNG。"""
    a = np.ascontiguousarray(arr)
    if a.ndim == 2:
        a = np.stack([a, a, a], axis=-1)
    elif a.shape[2] == 4:
        a = a[:, :, :3]
    h, w = a.shape[:2]
    raw = b''.join(b'\x00' + a[y].tobytes() for y in range(h))
    with open(path, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n')
        f.write(_chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0)))
        f.write(_chunk(b'IDAT', zlib.compress(raw, 6)))
        f.write(_chunk(b'IEND', b''))
    return w, h


def grab(node, topic, timeout=8.0):
    box = {}

    def cb(m):
        if 'm' not in box:
            box['m'] = m
    node.subscribe(image_pb2.Image, topic, cb)
    t0 = time.time()
    while 'm' not in box and time.time() - t0 < timeout:
        time.sleep(0.05)
    return box.get('m')


def rgb_of(m):
    """gz Image -> (h,w,3) uint8。`<format>R8G8B8</format>` 出来的就是 RGB。

    别按 `pixel_format_type` 翻通道: gz 的枚举里 3 = R_FLOAT32(不是 BGR),
    照它翻会把整幅图换成补色(地面变薰衣草紫)。
    """
    h, w = m.height, m.width
    a = np.frombuffer(m.data, dtype=np.uint8)
    ch = a.size // (w * h)
    return a.reshape(h, w, ch)[:, :, :3]


def depth_of(m, near=0.05, far=2.0):
    """gz 32FC1 深度 -> 归一化 uint8(near=黑, far=白, 便于看图)。"""
    h, w = m.height, m.width
    f = np.frombuffer(m.data, dtype=np.float32)
    if f.size != w * h:                   # 个别版本给 16UC1(mm)
        f = np.frombuffer(m.data, dtype=np.uint16).astype(np.float32) / 1000.0
    d = np.nan_to_num(f.reshape(h, w), nan=far, posinf=far, neginf=near)
    d = np.clip((d - near) / (far - near), 0.0, 1.0)
    return (d * 255).astype(np.uint8), f.reshape(h, w)


def main():
    prefix = sys.argv[1] if len(sys.argv) > 1 else 'wrist'
    sides = [sys.argv[3]] if len(sys.argv) > 3 and sys.argv[2] == '--side' \
        else ['R', 'L']
    os.makedirs(OUT, exist_ok=True)
    node = gz_transport.Node()
    ok = True
    for side in sides:
        m = grab(node, '/wrist_cam_%s/image' % side)
        if m is None:
            print('[%s] rgb 无帧' % side)
            ok = False
        else:
            p = os.path.join(OUT, '%s_%s.png' % (prefix, side))
            w, h = write_png(p, rgb_of(m))
            print('[%s] rgb   %dx%d -> %s' % (side, w, h, p))
        d = grab(node, '/wrist_cam_%s/depth_image' % side)
        if d is None:
            print('[%s] depth 无帧' % side)
            ok = False
        else:
            img, raw = depth_of(d)
            p = os.path.join(OUT, '%s_%s_depth.png' % (prefix, side))
            write_png(p, img)
            valid = raw[raw > 0.06]
            rng = ('%.3f~%.3f m' % (valid.min(), valid.max())) \
                if valid.size else '全无效'
            print('[%s] depth %dx%d 有效 %d%% 范围 %s -> %s'
                  % (side, d.width, d.height,
                     100 * valid.size / raw.size, rng, p))
    return 0 if ok else 2


if __name__ == '__main__':
    sys.exit(main())
