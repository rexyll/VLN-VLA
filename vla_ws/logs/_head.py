#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""wrist_view.py —— 腕部两路 RGB-D 的**实时**预览窗口(给人在屏幕上看)。

与 `wrist_cam_shot.py` 的区别: 那个是抓一帧落盘做判据, 这个是常驻窗口, 5Hz
跟着仿真走 —— 调机位、看抓取时手指怎么合、看壶把有没有进画面, 都靠它。

窗口布局(默认 2x2, 每个格子 256x192 放大 1.4 倍):
    +-----------+-----------+
    |  R rgb    |  L rgb    |
    +-----------+-----------+
    |  R depth  |  L depth  |      depth: 近=黑 远=白(0.05~1.0m 线性)
    +-----------+-----------+

按键:  q / ESC 退出    d 只留 rgb(去掉深度行)    s 存一张 PNG 到 vla_ws/logs/
       + / -  缩放(默认 1.4 倍; 屏幕小就按 -)

为什么走 gz 原生话题而不是 ROS:
  腕部相机是掌 link 内的 rgbd_camera, 桥接成 ROS 话题只是给节点用; 看图不必
  经过 ros_gz_bridge, 少一层延迟和一次 launch。什么时候需要 ROS 侧: 抓取
  闭环(见 TaskPlan §6.7)。

注意 gz Image 的坑(§7.1 #71): `pixel_format_type == 3` 是 **R_FLOAT32**, 不是
BGR。别按它翻通道, 会把地面变成薰衣草紫。RGB 直接按 R8G8B8 读 3 通道即可。
"""
import os
import sys
import time

import numpy as np

import cv2
import gz.transport13 as gz_transport
import gz.msgs10.image_pb2 as image_pb2

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, 'vla_ws', 'logs')

SCALE = float(os.environ.get('WRIST_VIEW_SCALE', '1.4'))
SHOW_DEPTH = os.environ.get('WRIST_VIEW_DEPTH', '1') != '0'
NEAR, FAR = 0.05, 1.00


def rgb_of(m):
    """gz Image(R8G8B8) -> (h,w,3) uint8, RGB 顺序。"""
    h, w = m.height, m.width
    a = np.frombuffer(m.data, dtype=np.uint8)
    ch = a.size // (w * h)
    return a.reshape(h, w, ch)[:, :, :3].copy()


def depth_of(m):
    """gz 32FC1 -> (h,w) float32, 单位 m。"""
    h, w = m.height, m.width
    f = np.frombuffer(m.data, dtype=np.float32)
    if f.size != w * h:
        f = np.frombuffer(m.data, dtype=np.uint16).astype(np.float32) / 1000.0
    return f.reshape(h, w).copy()


def depth_u8(d):
    x = np.nan_to_num(d, nan=FAR, posinf=FAR, neginf=NEAR)
    x = np.clip((x - NEAR) / (FAR - NEAR), 0.0, 1.0)
    return (x * 255).astype(np.uint8)


def label(img, txt, ok=True):
    cv2.rectangle(img, (0, 0), (img.shape[1], 18), (28, 28, 28), -1)
    cv2.putText(img, txt, (5, 13), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                (90, 230, 120) if ok else (90, 120, 240), 1, cv2.LINE_AA)
    return img


def blank(w, h, txt):
    img = np.full((h, w, 3), 45, np.uint8)
    cv2.putText(img, txt, (8, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (120, 120, 120), 1, cv2.LINE_AA)
    return img


