#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""wrist_view.py —— 腕部/掌侧两套 RGB-D 的**实时**预览窗口(给人在屏幕上看)。

与 `wrist_cam_shot.py` 的区别: 那个是抓一帧落盘做判据, 这个是常驻窗口, 5Hz
跟着仿真走 —— 调机位、看抓取时手指怎么合、看壶把有没有进画面, 都靠它。

窗口布局(2x2, 每格按 SCALE 放大; 两套相机都是 4:3, 换套不用改窗口尺寸):
    +-----------+-----------+
    |  R rgb    |  L rgb    |
    +-----------+-----------+
    |  R depth  |  L depth  |      depth: 近=黑 远=白(0.05~1.0m 线性)
    +-----------+-----------+

两套机位(按键切换):
    1 = wrist_cam  P99 手背相机 256x192, 看手前方工作区
    2 = side_cam   P100 掌侧机位 192x144, 看指间抓取特写

按键:  q/ESC 退出   1/2 换相机套   d 只留 rgb   s 存一张 PNG   + / - 缩放

为什么走 gz 原生话题而不是 ROS:
  腕部相机是掌 link 内的 rgbd_camera, 桥接成 ROS 话题只是给节点用; 看图不必
  经过 ros_gz_bridge, 少一层延迟和一次 launch。

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

# 套名 -> 话题前缀; 名称同时用在标题栏与存图文件名上
SETS = (('wrist', 'wrist_cam'), ('side', 'side_cam'))


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


def main():
    global SCALE
    os.environ.setdefault('DISPLAY', ':0')
    node = gz_transport.Node()
    box = {}          # 'set_side_kind' -> (msg, t)
    hist = {}         # 同上 -> [最近若干到达时刻], 用来算真实帧率

    def mk(key):
        def cb(m):
            t = time.time()
            box[key] = (m, t)
            h = hist.setdefault(key, [])
            h.append(t)
            del h[:-8]
        return cb

    for setname, prefix in SETS:
        for s in ('R', 'L'):
            node.subscribe(image_pb2.Image, '/%s_%s/image' % (prefix, s),
                           mk('%s_%s_rgb' % (setname, s)))
            node.subscribe(image_pb2.Image, '/%s_%s/depth_image' % (prefix, s),
                           mk('%s_%s_dep' % (setname, s)))

    def rate(key):
        h = hist.get(key)
        if not h or len(h) < 2:
            return 0.0
        span = h[-1] - h[0]
        return (len(h) - 1) / span if span > 1e-3 else 0.0

    cur = os.environ.get('WRIST_VIEW_SET', 'wrist')
    if cur not in dict(SETS):
        cur = 'wrist'
    win = 'cams  (q quit | 1 wrist 2 side | d depth | s shot | +/- zoom)'
    # WINDOW_GUI_NORMAL: 关掉 cv2-Qt 自带的工具栏(那排放大镜图标), 否则它
    # 会把画布顶下去、窗口尺寸也不再等于画布尺寸。
    cv2.namedWindow(win, cv2.WINDOW_GUI_NORMAL | cv2.WINDOW_NORMAL)
    shot_n = 0
    size = (256, 192)
    show_depth = SHOW_DEPTH
    canvas = None
    last_sig = None

    def handle(k):
        """按键 -> 动作字符串(或 None 表示不用重画)。"""
        if k in (ord('q'), 27):
            return 'quit'
        if k == ord('1'):
            return 'switch:wrist'
        if k == ord('2'):
            return 'switch:side'
        if k == ord('d'):
            return 'depth'
        if k in (ord('+'), ord('=')):
            return 'zoom+'
        if k == ord('-'):
            return 'zoom-'
        if k == ord('s'):
            return 'shot'
        return None

    while True:
        # 只在"有新帧/该重画"时才算: 相机 5Hz, 而无脑按 25Hz 重画会把 CPU 抢走
        # —— 实测 RTF 从 1.00 掉到 0.78, 有一半是这个窗口的。
        sig = (cur, tuple(sorted((k, v[1]) for k, v in box.items())))
        if sig == last_sig and canvas is not None:
            k = handle(cv2.waitKey(30) & 0xFF)
            if k == 'quit':
                break
            if k == 'shot':
                p = os.path.join(OUT, 'view_%s_%02d.png' % (cur, shot_n))
                cv2.imwrite(p, canvas)
                shot_n += 1
                print('>>> 存图 %s' % p, flush=True)
            elif k and k.startswith('switch:'):
                cur = k.split(':', 1)[1]
                canvas = None
            elif k == 'depth':
                show_depth = not show_depth
                canvas = None
            elif k == 'zoom+':
                SCALE = min(3.0, SCALE * 1.25)
                canvas = None
            elif k == 'zoom-':
                SCALE = max(0.6, SCALE / 1.25)
                canvas = None
            continue
        last_sig = sig

        cells = []
        for s in ('R', 'L'):
            for kind in ('rgb', 'dep'):
                if kind == 'dep' and not show_depth:
                    continue
                key = '%s_%s_%s' % (cur, s, kind)
                e = box.get(key)
                if e is None or time.time() - e[1] > 2.0:
                    cells.append(('blank', None, s, kind, *size))
                else:
                    m = e[0]
                    size = (m.width, m.height)
                    cells.append((kind, (rgb_of(m) if kind == 'rgb'
                                         else depth_of(m)), s, kind,
                                  m.width, m.height))

        W = max(64, int(size[0] * SCALE))
        H = max(48, int(size[1] * SCALE))
        rows = []
        for i in range(0, len(cells), 2):
            row = []
            for kind, arr, s, kd, w, h in cells[i:i + 2]:
                hz = '%.1fHz' % rate('%s_%s_%s' % (cur, s, kd))
                if kind == 'blank':
                    c = blank(W, H, '%s %s  no frame' % (s, kd))
                elif kind == 'rgb':
                    c = cv2.resize(cv2.cvtColor(arr, cv2.COLOR_RGB2BGR),
                                   (W, H), interpolation=cv2.INTER_NEAREST)
                    label(c, '%s %s rgb %dx%d %s' % (cur, s, w, h, hz))
                else:
                    dm = depth_u8(arr)
                    c = cv2.applyColorMap(cv2.resize(
                        dm, (W, H), interpolation=cv2.INTER_NEAREST),
                        cv2.COLORMAP_BONE)
                    v = arr[(arr > NEAR + 0.01) & (arr < FAR)]
                    rng = ('%.2f~%.2fm' % (np.percentile(v, 5),
                                           np.percentile(v, 95))
                           if v.size > 50 else 'no valid')
                    label(c, '%s %s depth %s %s' % (cur, s, hz, rng))
                row.append(c)
            if len(row) == 1:
                row.append(np.full_like(row[0], 45))
            rows.append(np.hstack(row))
        canvas = np.vstack(rows)
        cv2.imshow(win, canvas)

        k = handle(cv2.waitKey(40) & 0xFF)
        if k == 'quit':
            break
        if k == 'shot':
            p = os.path.join(OUT, 'view_%s_%02d.png' % (cur, shot_n))
            cv2.imwrite(p, canvas)
            shot_n += 1
            print('>>> 存图 %s' % p, flush=True)
        elif k and k.startswith('switch:'):
            cur = k.split(':', 1)[1]
            canvas = None
        elif k == 'depth':
            show_depth = not show_depth
        elif k == 'zoom+':
            SCALE = min(3.0, SCALE * 1.25)
        elif k == 'zoom-':
            SCALE = max(0.6, SCALE / 1.25)

    cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    sys.exit(main())
