#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_depth_near.py —— 只查"行带内 <0.8m"的回波, 输出其仰角/方位/世界坐标

目的: 确认 P28 把行带下沿从 +4.2° 降到 +1.5° 后, front 左前扇区出现的
0.35m 回波究竟是 (a) 自身部件 还是 (b) 世界中的真实几何。
"""
import math
import time

import numpy as np
import gz.transport13 as gz_transport
from gz.msgs10.image_pb2 import Image as GzImage
from gz.msgs10.pose_v_pb2 import Pose_V as GzPose_V

CAM = 'front'
HFOV, VFOV = 100.0, 83.58
st = {'pose': None, 'done': False}


def pose_cb(m):
    for p in m.pose:
        if p.name.split('/')[-1].split('::')[-1] == 'robot':
            st['pose'] = p


def img_cb(msg):
    if st['done'] or st['pose'] is None:
        return
    h, w = msg.height, msg.width
    if h < 10 or len(msg.data) < h * w * 4:
        return
    a = np.frombuffer(msg.data[:h * w * 4], dtype='<f4').reshape(h, w)
    r0 = int(h * (0.5 - 18.0 / 83.58))
    r1 = int(h * (0.5 - 1.5 / 83.58)) + 1
    print(f'行带 r0={r0} r1={r1}  '
          f'仰角上沿={(h / 2 - r0) / h * VFOV:+.2f}° '
          f'下沿={(h / 2 - (r1 - 1)) / h * VFOV:+.2f}°')
    band = a[r0:r1]
    ok = np.isfinite(band) & (band > 0.05) & (band < 11.5)
    print(f'带内有效点 {int(ok.sum())}/{band.size}')
    m = ok & (band < 0.8)
    print(f'带内 <0.8m 的点 {int(m.sum())}')
    if m.sum() == 0:
        st['done'] = True
        return
    rr, cc = np.nonzero(m)
    d = band[rr, cc]
    ele = (h / 2.0 - (rr + r0)) / h * VFOV
    az = (w / 2.0 - cc) / w * HFOV
    p = st['pose']
    rx, ry = p.position.x, p.position.y
    yaw = math.atan2(2 * (p.orientation.w * p.orientation.z
                          + p.orientation.x * p.orientation.y),
                     1 - 2 * (p.orientation.y ** 2 + p.orientation.z ** 2))
    print(f'robot=({rx:+.2f},{ry:+.2f}) yaw={math.degrees(yaw):+.1f}°')
    print('  仰角范围 %+.2f° ~ %+.2f° | 方位范围 %+.2f° ~ %+.2f° | 深度 %.3f~%.3f'
          % (ele.min(), ele.max(), az.min(), az.max(), d.min(), d.max()))
    # 按方位分 5 个 20° 桶, 每桶看最小深度与位置
    for k in range(5):
        lo = -50 + 20 * k
        hi = lo + 20
        sel = (az >= lo) & (az < hi)
        if sel.sum() == 0:
            continue
        dd = d[sel]
        q = np.quantile(dd, 0.05)
        near = sel & (dd <= q * 1.15)
        gh = dd[near] * np.cos(np.radians(ele[near]))
        gx = rx + gh * np.cos(np.radians(az[near]) + yaw)
        gy = ry + gh * np.sin(np.radians(az[near]) + yaw)
        print(f'  方位[{lo:+3d},{hi:+3d})° 点{int(sel.sum()):5d} p5={q:.3f} '
              f'仰角{pd(ele[near]):+.1f}~{pd2(ele[near]):+.1f}° '
              f'世界≈({gx.mean():+.2f},{gy.mean():+.2f}) '
              f'距机{np.hypot(gx - rx, gy - ry).mean():.2f}m')
    st['done'] = True


def pd(x):
    return float(np.min(x))


def pd2(x):
    return float(np.max(x))


node = gz_transport.Node()
node.subscribe(GzPose_V, '/world/follow_world/dynamic_pose/info', pose_cb)
time.sleep(0.6)
for t in (f'/depth_{CAM}', f'/depth_{CAM}/depth_image'):
    node.subscribe(GzImage, t, img_cb)
t0 = time.time()
while time.time() - t0 < 10 and not st['done']:
    time.sleep(0.05)
