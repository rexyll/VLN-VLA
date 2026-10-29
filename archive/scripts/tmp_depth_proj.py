#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_depth_proj.py —— 深度虚警反投影定位

把 depth_front 的近距点云反投影到世界坐标, 聚类后输出疑似障碍物的
中心/半径, 与 world 内已知几何对比, 判断 0.70m 的近物是:
  (a) 世界中的真实障碍   (b) 机器人自身部件自遮挡   (c) 深度图伪影
"""
import math
import sys
import time

import numpy as np
import gz.transport13 as gz_transport
from gz.msgs10.image_pb2 import Image as GzImage
from gz.msgs10.pose_v_pb2 import Pose_V as GzPose_V

CAM = sys.argv[1] if len(sys.argv) > 1 else 'front'
REL = {'front': 0.0, 'left': math.pi / 2,
       'right': -math.pi / 2, 'back': math.pi}[CAM]
HFOV, VFOV = 100.0, 83.58

# world 内已知几何(与 work_area.sdf 一致), 用于比对
KNOWN = [
    ('ob1 box', 3.5, 2.0, 0.57), ('ob2 box', -3.0, 3.2, 0.67),
    ('ob3 cyl', 0.5, -3.2, 0.47), ('ob4 cyl', -4.2, -2.5, 0.57),
    ('ob5 box', 4.2, -3.0, 0.42), ('ob6 box', -1.5, -4.5, 0.62),
    ('ob7 cyl', 2.2, 4.6, 0.42),
]

st = {'pose': None, 'done': False}


def pose_cb(msg):
    for p in msg.pose:
        n = p.name.split('/')[-1].split('::')[-1]
        if 'robot' in n.lower() or 'chassis' in n.lower():
            st['pose'] = (p.position.x, p.position.y, p.position.z,
                          p.orientation.x, p.orientation.y,
                          p.orientation.z, p.orientation.w, n)
            return


def yaw_of(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


def img_cb(msg):
    if st['done'] or st['pose'] is None:
        return
    h, w = msg.height, msg.width
    if h < 10 or len(msg.data) < h * w * 4:
        return
    a = np.frombuffer(msg.data[:h * w * 4], dtype='<f4').reshape(h, w)
    ok = np.isfinite(a) & (a > 0.05) & (a < 11.5)
    if ok.sum() < 60:
        return
    rx, ry, rz, qx, qy, qz, qw, nm = st['pose']
    ryaw = yaw_of(qx, qy, qz, qw)
    head = ryaw + REL
    print(f'机器人 "{nm}" = ({rx:+.2f},{ry:+.2f},{rz:+.2f}) yaw={math.degrees(ryaw):+.1f}°')
    print(f'相机 {CAM} 朝向 {math.degrees(head):+.1f}°  '
          f'(HFOV {HFOV}° VFOV {VFOV}°)')

    # near 集合: 2.0m 以内
    rs, cs = np.nonzero(ok & (a < 2.0))
    vals = a[rs, cs]
    if vals.size == 0:
        print('2m 内无有效点')
        st['done'] = True
        return
    # 针孔反投影: 列 -> 水平角, 行 -> 垂直角
    az = (w / 2.0 - cs) / w * HFOV   # 列0=+Y(左), 已按标定修正
    el = (h / 2.0 - rs) / h * VFOV
    d = vals
    # 水平距离(投影到地面)
    dh = d * np.cos(np.radians(el))
    gx = rx + dh * np.cos(np.radians(az) + head)
    gy = ry + dh * np.sin(np.radians(az) + head)
    print(f'near(<2m) 点数 {vals.size}  深度 p5={np.quantile(d,0.05):.3f} '
          f'中位={np.median(d):.3f}')

    # 网格聚类(0.30m)
    cell = {}
    for i in range(vals.size):
        k = (int(math.floor(gx[i] / 0.30)), int(math.floor(gy[i] / 0.30)))
        cell.setdefault(k, []).append(i)
    top = sorted(cell.items(), key=lambda kv: -len(kv[1]))[:6]
    print('\n近点聚类 (0.30m 栅格, 按点数):')
    for (cxi, cyi), idxs in top:
        idxs = np.array(idxs)
        cx, cy = gx[idxs].mean(), gy[idxs].mean()
        r = math.hypot(cx - rx, cy - ry)
        near = min(KNOWN, key=lambda k: math.hypot(k[1] - cx, k[2] - cy))
        dn = math.hypot(near[1] - cx, near[2] - cy)
        print(f'  栅格({cxi:+3d},{cyi:+3d}) 点{len(idxs):5d} '
              f'世界≈({cx:+6.2f},{cy:+6.2f}) 距机{r:5.2f}m '
              f'depth中位={np.median(d[idxs]):5.3f} '
              f'| 最近已知: {near[0]} Δ{dn:.2f}m')

    # 逐个已知障碍, 看它在哪个扇区、理论最近表面距离
    print('\n已知几何在前方 4m 内的理论表面距离 (相机朝向参考):')
    for nm2, ox, oy, orr in KNOWN:
        dd = math.hypot(ox - rx, oy - ry)
        if dd > 4.0:
            continue
        brg = math.degrees(math.atan2(oy - ry, ox - rx))
        rel = (brg - math.degrees(head) + 180.0) % 360.0 - 180.0
        surf = max(dd - orr, 0.0)
        print(f'  {nm2:9s} 中心距{dd:5.2f}m 方位{brg:+7.1f}° '
              f'相对相机{rel:+7.1f}° 表面≈{surf:5.2f}m'
              + ('  <== 在当前扇区内' if abs(rel) < HFOV / 2 else ''))
    st['done'] = True


node = gz_transport.Node()
node.subscribe(GzPose_V, '/world/follow_world/dynamic_pose/info', pose_cb)
time.sleep(0.6)
for t in (f'/depth_{CAM}', f'/depth_{CAM}/depth_image'):
    node.subscribe(GzImage, t, img_cb)

t0 = time.time()
while time.time() - t0 < 10.0 and not st['done']:
    time.sleep(0.05)
if not st['done']:
    print('!! 未收到位姿或深度帧  pose=%s' % (st['pose'] is not None,))
