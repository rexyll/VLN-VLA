#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_sect_calib.py —— 标定"扇区索引 -> 方位角"的对应关系

depth_percept 的注释写的是"左→右, 中心方位 c+40,c+20,c,c-20,c-40",
即 section[0] 在相机左侧。但 Gazebo 图像列 0 对应 +Y(左) 还是 -Y(右)
需要实测确认 —— 若反了, 8 方位合成里的 fl/fr 就会互换, 安全层绕障时会
朝"错误的一侧"打方向。

做法: 取当前真值位姿, 对每台相机算 5 个扇区的 5% 分位数, 然后用两种
假设各算一遍射线真值, 比较总误差, 误差小者即为真实映射。
  H1: section[0] = +40° (左)   <- 注释的说法
  H2: section[0] = -40° (右)   <- 反过来的可能
"""
import math
import time

import numpy as np
import gz.transport13 as gz_transport
from gz.msgs10.image_pb2 import Image as GzImage
from gz.msgs10.pose_v_pb2 import Pose_V as GzPose_V

WALL, ROBOT_R = 6.0, 0.18
OBST = [(3.5, 2.0, 0.57), (-3.0, 3.2, 0.67), (0.5, -3.2, 0.47),
        (-4.2, -2.5, 0.57), (4.2, -3.0, 0.37), (-1.5, -4.5, 0.62),
        (2.2, 4.6, 0.42)]
PERSON_R = 0.30
DIRS = (('front', 0.0), ('left', 1.5707963),
        ('right', -1.5707963), ('back', 3.1415926))
MAXD, N, Q = 12.0, 5, 0.05


def ray_min(x, y, ang, extra=()):
    best = MAXD
    c, s = math.cos(ang), math.sin(ang)
    if c > 1e-6:
        best = min(best, (WALL - x) / c)
    if c < -1e-6:
        best = min(best, (-WALL - x) / c)
    if s > 1e-6:
        best = min(best, (WALL - y) / s)
    if s < -1e-6:
        best = min(best, (-WALL - y) / s)
    for ox, oy, r in list(OBST) + list(extra):
        R = r + ROBOT_R
        dx, dy = ox - x, oy - y
        proj = dx * c + dy * s
        if proj <= 0:
            continue
        p2 = dx * dx + dy * dy - proj * proj
        if p2 < R * R:
            best = min(best, proj - math.sqrt(max(R * R - p2, 0.0)))
    return max(best, 0.05)


st = {'pose': None, 'sect': {}}


def pose_cb(m):
    for p in m.pose:
        n = p.name.split('/')[-1].split('::')[-1]
        if n == 'robot':
            st['pose'] = p
        elif n == 'person':
            st['person'] = (p.position.x, p.position.y)


def mk(name):
    def cb(msg):
        if name in st['sect']:
            return
        h, w = msg.height, msg.width
        if h < 10 or len(msg.data) < h * w * 4:
            return
        a = np.frombuffer(msg.data[:h * w * 4], dtype='<f4').reshape(h, w)
        # 与 depth_percept 当前实现保持一致(P28: 按仰角取带 +1.5°~+18°)
        r0 = int(h * (0.5 - 18.0 / 83.58))
        r1 = int(h * (0.5 - 1.5 / 83.58)) + 1
        b = a[max(0, r0):min(h, r1)]
        ok = np.isfinite(b) & (b > 0.12) & (b < 11.5)
        if int(ok.sum()) < 60:
            return
        sec = []
        cw0, cw1 = int(w * 0.04), int(w * 0.96)
        for k in range(N):
            c0 = cw0 + int((cw1 - cw0) * k / N)
            c1 = cw0 + int((cw1 - cw0) * (k + 1) / N)
            v = b[:, c0:c1][ok[:, c0:c1]]
            sec.append(float(np.quantile(v, Q)) if v.size >= 15 else -1.0)
        st['sect'][name] = sec
    return cb


node = gz_transport.Node()
node.subscribe(GzPose_V, '/world/follow_world/dynamic_pose/info', pose_cb)
for nm, _ in DIRS:
    for t in (f'/depth_{nm}', f'/depth_{nm}/depth_image'):
        node.subscribe(GzImage, t, mk(nm))

t0 = time.time()
while time.time() - t0 < 10 and not (st['pose'] and len(st['sect']) >= 4):
    time.sleep(0.05)
if not st['pose']:
    raise SystemExit('!! 未拿到 robot 真值位姿')

p = st['pose']
rx, ry = p.position.x, p.position.y
qx, qy, qz, qw = (p.orientation.x, p.orientation.y,
                  p.orientation.z, p.orientation.w)
yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
per = st.get('person')
extra = [(per[0], per[1], PERSON_R)] if per else []
print(f'robot=({rx:+.2f},{ry:+.2f}) yaw={math.degrees(yaw):+.1f}°  '
      f'person={per}')

for name, rel in DIRS:
    sec = st['sect'].get(name)
    if not sec:
        print(f'{name}: 无帧')
        continue
    base = yaw + rel
    # H1: 索引 0 对应 +40°(左), 往下每次 -20°
    tr1 = [ray_min(rx, ry, base + math.radians((2 - k) * 18.4), extra)
           for k in range(N)]
    # H2: 反过来, 索引 0 对应 -40°(右)
    tr2 = [ray_min(rx, ry, base + math.radians(-(2 - k) * 18.4), extra)
           for k in range(N)]
    e1 = sum(abs(a - b) for a, b in zip(sec, tr1) if a > 0) / max(
        sum(1 for a in sec if a > 0), 1)
    e2 = sum(abs(a - b) for a, b in zip(sec, tr2) if a > 0) / max(
        sum(1 for a in sec if a > 0), 1)
    print(f'\n{name} (base={math.degrees(base):+.0f}°)')
    print('  实测扇区 p5 : ' + ' '.join(f'{v:6.2f}' for v in sec))
    print('  H1 真值(+40..-40): ' + ' '.join(f'{v:6.2f}' for v in tr1)
          + f'   平均|Δ|={e1:.3f}')
    print('  H2 真值(-40..+40): ' + ' '.join(f'{v:6.2f}' for v in tr2)
          + f'   平均|Δ|={e2:.3f}')
    print('  ==> ' + ('H1 成立(索引0=左)' if e1 < e2 else 'H2 成立(索引0=右, 需修正)'))
