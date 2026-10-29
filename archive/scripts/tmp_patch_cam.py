#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""修正 4 向 RGB-D 相机参数, 使其严格满足:
   - 4 个视野完整覆盖 360°
   - 相邻视野重叠约 10°

原参数: horizontal_fov = 1.8 rad = 103.13°, 安装间隔 90°
        -> 重叠 = 103.13 - 90 = 13.13°  (偏大 3.1°)
        -> 且 4 个相机都挂在 chassis 前部 x=+0.10, 不是同心布置

改为  : horizontal_fov = 100.000° = 1.7453293 rad
        -> 重叠 = 100 - 90 = 10.00° (精确)
        -> 4 个相机统一 x=0 (同心), 扇形严格以车体中心为圆心

另外: chassis 上的"雷达桅杆" visual 原为 z 中心 0.08 / 长 0.12, 顶端高到
      相对 z=0.14, 高于相机安装面 z=0.10, 且柱面距相机中心仅 0.02m ——
      会占满 back/left/right 相机的近场视野(深度图里出现"贴脸"柱面),
      使这四个方向的识人/测距严重失真。改短并前移。
"""
import re
import shutil
import sys

P = '/home/earl/workspace/VLA/vla_ws/src/vla_follow/worlds/work_area.sdf'
BAK = '/home/earl/workspace/VLA/vla_ws/src/vla_follow/worlds/work_area.sdf.bak_cam'

s = open(P, encoding='utf-8').read()
shutil.copy(P, BAK)
n_fov = n_pose = n_mast = 0

# ---- 1. FOV: 1.8 rad -> 100.000 deg ----
n_fov = s.count('<horizontal_fov>1.8</horizontal_fov>')
s = s.replace('<horizontal_fov>1.8</horizontal_fov>',
              '<horizontal_fov>1.7453293</horizontal_fov>')   # 100.000 deg

# ---- 2. 4 个 rgbd 相机改成同心安装 (x: 0.10 -> 0) ----
for yaw in ('0', '1.5707963', '-1.5707963', '3.1415926'):
    a = '<pose>0.10 0 0.10 0 0 %s</pose>' % yaw
    b = '<pose>0 0 0.10 0 0 %s</pose>' % yaw
    n_pose += s.count(a)
    s = s.replace(a, b)

# ---- 3. 桅杆缩短 + 前移, 让出相机近场 ----
s2, n_mast = re.subn(
    r'(<visual name="mast_v">\s*<pose>)0\.08 0 0\.08 0 0 0(</pose>\s*'
    r'<geometry><cylinder><radius>)0\.02(</radius><length>)0\.12(</length>)',
    r'\g<1>0.12 0 0.045 0 0 0\g<2>0.015\g<3>0.06\g<4>',
    s)
if n_mast:
    s = s2

open(P, 'w', encoding='utf-8').write(s)

print('FOV 替换       : %d 处 (1.8 -> 1.7453293 rad = 100.0 deg)' % n_fov)
print('相机位姿同心化 : %d 处 (x: 0.10 -> 0)' % n_pose)
print('桅杆缩短       : %d 处' % n_mast)
print()
print('理论校核: 相邻重叠 = %.2f - 90.00 = %.2f deg'
      % (100.0, 100.0 - 90.0))
if n_fov != 4 or n_pose != 4 or n_mast != 1:
    print('!! 替换数与预期不符, 请检查')
    sys.exit(1)
