#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""patch_sdf2.py —— 回退轮子姿态, 改为把关节轴方向取反 (0 0 1 → 0 0 -1)

依据: 姿态 roll=+90° 时轮子子坐标系 +Z 指向车体 -Y, DiffDrive 于是把
(v,w) 整体取反。保持几何不变(姿态照旧), 仅把 revolute 轴反向, 即可让
"正角速度 → 逆时针" 与 ROS 约定一致。
"""
import io
import re

P = '/home/earl/workspace/VLA/vla_ws/src/vla_follow/worlds/work_area.sdf'
s = io.open(P, encoding='utf-8').read()

# 1) 回退轮子姿态 roll
s = s.replace('<pose relative_to="chassis">0 0.13 -0.07 -1.5707 0 0</pose>',
              '<pose relative_to="chassis">0 0.13 -0.07 1.5707 0 0</pose>')
s = s.replace('<pose relative_to="chassis">0 -0.13 -0.07 -1.5707 0 0</pose>',
              '<pose relative_to="chassis">0 -0.13 -0.07 1.5707 0 0</pose>')
s = s.replace('<pose relative_to="skirt">0 0.20 -0.225 -1.5707 0 0</pose>',
              '<pose relative_to="skirt">0 0.20 -0.225 1.5707 0 0</pose>')
s = s.replace('<pose relative_to="skirt">0 -0.20 -0.225 -1.5707 0 0</pose>',
              '<pose relative_to="skirt">0 -0.20 -0.225 1.5707 0 0</pose>')

# 2) 只对 4 个轮子关节把轴取反
s = s.replace('<xyz resolved_frame="child">0 0 1</xyz>',
              '<xyz resolved_frame="child">0 0 -1</xyz>')

io.open(P, 'w', encoding='utf-8', newline='').write(s)
print('axis -1 count:', s.count('<xyz resolved_frame="child">0 0 -1</xyz>'))
print('roll +1.5707 count:', s.count('1.5707 0 0</pose>'))
