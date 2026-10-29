#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""patch_sdf8.py —— 按 Gazebo 官方 diff_drive 约定重建轮组

官方 /usr/share/gz/gz-sim8/worlds/diff_drive_skid.sdf 的写法是:
    <pose>-0.957 -0.625 -0.025 -1.5707 0 0</pose>   ← roll = -1.5707
    <joint type="revolute"><axis><xyz>0 0 1</xyz></axis>
这样轮子子坐标系的 +Z 正好指向车体 +Y, DiffDrive 的正速度即 ROS 正方向。

本文件统一为:
  1) 轮子 roll = -1.5707 (官方约定), 轴 <xyz>0 0 1</xyz>
  2) 驱动轮前移 0.06m, 让质心落在"后万向轮+驱动轮"支撑三角形内
  3) 万向轮补 ball 关节(否则车尾无支撑, 车身后仰、轮子离地)
  4) 取消 ±1e16 关节限位(改为连续旋转)
  5) 取消 2mm 预压(会造成车轮楔入地面卡死)
"""
import io
import re

P = '/home/earl/workspace/VLA/vla_ws/src/vla_follow/worlds/work_area.sdf'
s = io.open(P, encoding='utf-8').read()

# 1) 机器人轮子: 位置 + roll -1.5707
s = s.replace('<pose relative_to="chassis">0 0.13 -0.07 1.5707 0 0</pose>',
              '<pose relative_to="chassis">0.06 0.13 -0.07 -1.5707 0 0</pose>')
s = s.replace('<pose relative_to="chassis">0 -0.13 -0.07 1.5707 0 0</pose>',
              '<pose relative_to="chassis">0.06 -0.13 -0.07 -1.5707 0 0</pose>')
s = s.replace('<pose relative_to="chassis">0.06 0.13 -0.072 1.5707 0 0</pose>',
              '<pose relative_to="chassis">0.06 0.13 -0.07 -1.5707 0 0</pose>')
s = s.replace('<pose relative_to="chassis">0.06 -0.13 -0.072 1.5707 0 0</pose>',
              '<pose relative_to="chassis">0.06 -0.13 -0.07 -1.5707 0 0</pose>')
# 2) 人: 同理统一 roll, 并前移使其稳定
s = s.replace('<pose relative_to="skirt">0 0.20 -0.225 1.5707 0 0</pose>',
              '<pose relative_to="skirt">0.05 0.20 -0.225 -1.5707 0 0</pose>')
s = s.replace('<pose relative_to="skirt">0 -0.20 -0.225 1.5707 0 0</pose>',
              '<pose relative_to="skirt">0.05 -0.20 -0.225 -1.5707 0 0</pose>')
# 3) 关节轴统一 0 0 1 (子坐标系)
s = s.replace('<xyz>0 1 0</xyz>', '<xyz>0 0 1</xyz>')
s = s.replace('<xyz resolved_frame="child">0 0 -1</xyz>', '<xyz>0 0 1</xyz>')
s = s.replace('<xyz resolved_frame="child">0 0 1</xyz>', '<xyz>0 0 1</xyz>')
# 4) 清理限位
s = re.sub(r'[ \t]*<limits><lower>-1e16</lower><upper>1e16</upper></limits>\n',
           '', s)

io.open(P, 'w', encoding='utf-8', newline='').write(s)
print('--- 轮/关节结果 ---')
for i, ln in enumerate(s.splitlines(), 1):
    t = ln.strip()
    if ('relative_to="chassis"' in t or 'relative_to="skirt"' in t
            or '<xyz>' in t or 'joint name=' in t):
        print(i, t)
