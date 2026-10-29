#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""final_sdf.py —— 确定最终机器人轮组配置(实测标定过的组合)

实测结论:
  1) 万向轮 <link name="caster"> 原来没有任何关节 → 是游离刚体, 实测会飞到
     (-5.1, 0, 3.1), 车尾无支撑 → 车身后仰 ~31°, 驱动轮离地 → 原地打转。
     必须补 joint。
  2) 轮子 roll=+90° 使关节轴落在 -Y, DiffDrive 把 (v,w) 整体取反。
     保持几何不变, 把轴写成 0 0 -1 即回到 +Y。
  3) 驱动轮前移 0.06m, 让质心落在"后滑脚 + 驱动轮"支撑三角形内(否则
     质心正压轮轴, 轻微碰撞就翘头)。
  4) 万向轮用 fixed 低摩擦滑脚(mu 0.02), 原地转向时打滑阻力可忽略。
"""
import io

P = '/home/earl/workspace/VLA/vla_ws/src/vla_follow/worlds/work_area.sdf'
s = io.open(P, encoding='utf-8').read()

# 1) 驱动轮前移
s = s.replace('<pose relative_to="chassis">0 0.13 -0.07 1.5707 0 0</pose>',
              '<pose relative_to="chassis">0.06 0.13 -0.07 1.5707 0 0</pose>')
s = s.replace('<pose relative_to="chassis">0 -0.13 -0.07 1.5707 0 0</pose>',
              '<pose relative_to="chassis">0.06 -0.13 -0.07 1.5707 0 0</pose>')
# 2) 关节轴取反 (几何不变, 方向回到 +Y)
s = s.replace('<xyz resolved_frame="child">0 0 1</xyz>',
              '<xyz resolved_frame="child">0 0 -1</xyz>')
# 3) 万向轮补关节 + 低摩擦
s = s.replace('''      <joint name="left_wheel_joint" type="revolute">''',
              '''      <!-- 万向轮必须有关节: 否则只是游离刚体, 车尾无支撑, 车身会后仰 -->
      <joint name="caster_joint" type="fixed">
        <parent>chassis</parent>
        <child>caster</child>
      </joint>
      <joint name="left_wheel_joint" type="revolute">''', 1)
s = s.replace('<friction><ode><mu>0.05</mu><mu2>0.05</mu2></ode></friction>',
              '<friction><ode><mu>0.02</mu><mu2>0.02</mu2></ode></friction>')

io.open(P, 'w', encoding='utf-8', newline='').write(s)
print('caster_joint:', s.count('caster_joint'))
print('axis -1:', s.count('0 0 -1</xyz>'))
for i, ln in enumerate(s.splitlines(), 1):
    if 'relative_to="chassis"' in ln or '<xyz' in ln:
        print(i, ln.strip())
