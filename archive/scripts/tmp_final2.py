#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""final2.py —— 幂等地把机器人模型设为最终配置(实测标定)

最终配置(每一条都用真值标定过):
  1) 万向轮补固定关节 + 低摩擦(mu 0.02) —— 否则万向轮是游离刚体, 车尾
     无支撑, 车身后仰 ~31°, 驱动轮离地 → "原地打转"的机械根源。
  2) 驱动轮前移 0.06m —— 质心原本正压在轮轴上, 支撑多边形退化成一条线,
     轻微碰撞就翘头。
  3) 关节轴 0 0 -1 —— 轮子 roll=+90° 后子坐标系 +Z 指向 -Y, DiffDrive 会
     把 (v,w) 整体取反(实测 w=+1.0 → -0.78 rad/s, v=+0.3 → -0.20 m/s)。
     保持几何不变、只反轴, 即可让"正角速度=逆时针"。
"""
import io

P = '/home/earl/workspace/VLA/vla_ws/src/vla_follow/worlds/work_area.sdf'
s = io.open(P, encoding='utf-8').read()

# 1) 驱动轮前移
s = s.replace('<pose relative_to="chassis">0 0.13 -0.07 1.5707 0 0</pose>',
              '<pose relative_to="chassis">0.06 0.13 -0.07 1.5707 0 0</pose>')
s = s.replace('<pose relative_to="chassis">0 -0.13 -0.07 1.5707 0 0</pose>',
              '<pose relative_to="chassis">0.06 -0.13 -0.07 1.5707 0 0</pose>')
# 2) 关节轴反号(机器人两轮 + 人两轮, 保持一致)
s = s.replace('<xyz resolved_frame="child">0 0 1</xyz>',
              '<xyz resolved_frame="child">0 0 -1</xyz>')
# 3) 万向轮低摩擦滑脚
s = s.replace('<friction><ode><mu>0.05</mu><mu2>0.05</mu2></ode></friction>',
              '<friction><ode><mu>0.02</mu><mu2>0.02</mu2></ode></friction>')
# 4) 确保万向轮关节存在且为 fixed
if 'caster_joint' not in s:
    s = s.replace('      <joint name="left_wheel_joint" type="revolute">',
                  '''      <!-- 万向轮必须有 joint: 否则是游离刚体, 车尾无支撑 -->
      <joint name="caster_joint" type="fixed">
        <parent>chassis</parent>
        <child>caster</child>
      </joint>
      <joint name="left_wheel_joint" type="revolute">''', 1)

io.open(P, 'w', encoding='utf-8', newline='').write(s)
print('caster_joint=', s.count('name="caster_joint"'),
      '| axis-1=', s.count('0 0 -1</xyz>'),
      '| wheels0.06=', s.count('chassis">0.06'))
