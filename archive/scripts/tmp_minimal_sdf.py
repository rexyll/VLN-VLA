#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""minimal_sdf.py —— 只做最小必要修改: 给万向轮补关节, 其余保持原始

用于确认: 在 yaw 公式修正后, 原始轮组(roll +90, 轴 0 0 1)的指令符号是否反。
"""
import io

P = '/home/earl/workspace/VLA/vla_ws/src/vla_follow/worlds/work_area.sdf'
s = io.open(P, encoding='utf-8').read()

# 轮子位置回到 x=0
s = s.replace('<pose relative_to="chassis">0.06 0.13 -0.07 1.5707 0 0</pose>',
              '<pose relative_to="chassis">0 0.13 -0.07 1.5707 0 0</pose>')
s = s.replace('<pose relative_to="chassis">0.06 -0.13 -0.07 1.5707 0 0</pose>',
              '<pose relative_to="chassis">0 -0.13 -0.07 1.5707 0 0</pose>')
# 轴回到 0 0 1
s = s.replace('<xyz resolved_frame="child">0 0 -1</xyz>',
              '<xyz resolved_frame="child">0 0 1</xyz>')
# 万向轮摩擦回 0.05
s = s.replace('<friction><ode><mu>0.02</mu><mu2>0.02</mu2></ode></friction>',
              '<friction><ode><mu>0.05</mu><mu2>0.05</mu2></ode></friction>')

io.open(P, 'w', encoding='utf-8', newline='').write(s)
print('caster_joint:', s.count('caster_joint'), '| axis -1:', s.count('0 0 -1'))
