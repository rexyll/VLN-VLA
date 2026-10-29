#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""restore_sdf.py —— 精确还原 work_area.sdf 的机器人/人的轮组原始写法"""
import io
import re

P = '/home/earl/workspace/VLA/vla_ws/src/vla_follow/worlds/work_area.sdf'
s = io.open(P, encoding='utf-8').read()

# 机器人轮子: 位置回到 x=0, roll +1.5707
s = re.sub(r'<pose relative_to="chassis">[^<]*</pose>',
           lambda m: ('<pose relative_to="chassis">0 0.13 -0.07 1.5707 0 0</pose>'
                      if '0.13 -0.07' in m.group(0) or ' 0.13 ' in m.group(0)
                      else (('<pose relative_to="chassis">0 -0.13 -0.07 1.5707 0 0</pose>')
                            if ' -0.13 ' in m.group(0)
                            else m.group(0))), s)
# 人轮子
s = s.replace('<pose relative_to="skirt">0.05 0.20 -0.225 -1.5707 0 0</pose>',
              '<pose relative_to="skirt">0 0.20 -0.225 1.5707 0 0</pose>')
s = s.replace('<pose relative_to="skirt">0.05 -0.20 -0.225 -1.5707 0 0</pose>',
              '<pose relative_to="skirt">0 -0.20 -0.225 1.5707 0 0</pose>')

# 关节轴回原始写法 + 恢复限位
s = s.replace('''        <axis>
          <xyz>0 0 1</xyz>
        </axis>''',
              '''        <axis>
          <xyz resolved_frame="child">0 0 1</xyz>
          <limits><lower>-1e16</lower><upper>1e16</upper></limits>
        </axis>''')

# 去掉 caster 关节
s = s.replace('''      <!-- 万向轮必须有关节, 否则只是游离刚体, 车尾无支撑 -->
      <joint name="caster_joint" type="fixed">
        <parent>chassis</parent>
        <child>caster</child>
      </joint>
''', '')
s = re.sub(r'[ \t]*<joint name="caster_joint"[^>]*>\n(?:.*?\n)*?[ \t]*</joint>\n',
           '', s)

# caster 摩擦回 0.05
s = s.replace('<friction><ode><mu>0.01</mu><mu2>0.01</mu2></ode></friction>',
              '<friction><ode><mu>0.05</mu><mu2>0.05</mu2></ode></friction>')

io.open(P, 'w', encoding='utf-8', newline='').write(s)
print('caster_joint 残留:', s.count('caster_joint'))
print('limits 数量:', s.count('1e16'))
print('resolved_frame 数量:', s.count('resolved_frame'))
for i, ln in enumerate(s.splitlines(), 1):
    t = ln.strip()
    if 'relative_to="chassis"' in t or 'relative_to="skirt">0 0.2' in t \
            or 'relative_to="skirt">0 -0.2' in t:
        print(i, t)
