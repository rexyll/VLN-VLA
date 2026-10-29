#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""patch_sdf9.py —— 万向轮改为低摩擦滑脚(fixed 关节 + mu 0.01)

ball 关节的球在"原地转向"时必须绕竖直轴自转, 该机制在 dartsim 里把偏航
几乎锁死(实测角速度只剩指令的 4%)。改用固定低摩擦滑脚: 直行时随动、
原地转向时打滑阻力极小(0.01*1.5kg ≈ 0.15N), 与真实万向脚轮行为接近。
"""
import io

P = '/home/earl/workspace/VLA/vla_ws/src/vla_follow/worlds/work_area.sdf'
s = io.open(P, encoding='utf-8').read()

s = s.replace('''      <joint name="caster_joint" type="ball">''',
              '''      <joint name="caster_joint" type="fixed">''')

# caster 摩擦: 定位 caster 段内的 mu=0.05
old = '''        <collision name="c">
          <geometry><sphere><radius>0.03</radius></sphere></geometry>
          <surface>
            <friction><ode><mu>0.05</mu><mu2>0.05</mu2></ode></friction>
          </surface>
        </collision>'''
new = '''        <collision name="c">
          <geometry><sphere><radius>0.03</radius></sphere></geometry>
          <surface>
            <friction><ode><mu>0.01</mu><mu2>0.01</mu2></ode></friction>
          </surface>
        </collision>'''
if old in s:
    s = s.replace(old, new)
    print('caster friction -> 0.01')
else:
    print('caster friction block NOT matched, 手动查看')
    i = s.find('caster')
    print(s[i - 200:i + 900])

io.open(P, 'w', encoding='utf-8', newline='').write(s)
print('ball->fixed:', s.count('type="fixed"'))
