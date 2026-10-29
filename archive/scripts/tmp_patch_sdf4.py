#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""patch_sdf4.py —— 给机器人万向轮补上关节(根因修复)

原模型 <link name="caster"> 没有任何 <joint>, 在 SDF 中它是游离刚体,
不承载车重 → 车身只有两个驱动轮(共线)支撑, 重心又几乎压在这条线上,
于是启动即后仰 ~20-30°, 车尾底板着地、驱动轮离地空转 —— 表现为
"原地打转、朝向调不正、距离不收敛"。

修复: 用 ball 关节把万向轮接到 chassis, 恢复"两驱动轮 + 后万向球"三点
支撑(与前移 0.06m 的驱动轮配合, 重心落在支撑三角形内部)。
"""
import io

P = '/home/earl/workspace/VLA/vla_ws/src/vla_follow/worlds/work_area.sdf'
s = io.open(P, encoding='utf-8').read()

anchor = '''      <joint name="left_wheel_joint" type="revolute">'''
assert anchor in s, 'anchor not found'
if 'caster_joint' not in s:
    caster_joint = '''      <!-- 万向轮必须有关节, 否则只是游离刚体, 车尾无支撑 -->
      <joint name="caster_joint" type="ball">
        <parent>chassis</parent>
        <child>caster</child>
      </joint>
'''
    s = s.replace(anchor, caster_joint + anchor, 1)
    io.open(P, 'w', encoding='utf-8', newline='').write(s)
    print('caster_joint inserted')
else:
    print('caster_joint already present')

for ln in s.splitlines():
    if 'joint name=' in ln:
        print(' ', ln.strip())
