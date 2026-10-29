#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""patch_sdf5.py —— 去掉轮关节的 <limits> ±1e16

dartsim 对 1e16 的关节限位会做异常处理(溢出/钳制), 导致轮子实际转速
只有指令的 ~40%, 表现为"转向很慢、对不正、走不动"。
revolute 关节不写 <limits> 即为连续旋转关节(无角度限制), 语义正确。
"""
import io

P = '/home/earl/workspace/VLA/vla_ws/src/vla_follow/worlds/work_area.sdf'
s = io.open(P, encoding='utf-8').read()

bad = '''          <axis>
            <xyz resolved_frame="child">0 0 -1</xyz>
            <limits><lower>-1e16</lower><upper>1e16</upper></limits>
          </axis>'''
bad2 = '''        <axis>
          <xyz resolved_frame="child">0 0 -1</xyz>
          <limits><lower>-1e16</lower><upper>1e16</upper></limits>
        </axis>'''
print('8空格变体', s.count(bad), '10空格变体', s.count(bad2))

import re
s2 = re.sub(r'[ \t]*<limits><lower>-1e16</lower><upper>1e16</upper></limits>\n',
            '', s)
io.open(P, 'w', encoding='utf-8', newline='').write(s2)
print('剩余 limits:', s2.count('-1e16'))
for ln in s2.splitlines():
    if 'multi_resolution' in ln:
        print(ln)
