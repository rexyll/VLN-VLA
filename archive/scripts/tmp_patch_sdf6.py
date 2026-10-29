#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""patch_sdf6.py —— 关节轴改为显式 0 1 0 并去掉非标准属性 resolved_frame

SDFormat 的 <axis><xyz> 只认 <xyz> 内容(+可选的 expressed_in), 没有
resolved_frame 属性; 写了也被忽略, 轴被解释到"连杆自身坐标系"里,
于是轮子绕竖直轴打滑 —— 车体几乎不转、前进也丢速。
改为明确给出父(车体)坐标系下的 +Y(左向)作为轮子转轴。
"""
import io
import re

P = '/home/earl/workspace/VLA/vla_ws/src/vla_follow/worlds/work_area.sdf'
s = io.open(P, encoding='utf-8').read()
s = s.replace('<xyz resolved_frame="child">0 0 -1</xyz>', '<xyz>0 1 0</xyz>')
s = s.replace('<xyz resolved_frame="child">0 0 1</xyz>', '<xyz>0 1 0</xyz>')
io.open(P, 'w', encoding='utf-8', newline='').write(s)
print('axis 0 1 0 count:', s.count('<xyz>0 1 0</xyz>'))
print('resolved_frame left:', s.count('resolved_frame'))
for i, ln in enumerate(s.splitlines(), 1):
    if '<xyz>' in ln:
        print(i, ln.strip())
