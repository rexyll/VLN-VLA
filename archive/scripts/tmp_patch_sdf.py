#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""patch_sdf.py —— 修正 work_area.sdf 里轮子滚转符号 (在 VM 上执行)"""
import io
import sys

P = '/home/earl/workspace/VLA/vla_ws/src/vla_follow/worlds/work_area.sdf'
REPL = [
    ('<pose relative_to="chassis">0 0.13 -0.07 1.5707 0 0</pose>',
     '<pose relative_to="chassis">0 0.13 -0.07 -1.5707 0 0</pose>'),
    ('<pose relative_to="chassis">0 -0.13 -0.07 1.5707 0 0</pose>',
     '<pose relative_to="chassis">0 -0.13 -0.07 -1.5707 0 0</pose>'),
    ('<pose relative_to="skirt">0 0.20 -0.225 1.5707 0 0</pose>',
     '<pose relative_to="skirt">0 0.20 -0.225 -1.5707 0 0</pose>'),
    ('<pose relative_to="skirt">0 -0.20 -0.225 1.5707 0 0</pose>',
     '<pose relative_to="skirt">0 -0.20 -0.225 -1.5707 0 0</pose>'),
]

s = io.open(P, encoding='utf-8').read()
n = 0
for a, b in REPL:
    c = s.count(a)
    if c:
        s = s.replace(a, b)
        n += c
io.open(P, 'w', encoding='utf-8', newline='').write(s)
print('replaced', n, 'of', len(REPL))
for a, _ in REPL:
    if a in s:
        print('STILL PRESENT:', a)
