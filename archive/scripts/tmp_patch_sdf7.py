#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""patch_sdf7.py —— 驱动轮加 2mm 预压(相对地面下沉), 确保抓地

三个接触点(两驱动轮 + 万向球)原本都恰好与地面相切(z=0), 接触力分配退化:
数值上容易变成"只由低摩擦万向球承重", 驱动轮悬空 → 指令怎么给车都不动,
或只有极小响应。把驱动轮下沉 2mm, 保证驱动轮始终压在地面上。
"""
import io

P = '/home/earl/workspace/VLA/vla_ws/src/vla_follow/worlds/work_area.sdf'
s = io.open(P, encoding='utf-8').read()
s = s.replace('<pose relative_to="chassis">0.06 0.13 -0.07 1.5707 0 0</pose>',
              '<pose relative_to="chassis">0.06 0.13 -0.072 1.5707 0 0</pose>')
s = s.replace('<pose relative_to="chassis">0.06 -0.13 -0.07 1.5707 0 0</pose>',
              '<pose relative_to="chassis">0.06 -0.13 -0.072 1.5707 0 0</pose>')
io.open(P, 'w', encoding='utf-8', newline='').write(s)
for ln in s.splitlines():
    if 'relative_to="chassis"' in ln:
        print(ln.strip())
