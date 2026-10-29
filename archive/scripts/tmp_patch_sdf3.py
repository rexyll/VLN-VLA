#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""patch_sdf3.py —— 机器人模型稳定性修复

问题: 驱动轮位于车身 x=0, 与整车质心同在一条竖线上, 后端仅一个低摩擦
      万向轮。支撑多边形退化成一条边 → 轻微碰撞/加速就前倾"翘头",
      车头边缘(箱体 0.08m 高)砸到地面, 轮子离地空转 → 无有效转角,
      跟随控制器永远对不正目标, 表现为"原地打转"。

修复: 驱动轮前移 0.06m, 使质心(x≈0)落在"后万向轮(-0.12) + 驱动轮(+0.06)"
      构成的支撑三角形内部。
"""
import io

P = '/home/earl/workspace/VLA/vla_ws/src/vla_follow/worlds/work_area.sdf'
s = io.open(P, encoding='utf-8').read()

s = s.replace('<pose relative_to="chassis">0 0.13 -0.07',
              '<pose relative_to="chassis">0.06 0.13 -0.07')
s = s.replace('<pose relative_to="chassis">0 -0.13 -0.07',
              '<pose relative_to="chassis">0.06 -0.13 -0.07')

io.open(P, 'w', encoding='utf-8', newline='').write(s)
for ln in s.splitlines():
    if 'relative_to="chassis"' in ln:
        print(ln.strip())
