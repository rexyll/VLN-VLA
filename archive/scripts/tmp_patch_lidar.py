#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""修复 gpu_lidar: 360 条射线全部返回量程下限 0.12m。

诊断依据(tmp_scan_diag.py 实测):
  angle_min=-180° angle_max=+180° 360 点  range 0.12~12.0
  有效射线 360/360, min=中位=max=0.120, 无回波 0 条
所有方向、所有射线都是精确的 range_min —— 真实场景不可能出现(至少要能照到
5.75m 外的墙)。说明光线根本没打出去, 返回值被 clamp 到下限。

两个原因一起改:
1. type="gpu_lidar" 走 Ogre2 的 GPU 深度渲染; 本机是虚拟机(软件光栅),
   该路径拿不到深度图 → 全 0 → 被 clamp 到 range_min。
   改成 type="lidar"(CPU 射线投射), 不依赖 GPU。360 线 10Hz 的 CPU 开销很小。
2. 缺 <vertical> 段。官方示例(以及 gz-sim 的 lidar 解析器)都带这一段,
   缺失时垂直方向的采样数不确定, 个别版本会退化成 0 行。
3. 顺带把安装位抬到 z=0.20 并居中 x=0: 无论 <pose> 被解释成"相对 link"
   (世界 z=0.33) 还是"相对 model"(世界 z=0.20), 都在车顶(世界 0.18)之上,
   两种解释下都不会自遮挡。
"""
import re
import shutil
import sys

P = '/home/earl/workspace/VLA/vla_ws/src/vla_follow/worlds/work_area.sdf'
shutil.copy(P, P + '.bak_lidar')
s = open(P, encoding='utf-8').read()

# 1) gpu_lidar -> lidar (CPU 射线)
c1 = s.count('<sensor name="gpu_lidar" type="gpu_lidar">')
s = s.replace('<sensor name="gpu_lidar" type="gpu_lidar">',
              '<sensor name="gpu_lidar" type="lidar">')

# 2) 安装位: 抬高 + 居中
c2 = s.count('<pose>0.08 0 0.15 0 0 0</pose>')
s = s.replace('<pose>0.08 0 0.15 0 0 0</pose>',
              '<pose>0 0 0.20 0 0 0</pose>')

# 3) 补 <vertical> 段
pat = re.compile(
    r'(<scan>\s*<horizontal>\s*<samples>360</samples>\s*<resolution>1</resolution>\s*'
    r'<min_angle>-3\.14159</min_angle>\s*<max_angle>3\.14159</max_angle>\s*'
    r'</horizontal>)(\s*</scan>)')
vert = ('\n              <vertical>'
        '\n                <samples>1</samples>'
        '\n                <resolution>1</resolution>'
        '\n                <min_angle>0</min_angle>'
        '\n                <max_angle>0</max_angle>'
        '\n              </vertical>')
s, c3 = pat.subn(r'\g<1>' + vert + r'\g<2>', s)

open(P, 'w', encoding='utf-8').write(s)
print('sensor type : %d 处 (gpu_lidar -> lidar)' % c1)
print('安装位      : %d 处 (0.08,0,0.15 -> 0,0,0.20)' % c2)
print('vertical 段 : %d 处' % c3)
if not (c1 == 1 and c2 == 1 and c3 == 1):
    print('!! 替换数与预期不符')
    sys.exit(1)
