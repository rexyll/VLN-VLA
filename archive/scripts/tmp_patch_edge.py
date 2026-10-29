#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_patch_edge.py —— P29: 剔除深度图边缘伪影 + near-clip 自遮挡

P28 把行带下沿从 +4.2° 降到 +1.5° 后, front 左前扇区读到 0.35m。
定向探测(tmp_depth_near.py)显示这批点:
    方位 +40.00°~+50.00°   <- 恰好是画面最左边缘
    仰角 +1.74°~+3.83°     <- 恰好是行带最底部
    深度 0.330~0.404m
    数量 153 / 带内 4453 = 3.4%
世界坐标反投影落在空地(距机 0.37m, 45° 左前), 那里没有任何几何体 ——
这是深度图**边缘的插值伪影**, 集中度刚好超过扇区 5% 分位数门槛,
于是 p5 从"真实的几米"坍缩到 0.33m, 安全层收到"0.35m 有障碍"。

修法(两条都很硬, 无副作用):
  1) 列方向裁 4%: 只取画面中间 92% 的列。
     深度图左右边缘是空洞插值区, 本来不可信; 而相邻相机有 10° 重叠,
     该方向的真实距离由**另一台相机的中心区**给出, 不丢信息。
     扇区中心方位随之从 ±40/±20/0° 变为 ±36.8/±18.4/0°。
  2) 深度下限 0.05 -> 0.12: 相机 <clip><near>0.1</near>, 物理上不可能
     有 <0.1m 的回波; 实测 <0.12m 的值全是机器人自身底盘的自遮挡(0.105m)。
"""
import shutil
import sys

P = '/home/earl/workspace/VLA/vla_ws/src/vla_follow/vla_follow/depth_percept.py'
shutil.copy(P, P + '.bak_edge')
s = open(P, encoding='utf-8').read()

# ---- 1. 常量: 边缘裁剪比例 ----
OLD0 = "SECT_Q = 0.05       # 扇区内取 5% 分位数(扇区窄, 不怕被背景稀释)"
NEW0 = """SECT_Q = 0.05       # 扇区内取 5% 分位数(扇区窄, 不怕被背景稀释)
EDGE_CROP = 0.04    # P29: 列方向裁掉左右各 4% —— 深度图边缘是空洞插值区,
#                     实测这里会冒出 0.33m 的假回波(方位 +40~+50°), 且集中
#                     度刚好超过 5% 分位数门槛, 会把"几米外的空地"读成 0.35m。
#                     相邻相机有 10° 重叠, 该方向的真实值由另一台相机的中心
#                     区给出, 裁边不丢信息。
D_MIN = 0.12        # P29: 相机 near=0.1, 不可能有更近的回波; 实测 <0.12 的
#                     值全是自身底盘自遮挡(0.105m)"""
if s.count(OLD0) != 1:
    print('!! 常量锚点异常')
    sys.exit(1)
s = s.replace(OLD0, NEW0, 1)

# ---- 2. 有效区间 ----
OLD1 = "            ok = np.isfinite(a) & (a > 0.05) & (a < 11.5)"
NEW1 = "            ok = np.isfinite(a) & (a > D_MIN) & (a < 11.5)"
if s.count(OLD1) != 1:
    print('!! ok 锚点异常')
    sys.exit(1)
s = s.replace(OLD1, NEW1, 1)

# ---- 3. 扇区列范围改为中心 92% ----
OLD2 = """            sec = []
            for k in range(SECT_N):
                c0, c1 = int(w * k / SECT_N), int(w * (k + 1) / SECT_N)
                v = a[:, c0:c1][ok[:, c0:c1]]
                sec.append(float(np.quantile(v, SECT_Q)) if v.size >= 15
                           else -1.0)"""
NEW2 = """            sec = []
            cw0 = int(w * EDGE_CROP)
            cw1 = int(w * (1.0 - EDGE_CROP))
            for k in range(SECT_N):
                c0 = cw0 + int((cw1 - cw0) * k / SECT_N)
                c1 = cw0 + int((cw1 - cw0) * (k + 1) / SECT_N)
                v = a[:, c0:c1][ok[:, c0:c1]]
                sec.append(float(np.quantile(v, SECT_Q)) if v.size >= 15
                           else -1.0)"""
if s.count(OLD2) != 1:
    print('!! 扇区锚点异常')
    sys.exit(1)
s = s.replace(OLD2, NEW2, 1)

# ---- 4. 误差日志的扇区中心方位 +-40/+-20 -> +-36.8/+-18.4 ----
OLD3 = """                    # 扇区中心方位: section 0 在相机左侧 +40°, 依次 -20°
                    a = ryaw + rel + math.radians((2 - k) * 20.0)"""
NEW3 = """                    # 扇区中心方位: 裁掉左右各 4% 后, 每扇区宽 18.4°,
                    # section 0 在相机左侧 +36.8°, 依次 -18.4°
                    a = ryaw + rel + math.radians((2 - k) * SECT_DEG)"""
if s.count(OLD3) != 1:
    print('!! 误差日志方位锚点异常')
    sys.exit(1)
s = s.replace(OLD3, NEW3, 1)

OLD4 = "                    off = (2 - worst_k) * 20"
NEW4 = "                    off = (2 - worst_k) * SECT_DEG"
if s.count(OLD4) != 1:
    print('!! off 锚点异常')
    sys.exit(1)
s = s.replace(OLD4, NEW4, 1)

# ---- 5. SECT_DEG 常量 ----
OLD5 = "EDGE_CROP = 0.04"
NEW5 = "SECT_DEG = 18.4    # 每扇区方位跨度(裁边后 92%/5*100 = 18.4°)\nEDGE_CROP = 0.04"
s = s.replace(OLD5, NEW5, 1)

open(P, 'w', encoding='utf-8').write(s)
print('PATCH_OK')
print('  D_MIN:', s.count('D_MIN'), ' EDGE_CROP:', s.count('EDGE_CROP'),
      ' SECT_DEG:', s.count('SECT_DEG'))
