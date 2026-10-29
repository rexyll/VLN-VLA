#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_patch_p37.py —— P37: 消除自身遮挡 + 深度近场下限

实测发现两处自遮挡, 让全向避障把"自己"当成障碍, 机器人几乎不敢前进
(评测里 cmd_v 均值只有 0.066 m/s, 日志满是 `全向急停 前0.16 左前0.19`):

  1) 雷达桅杆 mast_v 在 `0.12 0 0.045`, 半径 0.015, 长 0.06 → 顶面 z=0.075。
     4 台相机装在 z=0.10 朝外, 于是桅杆落在每台相机画面中心偏下,
     成了一个 32x32 px、距离 0.11m 的暗色斑点(person_vision 里那个
     "0.11m 伪影" 就是它)。改成下沉 + 后移到 `-0.02 0 0.028`,
     顶端 z=0.053, 落在相机视锥之外。
  2) depth_percept 的 D_MIN 还是 0.12, 车体前缘/桅杆根部仍会漏进
     prox 的 front 扇区。D_MIN 提到 0.30 —— 车体外接半径 0.18、
     停车距离 0.66, 0.30m 以内的回波对避障没有决策价值,
     留着只会把自己当障碍。
"""
import io
import sys

SP = 'vla_ws/src/vla_follow/worlds/work_area.sdf'
DP = 'vla_ws/src/vla_follow/vla_follow/depth_percept.py'

sdf = io.open(SP, encoding='utf-8').read()
dp = io.open(DP, encoding='utf-8').read()
if 'P37' in sdf:
    print('!! 已打过 P37')
    sys.exit(1)


def rep(buf, old, new, tag):
    n = buf.count(old)
    if n != 1:
        print('!! [%s] 锚点命中 %d 次' % (tag, n))
        sys.exit(1)
    print('   ok %s' % tag)
    return buf.replace(old, new, 1)


sdf = rep(sdf,
          """        <visual name="mast_v">
          <pose>0.12 0 0.045 0 0 0</pose>
          <geometry><cylinder><radius>0.015</radius><length>0.06</length></cylinder></geometry>""",
          """        <!-- P37: 原 0.12 0 0.045 / r0.015 / len0.06 的顶面 z=0.075 恰好
             落在 4 台相机(z=0.10)的画面中心偏下, 成为一个 0.11m 的暗斑,
             既被 person_vision 当成"很近的人", 又被深度避障当成"正前方
             障碍"(日志 全向急停 前0.16 就是它)。下沉并后移到视锥之外。 -->
        <visual name="mast_v">
          <pose>-0.02 0 0.028 0 0 0</pose>
          <geometry><cylinder><radius>0.013</radius><length>0.05</length></cylinder></geometry>""",
          'S1-mast')

dp = rep(dp,
         """D_MIN = 0.12        # P29: 相机 near=0.1, 不可能有更近的回波; 实测 <0.12 的""",
         """D_MIN = 0.30        # P37: 0.12 → 0.30, 屏蔽车体前缘/桅杆根部的自遮挡
                    # (车体外接半径 0.18、停车距离 0.66, 0.30m 内的回波
                    # 对避障没有决策价值, 保留只会让机器人把自己当障碍)
# P29 说明: 相机 near=0.1, 不可能有更近的回波; 实测 <0.12 的""",
         'S2-dmin')

io.open(SP, 'w', encoding='utf-8', newline='\n').write(sdf)
io.open(DP, 'w', encoding='utf-8', newline='\n').write(dp)
print('P37_APPLIED')
