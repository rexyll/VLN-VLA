#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_patch_p36.py —— P36: 速度尺度补偿 + 对正提速 + 远处误检抑制

实测(/robot/cmd_vel 直发, odom 测位移):
    指令 0.20 → 实测 0.121      指令 0.62 → 实测 0.391
    指令 0.35 → 实测 0.221      指令 0.75 → 实测 0.462
    指令 0.52 → 实测 0.343      指令 0.90 → 实测 0.502
                                指令 1.10 → 实测 0.616
  即 实测 ≈ 指令 × 0.62 —— 整个速度环的"实际速度"只有指令的 62%,
  而 PID/安全层/限速全按"真实 m/s"设计, 于是永远跑不到该有的速度,
  表现为"盯着目标慢慢挪、被拉开 5~10m"。这是跟随跟不上的真正根因。

  S1  新增 cmd_gain=1.62, 下发前把内部真实速度换算成指令速度
  S2  ALIGN 边转边走 0.60 → 0.85 倍 approach_speed(对正段不再拖后腿)
  S3  人速上限 0.55 → 0.50(机器人真实 0.52, 留跟踪余量)
  S4  person_vision: DEPTH_MAX 11.5 → 7.0, AREA_MIN 45 → 30
      (10m 外 5px 宽的暗斑会通过几何校验被误当成人, 实测导致
       "人在 10m 外还显示 TRACK 未丢失")
"""
import io
import sys

FP = 'vla_ws/src/vla_follow/vla_follow/follower.py'
WP = 'vla_ws/src/vla_follow/vla_follow/person_walk.py'
VP = 'vla_ws/src/vla_follow/vla_follow/person_vision.py'

s = io.open(FP, encoding='utf-8').read()
w = io.open(WP, encoding='utf-8').read()
v = io.open(VP, encoding='utf-8').read()
if 'P36' in s:
    print('!! 已打过 P36')
    sys.exit(1)


def rep(buf, old, new, tag):
    n = buf.count(old)
    if n != 1:
        print('!! [%s] 锚点命中 %d 次' % (tag, n))
        sys.exit(1)
    print('   ok %s' % tag)
    return buf.replace(old, new, 1)


# ---- S1a 配置 ----
s = rep(s, """    'vis_bias_m': 0.25,             # P35: 视觉测的是人体表面, 补成模型中心距""",
        """    'cmd_gain': 1.62,               # P36: 指令速度→实际速度的补偿(实测×0.62)
    'vis_bias_m': 0.25,             # P35: 视觉测的是人体表面, 补成模型中心距""",
        'S1a')

# ---- S1b 下发时换算 ----
s = rep(s, """        w_t = max(-2.2, min(2.2, w_t))
        self._publish(self.v, w_t)""",
        """        w_t = max(-2.2, min(2.2, w_t))
        # P36: self.v 是"真实 m/s"(PID/安全层/限速都用它), 下发前换算成
        # 指令值(实测平台 指令 × 0.62 ≈ 实际)。
        self._publish(self.v * float(self.cfg['cmd_gain']), w_t)""",
        'S1b')

# ---- S2 对正段速度 ----
s = rep(s, """            if aim < math.radians(34.0):
                v_t = 0.6 * self.cfg['approach_speed']""",
        """            if aim < math.radians(34.0):
                # P36: 0.60 → 0.85。对正段占了追踪时间的一半以上,
                # 0.6 倍速度让机器人在"边转边走"时明显掉队。
                v_t = 0.85 * self.cfg['approach_speed']""",
        'S2')

io.open(FP, 'w', encoding='utf-8', newline='\n').write(s)

# ---- S3 人速 ----
w = rep(w, """        # P35: 上限 0.60→0.55 —— 机器人 approach_speed=0.52, 人要能被跟上
        self.target_speed = random.uniform(0.15, 0.55)""",
        """        # P35: 上限 0.60→0.55 —— 机器人 approach_speed=0.52, 人要能被跟上
        # P36: 0.55→0.50, 给速度环留出真正的跟踪余量
        self.target_speed = random.uniform(0.15, 0.50)""",
        'S3a')
w = rep(w, """                self.target_speed = random.uniform(0.15, 0.55)""",
        """                self.target_speed = random.uniform(0.15, 0.50)""",
        'S3b')
io.open(WP, 'w', encoding='utf-8', newline='\n').write(w)

# ---- S4 检测距离/面积 ----
v = rep(v, """AREA_MIN, AREA_MAX = 45, 22000""",
        """AREA_MIN, AREA_MAX = 30, 22000   # P36: 45→30, 4~5m 外的人也有 40px""",
        'S4a')
v = rep(v, """DEPTH_MAX = 11.5""",
        """DEPTH_MAX = 7.0   # P36: 11.5→7.0, 更远处 5px 的暗斑会被误判成人""",
        'S4b')
io.open(VP, 'w', encoding='utf-8', newline='\n').write(v)

print('P36_APPLIED')
