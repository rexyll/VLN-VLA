#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_patch_target.py —— P31: 把"被跟随的人"从障碍物里摘出去

跟随回归(人沿 r=2m 圆周以 0.32m/s 行走 111s)的两个硬伤:

1) 距离长期停在 1.385m, 只有 25.6% 时间落在 1.00±0.15m 内。
   approach_speed 已被我压到 0.32m/s, 与人的步速恰好相等 -> 永远追不上。
2) 全程 259 条 "全向限速", 例如:
       距离=1.03m(目标1.00m) ... 全向限速 前0.71 左前1.02 右前2.35 v<=0.09
   跟到 1.0m 时, 人的**躯干表面**距相机只有 0.7m, 低于 slow_d(1.6),
   安全层把"要跟随的人"当成障碍物, 一路把速度压到 0.06~0.12m/s。
   两个问题叠加, 机器人只能以 0.1m/s 爬, 人一走动就掉队。

修法:
  · 安全层拿到目标距离/方位, 若某个前向回波的距离与"人的躯干表面期望
    距离"相差 < 0.45m, 且该方位在目标方向附近, 就判定"那是目标不是障碍",
    把它抬到 slow_d 以上, 不再据此限速。
  · 静止障碍物与人的距离很难恰好重合, 而且真正贴近时(目标距离 < 0.6m)
    仍有独立的"防撞人"退避逻辑, 不会因此撞人。
  · approach_speed 0.32 -> 0.38, 给 0.32m/s 步速留出追赶余量。
"""
import json
import shutil
import sys

PF = '/home/earl/workspace/VLA/vla_ws/src/vla_follow/vla_follow/follower.py'
shutil.copy(PF, PF + '.bak_target')
s = open(PF, encoding='utf-8').read()

# ---- 1. 函数签名 ----
OLD = "    def _safety_gate(self, v_t, w_t, now):"
NEW = "    def _safety_gate(self, v_t, w_t, now, tgt_d=None, tgt_b=None):"
if s.count(OLD) != 1:
    print('!! 签名锚点异常')
    sys.exit(1)
s = s.replace(OLD, NEW, 1)

# ---- 2. 摘出目标 ----
OLD = """        d_head = min(d_f, min(d_fl, d_fr) * 0.72)"""
NEW = """        # P31: 把"被跟随的人"从障碍里摘出去。
        # 跟到 1.0m 时人的躯干表面距相机只有 ~0.7m, 低于 slow_d(1.6), 安全
        # 层于是把要跟随的人当成障碍, 一路把速度压到 0.06~0.12m/s(实测 111s
        # 里 259 条全向限速), 机器人只能慢慢爬, 人一走动就掉队。
        if tgt_d is not None and 0.25 < tgt_d < 6.0:
            expect = max(tgt_d - 0.30, 0.25)      # 人躯干表面的期望距离
            lim = slow_d * 1.02                   # 抬到限速阈值之上
            if abs(tgt_b) <= math.radians(45.0) and abs(d_f - expect) < 0.45:
                d_f = max(d_f, lim)
            if tgt_b > math.radians(-10.0) and abs(d_fl - expect) < 0.45:
                d_fl = max(d_fl, lim)
            if tgt_b < math.radians(10.0) and abs(d_fr - expect) < 0.45:
                d_fr = max(d_fr, lim)
        d_head = min(d_f, min(d_fl, d_fr) * 0.72)"""
if s.count(OLD) != 1:
    print('!! d_head 锚点异常')
    sys.exit(1)
s = s.replace(OLD, NEW, 1)

# ---- 3. 两个出口传入目标信息 ----
OLD = """            v_t, w_t, _sn = self._safety_gate(v_t, w_t, now)
            if _sn:
                note += ' ' + _sn"""
NEW = """            v_t, w_t, _sn = self._safety_gate(v_t, w_t, now, dist, brg)
            if _sn:
                note += ' ' + _sn"""
if s.count(OLD) != 1:
    print('!! ALIGN 出口锚点异常')
    sys.exit(1)
s = s.replace(OLD, NEW, 1)

OLD = """        # 最终裁剪: 任何路径算出的速度都要过这一关
        v_t, w_t, _sn = self._safety_gate(v_t, w_t, now)"""
NEW = """        # 最终裁剪: 任何路径算出的速度都要过这一关
        v_t, w_t, _sn = self._safety_gate(v_t, w_t, now, dist, brg)"""
if s.count(OLD) != 1:
    print('!! TRACK 出口锚点异常')
    sys.exit(1)
s = s.replace(OLD, NEW, 1)

# ---- 4. approach_speed ----
OLD = "    'approach_speed': 0.2,"
if s.count(OLD) != 1:
    print('!! approach_speed 锚点异常')
    sys.exit(1)
s = s.replace(OLD, "    'approach_speed': 0.38,", 1)

open(PF, 'w', encoding='utf-8').write(s)
print('PATCH_OK  gate签名:', s.count('tgt_d=None'), ' 传参:', s.count('now, dist, brg'))

# ---- 5. vision_config.json 同步 ----
CP = '/home/earl/workspace/VLA/vla_demo/vision_config.json'
c = json.load(open(CP, encoding='utf-8'))
c['approach_speed'] = 0.38
json.dump(c, open(CP, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
print('CONFIG_OK approach_speed =', c['approach_speed'])
