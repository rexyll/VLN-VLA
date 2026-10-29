#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P40b: 把 v_cap 的定义提到带内分支之前

P40 的带内分支要用 v_cap 做驻留限速, 但 v_cap 原来定义在 else 分支里,
第一次进带内会 NameError; 顺带删掉 else 里那行旧的前馈(pspd 版),
它会把 P40 新算的 v_ff 覆盖掉。
"""
import io
import os
import sys

P = os.environ.get('VLA_SRC') or '_p40_follower.py'
src = io.open(P, encoding='utf-8').read()

OLD = """        if abs(err_d) <= tol:
            # P40: 带内**不能停**。人只要还在走, 一停就掉出带。
            # 原来这里 v_t=0.0 且把积分清零, 于是永远在"停-追-停"之间摆动。
            v_t = max(-0.25, min(v_cap, v_ff))
            self._prev_err_d = err_d
            self._int_d *= 0.995              # 带内缓慢泄放, 不硬清零
        else:
            self._int_d = max(-1.2, min(1.2,
                               self._int_d + self.cfg['pid_ki_v'] * err_d * dtc))
            der_d = (err_d - self._prev_err_d) / dtc
            self._prev_err_d = err_d
            v_pid = (self.cfg['pid_kp_v'] * err_d + self._int_d
                     + self.cfg['pid_kd_v'] * der_d + v_ff)
            # P22 修正: 原式 v_cap = max(approach_speed, pspd+0.25) 在目标快跑时
            # 会把上限抬到 0.9 m/s 以上。而视觉距离只有 5 Hz、且允许 0.8s 陈旧,
            # 0.6 m/s 意味着最多 0.5m 的"盲走" —— 实测直冲 0.58m(安全距 0.65),
            # 差点撞人。这里改成"接近速度"与"视觉时延内不冲进安全距"取小:
            #   v_lat = (dist - guard_d) / 视觉时延
            # 目标 1.5m 时 v_lat≈1.06(不限制), 1.0m 时 0.44, 近了自然收。
            lat = 0.8
            room = max(0.02, dist - guard_d)
            v_lat = room / lat
            v_ff = min(0.6, pspd + 0.20) if dist > 2.0 else 0.0
            v_cap = min(max(self.cfg['approach_speed'], v_ff), v_lat)
            v_t = max(-0.35, min(v_cap, v_pid))
"""

NEW = """        # P22 修正: 原式 v_cap = max(approach_speed, pspd+0.25) 在目标快跑时
        # 会把上限抬到 0.9 m/s 以上。而视觉距离只有 5 Hz、且允许 0.8s 陈旧,
        # 0.6 m/s 意味着最多 0.5m 的"盲走" —— 实测直冲 0.58m(安全距 0.65),
        # 差点撞人。这里改成"接近速度"与"视觉时延内不冲进安全距"取小:
        #   v_lat = (dist - guard_d) / 视觉时延
        # 目标 1.5m 时 v_lat≈1.06(不限制), 1.0m 时 0.44, 近了自然收。
        # P40: 这段提到带内/带外的分叉之前 —— 带内驻留也要受同一个限速约束,
        # 否则"人走近时前馈仍按人速推车"会顶进安全距。
        lat = 0.8
        room = max(0.02, dist - guard_d)
        v_lat = room / lat
        v_cap = min(max(self.cfg['approach_speed'], v_ff), v_lat)
        if abs(err_d) <= tol:
            # P40: 带内**不能停**。人只要还在走, 一停就掉出带。
            # 原来这里 v_t=0.0 且把积分清零, 于是永远在"停-追-停"之间摆动。
            v_t = max(-0.25, min(v_cap, v_ff))
            self._prev_err_d = err_d
            self._int_d *= 0.995              # 带内缓慢泄放, 不硬清零
        else:
            self._int_d = max(-1.2, min(1.2,
                               self._int_d + self.cfg['pid_ki_v'] * err_d * dtc))
            der_d = (err_d - self._prev_err_d) / dtc
            self._prev_err_d = err_d
            v_pid = (self.cfg['pid_kp_v'] * err_d + self._int_d
                     + self.cfg['pid_kd_v'] * der_d + v_ff)
            v_t = max(-0.35, min(v_cap, v_pid))
"""

if NEW in src:
    print('已存在, 跳过')
    sys.exit(0)
if src.count(OLD) != 1:
    print('!! 匹配 %d 次' % src.count(OLD))
    sys.exit(1)
src = src.replace(OLD, NEW)
io.open(P, 'w', encoding='utf-8', newline='\n').write(src)
print('P40b OK, %d 字节' % len(src))

# 顺带确认没有遗留的 pspd 版本前馈
for i, ln in enumerate(src.split('\n')):
    if 'pspd + 0.20' in ln or 'pspd+' in ln:
        print('  !! 仍残留: %d %s' % (i + 1, ln.strip()))
