#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P42: 距离环去分段 —— 消除 ±tol 边界上的指令跳变

eval9 日志证据(P41 修好人形之后):
  距离=1.07m 方位=+8°  指令v=+0.03
  距离=1.14m 方位=+2°  指令v=+0.39   ← 只差 0.07m, 指令差 13 倍
  距离=1.07m 方位=+9°  指令v=+0.03
  距离=1.15m 方位=+5°  指令v=+0.40

原因: P40 留下了"带内/带外"两个分支 —— 带内直接用前馈(v_ff≈0.03),
带外走 PID(kp=1.9, err=0.15 → 0.29 + 积分 → 0.40)。误差穿过 ±tol 时
指令从 0.03 跳到 0.40, 机器人一脚油门一脚刹车, 距离在 1.06~1.15 之间抖,
永远 settle 不进 0.9~1.1。

改法: 合成一个连续回路 —— 前馈 + PID 始终相加。
  "跟着人走"由前馈 v_ff 提供, "距离对不对"由 PID 提供, 两者相加即可;
  根本不需要再按 ±tol 分叉。只剩一个 ±0.02m 的窄死区吸收测距噪声。
  人匀速走且距离达标时 err≈0 → v = v_ff, 正是理想的驻留行为。
"""
import ast
import io
import os
import sys

P = os.environ.get('VLA_SRC') or '_p42_follower.py'
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
            v_t = max(-0.35, min(v_cap, v_pid))
"""

NEW = """        # P42: 原来这里是"带内/带外"两个分支, 误差一穿过 ±tol, 指令就从
        # 纯前馈(v_ff≈0.03)跳到 PID 输出(≈0.40) —— 实测 1.07m 时 0.03、
        # 1.14m 时 0.39, 差 0.07m 指令就差 13 倍, 机器人一脚油门一脚刹车,
        # 距离在 1.06~1.15 之间抖, 永远 settle 不进 0.9~1.1。
        # 现在合成一个连续回路: 前馈负责"跟着人走", PID 负责"距离对不对",
        # 两者始终相加。人匀速走且距离达标时 err≈0 → v=v_ff, 正是驻留行为,
        # 完全不需要按 tol 分叉。只留 ±0.02m 窄死区吸收测距噪声。
        vtol = 0.02
        if abs(err_d) < vtol:
            err_d = 0.0
        self._int_d = max(-0.9, min(0.9,
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

ast.parse(src)
print('P42 OK, 语法通过, %d 字符' % len(src))
