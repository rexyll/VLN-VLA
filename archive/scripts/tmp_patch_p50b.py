#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P50 —— 侧翼余量不得直接禁行（本次问题的总根源）。"""
import ast
import io
import os
import sys

SRC = os.environ.get('VLA_SRC', '/home/earl/workspace/VLA/vla_ws/src/'
                     'vla_follow/vla_follow/follower.py')
L = io.open(SRC, encoding='utf-8').read().split('\n')

i = None
for k, ln in enumerate(L):
    if ln.strip() == "d_wing = min(d_l, d_r, _pv('bl'), _pv('br'))":
        i = k
        break
if i is None:
    print('!! 找不到 d_wing 行')
    sys.exit(2)
print('找到第 %d 行' % (i + 1))
for k in range(i, i + 9):
    print('   %d| %s' % (k + 1, L[k]))

# 原始 8 行: d_wing / if / 三条注释 / v_t=0 / if abs / w_t=
assert L[i + 1].strip() == 'if d_wing < stop_d:', L[i + 1]
assert L[i + 7].strip() == 'w_t = 0.9 if d_r < d_l else -0.9', L[i + 7]

NEW = """                d_wing = min(d_l, d_r, _pv('bl'), _pv('br'))
                # P50: 侧翼余量只能**限速**, 不能直接禁行。
                # 原判据 `d_wing < stop_d(0.66) → v_t = 0` 是拿**正前方**
                # 的停车距离去判**侧向** —— 机器人沿墙/沿障碍正常直行时,
                # 侧向只有 0.5~0.6m 完全正常, 却会被永久禁行。
                # 实测(日志): 机器人 y=-5.39、右侧到墙 0.54m、正前方还有
                # 0.98m 空间、限速允许 0.54m/s, 结果 `指令v=+0.00` 恒为 0,
                # 26s 一动不动 → 视觉丢失 → GOTO/SCAN → 打转。
                # 这条规则正是"人一走到障碍附近, 机器人就卡住并丢目标"的
                # 总根源: 人靠近障碍 → 机器人跟着靠近 → 侧翼 < 0.66 →
                # 永久禁行 → 掉队 → 丢失。
                _wing_min = stop_d * 0.55          # 0.363m: 真正贴死才停
                if d_wing < _wing_min:
                    # P35: 原实现"限速到 0.10 继续挤"在实测中会慢慢钻进
                    # 立柱(0.10m/s x 15s = 1.5m, gap 走到 -0.42)。侧翼真
                    # 的贴死时必须停止前进, 并强制往空的一侧转。
                    v_t = 0.0
                    if abs(w_t) < 0.35:
                        w_t = 0.9 if d_r < d_l else -0.9
                elif d_wing < stop_d:
                    # 侧翼偏窄但不至于贴死: 只限速, 允许沿墙/沿障继续直行
                    _v_wing = max(0.12, (_wing_min + 0.30 - d_wing) / 0.45)
                    if v_t > _v_wing:
                        v_t = _v_wing
                    note = (note + ' 侧翼限速 侧%.2f v<=%.2f'
                            % (d_wing, _v_wing)).strip()"""

L[i:i + 8] = NEW.split('\n')
s = '\n'.join(L)
io.open(SRC, 'w', encoding='utf-8', newline='\n').write(s)
ast.parse(s)
print('P50 应用完成: 侧翼禁行 → 侧翼限速')
print('语法 OK')
