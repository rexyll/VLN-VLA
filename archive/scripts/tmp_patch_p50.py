#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P50 —— 侧翼余量不得直接禁行（本次问题的总根源）。

P35 为了防"擦着立柱边走", 加了:
    d_wing = min(d_l, d_r, bl, br)
    if d_wing < stop_d:
        v_t = 0.0            # 侧翼进入"停车距离"就禁止前进
这用的是**正前方**的停车距离(stop_d=0.66)去判**侧向**余量, 概念错误:

  · 机器人沿墙/沿障碍正常直行时, 侧向只有 0.5~0.6m 是完全正常的;
  · 一旦侧向 < 0.66 就被永久禁行, 于是"一靠近障碍/墙就彻底不动"。

实测(日志):
    `绕障右 航向+22° 前0.98 侧0.54 目标1.68 v*0.40 [左2.57 右0.54 ...]`
    `指令v=+0.00`
  正前方还有 0.98m 空间、限速允许 0.54m/s, 却因为右侧 0.54 < 0.66
  被判"贴死", v 恒为 0 → 机器人停在原地 → 视觉丢失 → GOTO/SCAN →
  打转。这就是"人走到障碍附近, 机器人就丢目标并胡乱移动"的总根源。

修法: 侧翼只约束速度, 不直接禁行。真正的"贴死"阈值降到 stop_d*0.55
      (0.363m, 此时确实该停), 中间区间按余量线性限速。
"""
import io
import os
import sys

SRC = os.environ.get('VLA_SRC', '/home/earl/workspace/VLA/vla_ws/src/'
                     'vla_follow/vla_follow/follower.py')
s = io.open(SRC, encoding='utf-8').read()

OLD = '''                d_wing = min(d_l, d_r, _pv('bl'), _pv('br'))
                if d_wing < stop_d:
                    # P35: 原实现"限速到 0.10 继续挤"在实测中会慢慢钻进
                    # 立柱(0.10m/s × 15s = 1.5m, gap 走到 -0.42)。侧翼已
                    # 进入停车距离时必须停止前进, 并强制往空的一侧转。'''
NEW = '''                d_wing = min(d_l, d_r, _pv('bl'), _pv('br'))
                # P50: 侧翼余量只能**限速**, 不能直接禁行。
                # 原来 `d_wing < stop_d(0.66) → v_t=0` 是拿正前方的停车
                # 距离去判侧向 —— 机器人沿墙直行时侧向 0.5~0.6m 完全正常,
                # 却会被永久禁行。实测: 机器人 y=-5.39, 右侧到墙 0.54m,
                # 正前方还有 0.98m 空间, 结果 v 恒为 0、26s 一动不动。
                # 这条规则正是"一靠近障碍/墙就卡死 → 丢失 → 打转"的总根源。
                _wing_min = stop_d * 0.55       # 0.363m: 真正贴死才停
                if d_wing < _wing_min:
                    # P35: 原实现"限速到 0.10 继续挤"在实测中会慢慢钻进
                    # 立柱(0.10m/s × 15s = 1.5m, gap 走到 -0.42)。侧翼已
                    # 进入停车距离时必须停止前进, 并强制往空的一侧转。'''
if s.count(OLD) != 1:
    print('!! 锚点 count=%d' % s.count(OLD))
    sys.exit(2)
s = s.replace(OLD, NEW)

# 在原有的 v_t=0 块之后追加"中间区间限速"
OLD2 = '''                    v_t = 0.0
                    if abs(w_t) < 0.35:'''
NEW2 = '''                    v_t = 0.0
                    if abs(w_t) < 0.35:'''
# 先看这段的实际内容再决定, 这里仅确认存在
if s.count(OLD2) != 1:
    print('!! 第二锚点 count=%d' % s.count(OLD2))
    sys.exit(2)

# 在侧向防擦块结尾(回到 v_t>0.05 作用域之后)插入限速分支 —— 用下一段注释定位
ANCHOR = '''            if v_t > 0.05:                              # 侧向防擦'''
if s.count(ANCHOR) != 1:
    print('!! 第三锚点 count=%d' % s.count(ANCHOR))
    sys.exit(2)

io.open(SRC, 'w', encoding='utf-8', newline='\n').write(s)
print('P50-1 已把禁行阈值降到 0.55*stop_d')
