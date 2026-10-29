#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P49 —— 绕行航向改为世界系锁存（否则原地转圈）。

P48 把 `_aim` 从"目标方位+偏角"改成绝对 ±avoid_deg, 结果更糟:
`_aim` 是**相对车头**的固定偏角, 车头转了它还是 ±72°, w 恒为 1.7,
机器人以 1.7rad/s 匀速转圈(实测 26s 转个不停, 位移 0.07m)。

正确做法: 进入绕障时在**世界系**锁存一个航向 `_avoid_hdg`, 之后控制
`w` 收敛到该航向(误差到 0 就停), 而不是每 tick 再加一个固定偏角。

选向规则:
  · 目标本来就在侧方(|brg|>15°) → 直接朝目标(它本身就不在被挡的方向)
  · 目标在正前方且被挡 → 车头偏转到余量更大的那一侧
"""
import io
import os
import sys

SRC = os.environ.get('VLA_SRC', '/home/earl/workspace/VLA/vla_ws/src/'
                     'vla_follow/vla_follow/follower.py')
s = io.open(SRC, encoding='utf-8').read()
n = 0


def sub(old, new, tag, cnt=1):
    global s, n
    if s.count(old) != cnt:
        print('!! %s 锚点 count=%d' % (tag, s.count(old)))
        sys.exit(2)
    s = s.replace(old, new, cnt)
    n += 1
    print('   ok %s' % tag)


# 1) 新增状态
sub("""        self._blk2_t0 = 0.0           # 绕障触发计时(与脱困用的 _blk_t0 分开)""",
    """        self._blk2_t0 = 0.0           # 绕障触发计时(与脱困用的 _blk_t0 分开)
        # P49: 绕行航向必须在**世界系**锁存。用"相对车头的固定偏角"会让
        # w 恒为 2.4*72°=3.0(钳 1.7), 车头转了偏角还是 72° → 匀速转圈。
        self._avoid_hdg = 0.0         # 绕行目标航向(世界系)
        self._avoid_hdg_t = 0.0       # 锁存时刻""",
    'P49-1 状态')

# 2) 进入绕障时锁存世界航向
sub("""                        self._avoid_side = 1.0 if _room_l >= _room_r else -1.0
                        self._blk2_t0 = 0.0""",
    """                        self._avoid_side = 1.0 if _room_l >= _room_r else -1.0
                        # P49: 锁存世界系绕行航向。
                        # 目标已在侧方 → 直接朝目标(它不在被挡的方向上,
                        #   朝它走本身就是在绕); 目标在正前方被挡 → 车头
                        #   偏转到余量更大的一侧。
                        if abs(brg) > math.radians(15.0):
                            self._avoid_hdg = wrap(ryaw + brg)
                            self._avoid_side = 1.0 if brg > 0 else -1.0
                        else:
                            self._avoid_hdg = wrap(
                                ryaw + self._avoid_side
                                * math.radians(float(self.cfg['avoid_deg'])))
                        self._avoid_hdg_t = now
                        self._blk2_t0 = 0.0""",
    'P49-2 锁存航向')

# 3) 绕行时按航向误差控制 w
sub("""            # P48 关键: 绕行偏转角以**车头**为基准, 不是"目标方位 + 偏角"。
            # 原式在目标已处于侧方(如左 72°)且选右绕时, `brg + (-72°) = 0`
            # —— 车头"恰好对着目标", 而目标方向正是被挡的方向, 于是 w=0,
            # 车头不转, d_head 永远不改善 → 永久急停。
            # 实测: `绕障右 ... v*0.40` 但 `指令v=0.00 w=+0.0`, 机器人
            # 26s 一动不动。
            _aim = math.radians(float(self.cfg['avoid_deg'])) * self._avoid_side
            # 目标本来就在绕行侧(同向): 可以少偏一点, 别绕过过头, 但至少 40°
            if self._avoid_side > 0 and brg > math.radians(40.0):
                _aim = min(_aim, brg - math.radians(40.0))
            elif self._avoid_side < 0 and brg < -math.radians(40.0):
                _aim = max(_aim, brg + math.radians(40.0))
            w_t = max(-1.7, min(1.7, 2.4 * _aim))
            _rm = min(_df, _side_room)
            # 侧方越窄走得越慢, 但不允许停下 —— 一停就退回"原地转"
            v_t = min(float(self.cfg['approach_speed']) * 0.72, 0.40,
                      max(float(self.cfg['avoid_min_v']),
                          (_rm - _stop + 0.12) / 0.45))""",
    """            # P49: 按**世界系锁存航向**的误差控制转向, 误差收敛到 0 即
            # 车头停在绕行方向直着走。P48 用"相对车头的固定偏角"会让 w
            # 恒为正 → 匀速转圈(实测 26s 转不停, 位移 0.07m)。
            _err_hdg = wrap(self._avoid_hdg - ryaw)
            w_t = max(-1.7, min(1.7, 2.2 * _err_hdg))
            # 速度按**车头正前方**的真实余量给(绕行方向就是车头方向)
            v_t = min(float(self.cfg['approach_speed']) * 0.72, 0.40,
                      max(float(self.cfg['avoid_min_v']),
                          (_df - _stop + 0.12) / 0.45))""",
    'P49-3 按航向误差转向')

sub("""            _dn = ('绕障%s 前%.2f 侧%.2f 目标%.2f v*%.2f [左%.2f右%.2f '
                   '左前%.2f右前%.2f 后%.2f]'
                   % ('左' if self._avoid_side > 0 else '右',
                      _df, _side_room, dist, v_t, _dl, _dr, _dfl, _dfr,
                      min(_q('bl'), _q('br'))))""",
    """            _dn = ('绕障%s 航向%+.0f°(差%+.0f°) 前%.2f 侧%.2f 目标%.2f '
                   'v*%.2f [左%.2f右%.2f 左前%.2f右前%.2f]'
                   % ('左' if self._avoid_side > 0 else '右',
                      math.degrees(self._avoid_hdg) % 360.0,
                      math.degrees(_err_hdg), _df, _side_room, dist, v_t,
                      _dl, _dr, _dfl, _dfr))""",
    'P49-4 日志')

io.open(SRC, 'w', encoding='utf-8', newline='\n').write(s)
print('P49 应用完成 %d 处' % n)
import ast
ast.parse(io.open(SRC, encoding='utf-8').read())
print('语法 OK')
