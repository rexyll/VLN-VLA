#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P48 —— 让绕障真正能走起来（三处配套）。

实测(日志):
  `绕障右 前0.90 侧0.76 目标2.12 v*0.40 [左0.64 右1.09 左前1.49 右前0.76]`
  `指令v=+0.00  w=+0.0`  机器人一动不动。

三个缺陷:
 ① `_aim = brg + avoid_deg*side`。目标在左 72°、选右绕时
    `_aim = 72° - 72° = 0` —— 车头"恰好对着目标"，而目标方向正是被挡
    的方向, 于是 w=0、车头不转、`d_head` 永远不改善 → 永久急停。
    绕行方向必须以**车头**为基准(`_aim = ±avoid_deg`), 而不是"目标方位+偏角"。
 ② P44 的斜前方折算 `d_head = min(d_f, min(d_fl,d_fr)*0.72)` 在绕障时
    是致命的: 车头已故意朝侧方偏 72°, 左前方**必然**是"正在绕的那个
    障碍"(0.76m → 折算 0.547 < stop_d 0.66) → 直接急停。绕障时必须
    只看车头正前方。
 ③ 退出条件用 `dist < d0 - 0.05` 不可靠(绕行必然会先远离目标)。改成
    "目标方位方向余量恢复(能直接看到/走向人)"才退出 —— 这才对应
    "绕到能重新看见人"。
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


# ---- ① + ③: 方向余量查询 + 新的退出判据 ----
sub('''        _df, _dfl, _dfr = _q('front'), _q('fl'), _q('fr')
        _dl, _dr = _q('left'), _q('right')
        _room_l = min(_dl, _dfl)
        _room_r = min(_dr, _dfr)
        _stop = float(self.cfg['lidar_stop_m'])
        _avoid_now = False''',
    '''        _df, _dfl, _dfr = _q('front'), _q('fl'), _q('fr')
        _dl, _dr = _q('left'), _q('right')
        _room_l = min(_dl, _dfl)
        _room_r = min(_dr, _dfr)
        _stop = float(self.cfg['lidar_stop_m'])
        _avoid_now = False

        def _dir_min(b):
            """P48: 方位角 b(相对车头, + = 左)方向上的最小余量。

            8 扇区中心是 0/±45/±90/±135/180, 取包围该角度的两个扇区的
            较小值(保守)。用于判断"目标方向是否已经通了" —— 通了就说明
            绕障可以把车头转回目标, 不必再用 dist 判据(绕行必然先远离)。
            """
            dd = (math.degrees(b) + 360.0) % 360.0
            tab = [(0.0, _q('front')), (45.0, _q('fl')),
                   (90.0, _q('left')), (135.0, _q('bl')),
                   (180.0, _q('back')), (225.0, _q('br')),
                   (270.0, _q('right')), (315.0, _q('fr')),
                   (360.0, _q('front'))]
            best = 99.0
            for i in range(len(tab) - 1):
                a0, v0 = tab[i]
                a1, v1 = tab[i + 1]
                if a0 - 22.5 <= dd <= a1 + 22.5:
                    best = min(best, v0, v1)
            return best''',
    'P48-1 方向余量函数')

sub('''                _clear = (_df > float(self.cfg['lidar_slow_m'])
                          * float(self.cfg['avoid_exit_v'])
                          and min(_dfl, _dfr) > 0.85)
                _tmo = now - self._avoid_t0 > float(self.cfg['avoid_max_s'])
                if (_clear and dist < self._avoid_d0 - 0.05) or _tmo:''',
    '''                # P48: 退出判据改成"目标方向已经通了"。原来用
                # `dist < _avoid_d0 - 0.05` —— 但绕行的第一步必然是
                # 先远离目标(往侧方走), 这个条件几乎永远不成立, 只能等
                # avoid_max_s 超时换边, 表现为"绕 9s 没效果再换边绕"。
                _clear_goal = (_dir_min(brg)
                               > float(self.cfg['lidar_slow_m']) * 0.80)
                _tmo = now - self._avoid_t0 > float(self.cfg['avoid_max_s'])
                if _clear_goal or _tmo:''',
    'P48-2 退出判据')

sub("""                    self.get_logger().info(
                        'FOLLOWER ✓ 绕障退出(历时%.1fs, %s, 目标%.2fm) → 恢复指向'
                        % (now - self._avoid_t0,
                           '超时换边' if _tmo else '前方畅通', dist))""",
    """                    self.get_logger().info(
                        'FOLLOWER ✓ 绕障退出(历时%.1fs, %s, 目标%.2fm) → 恢复指向'
                        % (now - self._avoid_t0,
                           '超时换边' if _tmo else '目标方向已畅通', dist))""",
    'P48-3 退出日志')

# ---- ①: 绕行方向以车头为基准 ----
sub('''        if _avoid_now:
            _side_room = _room_l if self._avoid_side > 0 else _room_r
            _aim = wrap(brg + math.radians(float(self.cfg['avoid_deg']))
                        * self._avoid_side)
            w_t = max(-1.7, min(1.7, 2.4 * _aim))''',
    '''        if _avoid_now:
            _side_room = _room_l if self._avoid_side > 0 else _room_r
            # P48 关键: 绕行偏转角以**车头**为基准, 不是"目标方位 + 偏角"。
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
            w_t = max(-1.7, min(1.7, 2.4 * _aim))''',
    'P48-4 绕向以车头为基准')

# ---- ②: 绕障时 d_head 不做斜前方折算 ----
sub('''        _d_diag = min(d_fl, d_fr)
        d_head = min(d_f, _d_diag * 0.72) if _d_diag < 0.85 else d_f''',
    '''        _d_diag = min(d_fl, d_fr)
        if self._avoiding:
            # P48: 绕障时车头已**故意**朝侧方偏了 avoid_deg, 左前方或右前方
            # 必然就是"正在绕的那个障碍" —— 若还按 0.72 折算成"正前方很近",
            # 就会直接急停。实测 `左前0.76` 折算成 0.547 < stop_d 0.66,
            # 绕障算出的 v=0.40 被压成 0, 车一动不动。绕障期间只看车头
            # 正前方的真实余量。
            d_head = d_f
        else:
            d_head = min(d_f, _d_diag * 0.72) if _d_diag < 0.85 else d_f''',
    'P48-5 绕障不做斜前折算')

io.open(SRC, 'w', encoding='utf-8', newline='\n').write(s)
print('P48 应用完成 %d 处' % n)
import ast
ast.parse(io.open(SRC, encoding='utf-8').read())
print('语法 OK')
