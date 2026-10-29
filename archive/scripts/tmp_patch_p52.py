#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P52 —— VFH 加 0.4s 世界系锁存, 消除正反馈振荡。

P51 的纯 VFH 会兜圈子: 扇区表锚定在**车头系**, 车头左转 15° 后目标方位
也少 15°, 而"最优扇区"还是那个 45° 扇区 —— 误差恒为 30°, w 恒为正,
于是原地/绕圈打转(实测位移 0.70m 但 dist 在 1.48~2.26 之间震荡, 从未接近)。

修法(也是经典 VFH 的做法): 选定扇区后把方向转成**世界角锁存 0.4s**,
期间按该世界角收敛(误差到 0 就停), 0.4s 后重新评估。这样既不会正反馈,
又能在走偏后修正 —— P49 "只在进入时锁一次" 的问题是走偏了不修正。
"""
import ast
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


# 状态: _avoid_hdg 改成 _vfh_w(世界系锁存) + _vfh_t + _vfh_hint
sub("""        self._avoid_hdg = 0.0         # 绕行目标航向(世界系)
        self._avoid_hdg_t = 0.0       # 锁存时刻""",
    """        self._vfh_w = None            # P52: VFH 选定的世界系方向(0.4s 锁存)
        self._vfh_t = 0.0             # 上次重选时刻
        self._vfh_hint = ''           # 日志用
        self._avoid_hdg = 0.0         # 兼容保留
        self._avoid_hdg_t = 0.0""",
    'P52-1 状态')

# 控制律
sub("""            # P51: VFH —— 在"过得去"的扇区里选最接近目标方向的。
            # 前几版失败都在"绕行方向怎么定"上(P48 固定偏角会顶死,
            # P49 锁存航向会走偏且误判退出)。VFH 不需要锁存任何东西:
            # 8 个扇区就是极坐标直方图, 每 tick 重算, 天然处理"目标在
            # 障碍后面"。对"身后方向"加惩罚, 避免掉头绕远路。
            _pass_m = max(0.95, _stop + 0.30)
            _goal_d = (math.degrees(brg) + 360.0) % 360.0
            _tab8 = [(0.0, 'front'), (45.0, 'fl'), (90.0, 'left'),
                     (135.0, 'bl'), (180.0, 'back'), (225.0, 'br'),
                     (270.0, 'right'), (315.0, 'fr')]
            _vb = None
            for _a, _k in _tab8:
                _vv = _q(_k)
                if _vv < _pass_m:
                    continue
                _c = abs(((_a - _goal_d + 180.0) % 360.0) - 180.0)
                _as = ((_a + 180.0) % 360.0) - 180.0
                if abs(_as) > 110.0:      # 身后方向: 加惩罚, 别掉头
                    _c += 45.0
                if _vb is None or _c < _vb[0]:
                    _vb = (_c, _a, _vv)
            if _vb is not None:
                _err_hdg = wrap(math.radians(_vb[1]) - brg)
                w_t = max(-1.6, min(1.6, 2.2 * _err_hdg))
                v_t = min(float(self.cfg['approach_speed']) * 0.72, 0.38,
                          max(float(self.cfg['avoid_min_v']),
                              (_vb[2] - _stop + 0.12) / 0.45))
                _hint = '扇区%+.0f° 余%.2f' % (_vb[1], _vb[2])
            else:
                # 四面都不够宽 → 原地慢转找缺口
                w_t = 1.0
                v_t = 0.0
                _hint = '无可行扇区→慢转'""",
    """            # P52: VFH + 0.4s 世界系锁存。
            # 纯 VFH(P51)会正反馈振荡: 扇区表锚定在**车头系**, 车头左转
            # 后目标方位同步减少, 而"最优扇区"仍是同一个, 误差恒为正 →
            # w 恒为正 → 绕圈(实测位移 0.70m, dist 却在 1.48~2.26 震荡,
            # 从未接近过目标)。
            # 现在: 选定后转成世界角锁存 0.4s, 期间按该世界角收敛(误差到
            # 0 即停), 0.4s 后重选 —— 既断开正反馈, 又能在走偏后修正
            # (P49 只在进入时锁一次, 走偏了不修正)。
            _pass_m = max(0.95, _stop + 0.30)
            if self._vfh_w is None or now - self._vfh_t > 0.40:
                _goal_d = (math.degrees(brg) + 360.0) % 360.0
                _tab8 = [(0.0, 'front'), (45.0, 'fl'), (90.0, 'left'),
                         (135.0, 'bl'), (180.0, 'back'), (225.0, 'br'),
                         (270.0, 'right'), (315.0, 'fr')]
                _vb = None
                for _a, _k in _tab8:
                    _vv = _q(_k)
                    if _vv < _pass_m:
                        continue
                    _c = abs(((_a - _goal_d + 180.0) % 360.0) - 180.0)
                    _as = ((_a + 180.0) % 360.0) - 180.0
                    if abs(_as) > 110.0:      # 身后方向: 加惩罚, 别掉头
                        _c += 45.0
                    if _vb is None or _c < _vb[0]:
                        _vb = (_c, _a, _vv)
                self._vfh_t = now
                if _vb is not None:
                    self._vfh_w = wrap(ryaw + math.radians(_vb[1]))
                    self._vfh_hint = ('扇区%+.0f° 余%.2f'
                                      % (_vb[1], _vb[2]))
                else:
                    self._vfh_w = None
                    self._vfh_hint = '无可行扇区'
            if self._vfh_w is not None:
                _err_hdg = wrap(self._vfh_w - ryaw)
                w_t = max(-1.6, min(1.6, 2.2 * _err_hdg))
                v_t = min(float(self.cfg['approach_speed']) * 0.72, 0.38,
                          max(float(self.cfg['avoid_min_v']),
                              (_df - _stop + 0.12) / 0.45))
                _hint = self._vfh_hint
            else:
                # 四面都不够宽 → 原地慢转找缺口
                w_t = 1.0
                v_t = 0.0
                _hint = self._vfh_hint""",
    'P52-2 VFH 锁存')

# 退出绕障时清空锁存
sub("""                if _clear_goal or _tmo:
                    self._avoiding = False
                    self._blk2_t0 = 0.0""",
    """                if _clear_goal or _tmo:
                    self._avoiding = False
                    self._vfh_w = None
                    self._blk2_t0 = 0.0""",
    'P52-3 退出清锁')

io.open(SRC, 'w', encoding='utf-8', newline='\n').write(s)
print('P52 应用完成 %d 处' % n)
ast.parse(io.open(SRC, encoding='utf-8').read())
print('语法 OK')
