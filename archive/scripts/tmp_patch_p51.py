#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P51 —— 绕障改为 VFH(向量场直方图): 每 tick 在"过得去的方向"里选最接近目标的。

前几版都栽在"绕行方向怎么定":
  P48 用"目标方位 + 固定偏角" → 目标已在侧方 ±72° 时 _aim≈0, 车头不转;
  P49 用"世界系锁存航向"     → 锁死一个方向, 走偏了不会修正,
                                绕障 1s 就因"目标方向通了"的误判退出;
  P46/P47 的侧别锁存          → 选错边就一路错到底。

VFH 的做法(这也是移动机器人里最成熟的一类局部规划):
  1. 8 个 45° 扇区就是现成的极坐标直方图;
  2. 每 tick 在所有"余量 >= pass_m"的扇区里, 选**最接近目标方位**的那个;
  3. 车头朝该扇区转, 速度按该扇区余量给。
不需要锁存侧别、不需要锁存航向、不需要切点 —— 天然处理"目标在障碍后"。
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


# --- 用 VFH 替换"锁存航向"的控制律 ---
sub("""            # P49: 按**世界系锁存航向**的误差控制转向, 误差收敛到 0 即
            # 车头停在绕行方向直着走。P48 用"相对车头的固定偏角"会让 w
            # 恒为正 → 匀速转圈(实测 26s 转不停, 位移 0.07m)。
            _err_hdg = wrap(self._avoid_hdg - ryaw)
            w_t = max(-1.7, min(1.7, 2.2 * _err_hdg))
            # 速度按**车头正前方**的真实余量给(绕行方向就是车头方向)
            v_t = min(float(self.cfg['approach_speed']) * 0.72, 0.40,
                      max(float(self.cfg['avoid_min_v']),
                          (_df - _stop + 0.12) / 0.45))""",
    """            # P51: VFH —— 在"过得去"的扇区里选最接近目标方向的。
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
    'P51-1 VFH 控制律')

# 日志换掉航向字段
sub("""            _dn = ('绕障%s 航向%+.0f°(差%+.0f°) 前%.2f 侧%.2f 目标%.2f '
                   'v*%.2f [左%.2f右%.2f 左前%.2f右前%.2f]'
                   % ('左' if self._avoid_side > 0 else '右',
                      math.degrees(self._avoid_hdg) % 360.0,
                      math.degrees(_err_hdg), _df, _side_room, dist, v_t,
                      _dl, _dr, _dfl, _dfr))""",
    """            _dn = ('绕障[%s] 前%.2f 侧%.2f 目标%.2f v*%.2f '
                   '左%.2f右%.2f 左前%.2f右前%.2f 后%.2f'
                   % (_hint, _df, _side_room, dist, v_t, _dl, _dr,
                      _dfl, _dfr, _q('back')))""",
    'P51-2 日志')

io.open(SRC, 'w', encoding='utf-8', newline='\n').write(s)
print('P51 应用完成 %d 处' % n)
ast.parse(io.open(SRC, encoding='utf-8').read())
print('语法 OK')
