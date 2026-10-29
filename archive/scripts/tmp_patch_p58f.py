#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P58f: 修 pyflakes 找出的两个真 bug。

1) 我插入的"安全距硬闸"用了 note, 但 note='' 在它后面才执行 -> NameError;
2) `_gap6` 未定义 —— P53 把真值字段改成 path_free 时删掉了间隙计算,
   而 note 字符串里还留着 %_gap6。这个分支一旦触发(间隙 <0.33m 且
   not tight), 整个 follower 直接崩, 不是"少打一行日志"。
"""
import io

P = '_p58_follower.py'
s = io.open(P, encoding='utf-8').read()

OLD = """        _clr = float(self.cfg['obstacle_clear_m'])"""
assert s.count(OLD) == 1
s = s.replace(OLD, """        note = ''
        _clr = float(self.cfg['obstacle_clear_m'])""", 1)

OLD2 = """        side_d = self.cfg['lidar_side_m']
        note = ''
"""
assert s.count(OLD2) == 1
s = s.replace(OLD2, """        side_d = self.cfg['lidar_side_m']
""", 1)

OLD3 = """                    elif _obj6 is not None:
                        # P45 关键: 必须给转向。停车 + w=0 = 永久死锁。
                        _lat6 = (-(_obj6[1] - rx) * math.sin(ryaw)
                                 + (_obj6[2] - ry) * math.cos(ryaw))"""
assert s.count(OLD3) == 1
s = s.replace(OLD3, """                    elif _obj6 is not None:
                        # P45 关键: 必须给转向。停车 + w=0 = 永久死锁。
                        # P58: _gap6 原来未定义(P53 删掉了计算但 note 里还
                        # 在引用), 触发即 NameError 崩节点。这里现算。
                        _gap6 = geo_gap(rx, ry, _obj6) - ROBOT_R
                        _lat6 = (-(_obj6[1] - rx) * math.sin(ryaw)
                                 + (_obj6[2] - ry) * math.cos(ryaw))""", 1)

# 清理两个"赋值但未使用"的局部变量
OLD4 = "        band = math.radians(self.cfg['aim_band_deg'])\n"
if s.count(OLD4) == 1:
    s = s.replace(OLD4, "", 1)

io.open(P, 'w', encoding='utf-8', newline='\n').write(s)
print('f ok')
