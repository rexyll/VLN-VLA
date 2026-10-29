#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P60b-2: 倾斜标志 + 急停看门狗 + 深度异常识别。"""
import io

P = '_p58_follower.py'
s = io.open(P, encoding='utf-8').read()

# ---------------- 1) 姿态回调里记"翘头"标志 ----------------
OLD = """            self._tilt = max(abs(math.degrees(_pr)), abs(math.degrees(_rl)))"""
NEW = """            self._tilt = max(abs(math.degrees(_pr)), abs(math.degrees(_rl)))
            # pitch>0 = 抬头(前轮离地), 靠倒车压平
            self._tilt_up = (math.degrees(_pr) > 0.0)"""
assert s.count(OLD) == 1
s = s.replace(OLD, NEW, 1)

OLD = """        self._tilt = 0.0          # 当前车体倾斜角(取 pitch/roll 绝对值最大)"""
NEW = """        self._tilt = 0.0          # 当前车体倾斜角(取 pitch/roll 绝对值最大)
        self._tilt_up = False     # 是否抬头(前轮离地)"""
assert s.count(OLD) == 1
s = s.replace(OLD, NEW, 1)

# ---------------- 2) 深度异常识别 ----------------
OLD = """        d_f, d_fl, d_fr = _pv('front'), _pv('fl'), _pv('fr')
        d_l, d_r = _pv('left'), _pv('right')
        d_b = min(_pv('bl'), _pv('back'), _pv('br'))"""
NEW = """        d_f, d_fl, d_fr = _pv('front'), _pv('fl'), _pv('fr')
        d_l, d_r = _pv('left'), _pv('right')
        d_b = min(_pv('bl'), _pv('back'), _pv('br'))
        # P60b: 8 扇区同时读到"极小且几乎相等" -> 不是障碍, 是相机在看
        # 地面/天花板(车体歪了、或深度图异常)。12x12 场地只有 7 个障碍,
        # 不可能四面都是 0.33m。这种读数下继续跑只会永久急停+原地打转,
        # 所以直接判为感知异常 -> 停车等待。
        _all8 = [d_f, d_fl, d_fr, d_l, d_r, d_b,
                 _pv('bl'), _pv('back'), _pv('br')]
        _mx8 = max(_all8)
        if _mx8 < 0.75 and self._tilt < 12.0:
            self._bad_depth = now - self._bad_depth if self._bad_depth \\
                else 0.0
            if self._bad_depth == 0.0:
                self._bad_depth = 1e-9
            if now - self._bad_depth > 0.8:
                v_t = 0.0
                w_t = 0.0
                return v_t, w_t, ('深度异常(8扇区均<%.2fm, 疑似相机看地/看天) '
                                 '已停车' % _mx8)"""
assert s.count(OLD) == 1
s = s.replace(OLD, NEW, 1)

# ---------------- 3) 急停看门狗 ----------------
OLD = """                    w_t = 1.15 * self._esc_dir
                    note = (f'全向急停 前{d_f:.2f} 左前{d_fl:.2f}'
                            f' 右前{d_fr:.2f}'
                            f' 绕{"左" if self._esc_dir > 0 else "右"}')"""
NEW = """                    w_t = 1.15 * self._esc_dir
                    note = (f'全向急停 前{d_f:.2f} 左前{d_fl:.2f}'
                            f' 右前{d_fr:.2f}'
                            f' 绕{"左" if self._esc_dir > 0 else "右"}')
                    # P60b: 急停看门狗 —— 连续急停且位置毫无进展时, 原地
                    # 转向救不了(转多少度还是被挡)。改成后退让出空间, 再
                    # 重新对正/重规划, 避免"卡住->打转->还是卡住"的死循环。
                    _xy = (self.rx, self.ry)
                    if self._esc_stuck_t0 <= 0.0:
                        self._esc_stuck_t0 = now
                        self._esc_last_xy = _xy
                    elif self._esc_last_xy is not None:
                        _mv = math.hypot(self.rx - self._esc_last_xy[0],
                                         self.ry - self._esc_last_xy[1])
                        if _mv < 0.10:
                            # 位置没变 -> 判定卡死
                            if now - self._esc_stuck_t0 > 3.0:
                                if d_b > stop_d * 0.9:
                                    v_t = -0.28
                                    w_t = 0.0
                                    note = (note + ' 卡死3s→后退让位').strip()
                                    self._esc_stuck_t0 = now + 2.0
                                else:
                                    # 后方也堵 -> 换边继续试
                                    self._esc_dir = -self._esc_dir
                                    self._esc_stuck_t0 = now
                                    self._esc_last_xy = _xy
                        else:
                            self._esc_last_xy = _xy
                            self._esc_stuck_t0 = now"""
assert s.count(OLD) == 1
s = s.replace(OLD, NEW, 1)

# ---------------- 4) 急停解除时清看门狗 ----------------
OLD = """        stop_d = self.cfg['lidar_stop_m']
        slow_d = self.cfg['lidar_slow_m']"""
NEW = """        if v_t > 0.02:            # 恢复前进 -> 看门狗复位
            self._esc_stuck_t0 = 0.0
            self._esc_last_xy = None
        stop_d = self.cfg['lidar_stop_m']
        slow_d = self.cfg['lidar_slow_m']"""
assert s.count(OLD) == 1
s = s.replace(OLD, NEW, 1)

io.open(P, 'w', encoding='utf-8', newline='\n').write(s)
print('P60b part2 ok')
