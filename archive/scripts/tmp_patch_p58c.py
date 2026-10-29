#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P58c: 控制段重写 —— 视野基元(TURN/STEER/TRACK) + 局部路径跟踪。"""
import io

P = '_p58_follower.py'
s = io.open(P, encoding='utf-8').read()

# ================= A. 对正判定 -> 视野基元判定 =================
OLD_A = """        # --- 3. 对正判定(P20): 目标是否落在 front 中线 ±band 内 ---
        aim = abs(brg)
        if self._aligning:
            # 退出对正需进入 0.65*band 内, 防边界抖动
            if aim <= band * 0.65:
                self._aligning = False
                self.get_logger().info(
                    f'FOLLOWER ✓ 已对正目标(偏差 {math.degrees(brg):+.1f}°'
                    f' ≤ {self.cfg["aim_band_deg"]:.0f}°), 转入距离跟踪')
        else:
            if aim > band:
                self._aligning = True
                self.get_logger().info(
                    f'FOLLOWER ⟳ 目标偏离中线 {math.degrees(brg):+.1f}°'
                    f' > {self.cfg["aim_band_deg"]:.0f}°, 先原地对正')
        # 扫描态固定小角速度旋转
        if src == '扫描':
            self._aligning = False
"""
assert s.count(OLD_A) == 1, ('A', s.count(OLD_A))

NEW_A = """        # --- 3. 视野基元判定 (P58, 需求1/2) ---
        # 三种基元, 优先级 TURN > PATH > STEER > TRACK:
        #   TURN  —— 目标不在 front 视野(主相机是 left/right/back): 需求2 要求
        #            **先转向**把它请进 front 视野并压到 ±ao_center_deg, 之后
        #            再按距离决定是否移动。v 必须为 0, 否则边转边冲会撞。
        #   STEER —— 目标在 front 视野内但超出 ±10°: 需求1 要求"在确保目标
        #            不抛出视野的前提下先移动", 所以**不停车**, 边走边转;
        #            速度随偏离角线性递减, 偏离越大越慢, 转向才跟得上。
        #   TRACK —— 已压在 ±10° 内: 距离环决定前进/后退/保持。
        aim = abs(brg)
        _center = math.radians(float(self.cfg['ao_center_deg']))
        _fov = math.radians(float(self.cfg['ao_front_fov_deg']))
        if src.startswith('视觉-'):
            _in_front = False            # 主相机是 left / right / back
        else:
            _in_front = aim <= _fov      # 视觉(front) / 搜索 / 丢失都按方位判
        if not _in_front:
            _prim = 'TURN'
        elif aim <= _center:
            _prim = 'TRACK'
        else:
            _prim = 'STEER'
        if _prim != self._prim_prev:
            _prevp, self._prim_prev = self._prim_prev, _prim
            _cn = {'front': '前', 'left': '左', 'right': '右',
                   'back': '后'}.get(self._fcam, self._fcam or '-')
            if _prim == 'TURN':
                self.get_logger().info(
                    'FOLLOWER ⟳ 目标不在 front 视野(主相机[%s] 偏离%+.0f°)'
                    ' → 先转向请入 ±%.0f°'
                    % (_cn, math.degrees(brg),
                       float(self.cfg['ao_center_deg'])))
            elif _prim == 'STEER':
                self.get_logger().info(
                    'FOLLOWER ↻ 目标偏中线%+.0f° 超出 ±%.0f°, 边走边转'
                    % (math.degrees(brg),
                       float(self.cfg['ao_center_deg'])))
            elif _prevp in ('STEER', 'TURN'):
                self.get_logger().info(
                    'FOLLOWER ✓ 目标已压入 ±%.0f°(偏差%+.1f°), 转距离跟踪'
                    % (float(self.cfg['ao_center_deg']),
                       math.degrees(brg)))
        self._aligning = (_prim == 'TURN')
        if src == '扫描':
            self._aligning = False
"""
s = s.replace(OLD_A, NEW_A, 1)

# ================= B. 删掉 ALIGN 段 =================
OLD_B = """        if self._aligning:
            # ---- ALIGN: 原地纯旋转, 比例 + 软死区, 方向锁存防 ±180° 抖动 ----
            sgn = 1.0 if brg >= 0 else -1.0
            if aim > 2.1:
                if self._align_dir == 0.0:
                    self._align_dir = sgn
                sgn = self._align_dir
            elif aim < 1.05:
                self._align_dir = 0.0
            w_mag = self.cfg['recenter_kp'] * (aim - band * 0.5)
            w_mag = max(self.cfg['turn_min_w'],
                        min(self.cfg['recenter_rot_speed'], w_mag))
            w_t = sgn * w_mag
            # P22: 偏差不大(≤34°)时允许"边走边转"。纯原地旋转 v=0 会让逃跑的
            # 目标越拉越远, 实测 90s 内进出对正 44 次 —— 每次进对正就停车,
            # 目标跑开 → 又进对正, 形成"走走停停追不上"。大偏差(目标在左右/
            # 后相机)仍保持纯旋转, 因为那时前方是空的, 边走会撞。
            if aim < math.radians(34.0):
                # P36: 0.60 → 0.85。对正段占了追踪时间的一半以上,
                # 0.6 倍速度让机器人在"边转边走"时明显掉队。
                v_t = 0.85 * self.cfg['approach_speed']
            else:
                v_t = 0.0
            note = (f'对正{math.degrees(brg):+.0f}°'
                    f'{"(边转边走)" if v_t > 0 else ""}'
                    f'{"(方向锁定)" if self._align_dir != 0 else ""}')
            v_t, w_t, _sn = self._safety_gate(v_t, w_t, now, dist, brg)
            if _sn:
                note += ' ' + _sn
            self._publish_smooth(v_t, w_t, now)
            self._log(now, src, dist, pspd, rspd, w_t, brg, note, nscan,
                      rx, ry, px, py)
            return

"""
assert s.count(OLD_B) == 1, ('B', s.count(OLD_B))
NEW_B = """        # ALIGN 段已删除(P58): 转向由 4c/4d 的基元层统一处理, 走同一条
        # 出口(安全层 + 平滑 + 日志), 不再单独 return —— 原来它横在绕障
        # 之前 return, 导致"目标跑到障碍后面"时永远进不了绕障。

"""
s = s.replace(OLD_B, NEW_B, 1)

# ================= C. 插入路径跟踪 + 基元执行 =================
OLD_C = """        # --- 6. 避障层2: 真值地图硬保护 ---"""
assert s.count(OLD_C) == 1, ('C', s.count(OLD_C))

NEW_C = """        # --- 4c. 局部路径跟踪 (P58, 需求2/3) ---
        # 规划在后台线程里跑(单次 15~40ms), tick 只读它的结果, 保证 20Hz
        # 的控制周期不被规划拖成锯齿。
        _pl = self._planner
        if self.cfg['use_planner'] and src != '扫描':
            self._maybe_replan(now, rx, ry, px, py)
        _path_mode = False
        _dv = 1.0
        if (self.cfg['use_planner'] and src != '扫描'
                and len(_pl.pts) >= 2 and _pl.level >= 0.0):
            _dv = _pl.detour(rx, ry, px, py)
            _tight = _pl.level < float(self.cfg['obstacle_clear_m']) - 1e-6
            # 三个入口: 必须绕行 / 只能走降级(偏紧)通道 / 目标本身不可达
            if _dv > 1.12 or _tight or not _pl.reached:
                _path_mode = True
        if _path_mode:
            _lp = _pl.lookahead(rx, ry)
            if _lp is not None:
                _errh = wrap(math.atan2(_lp[1] - ry, _lp[0] - rx) - ryaw)
                w_t = max(-1.45, min(1.45, 2.0 * _errh))
                if v_t < -0.05:
                    pass                 # 人逼近需后退: 距离优先(需求5)
                elif abs(_errh) > math.radians(50.0):
                    v_t = 0.0            # 方向差太多 -> 先转, 别斜冲
                else:
                    v_t = max(0.12, min(float(self.cfg['plan_v_max']),
                                        max(v_t, 0.12)))
                note = ('规划跟踪 %d点 绕行比%.2f 级别%.2f 航差%+.0f°%s'
                        % (len(_pl.pts), _dv, _pl.level,
                           math.degrees(_errh),
                           '' if _pl.reached else ' 目标不可达→到附近'))
                if not self._plan_mode_on:
                    self.get_logger().info(
                        'FOLLOWER ➤ 局部路径规划: %d 个航点, 长度比直线 %.2f, '
                        '最小间隙 %.2fm%s'
                        % (len(_pl.pts), _dv, _pl.level,
                           '' if _pl.reached else '(目标不可达, 取附近点)'))
                self._plan_mode_on = True
        elif self._plan_mode_on:
            self._plan_mode_on = False
            self.get_logger().info('FOLLOWER ✓ 局部路径已绕过障碍, 恢复直追')

        # --- 4d. 视野基元执行 (P58, 需求1/2) ---
        # 路径跟踪优先于视野对正: 绕障时车头必须偏, 否则又被安全层急停。
        # 绕完立刻回 TRACK(下面这段), 目标重新压回 ±10°。
        if not _path_mode:
            _cn2 = {'front': '前', 'left': '左', 'right': '右',
                    'back': '后'}.get(self._fcam, self._fcam or '-')
            if _prim == 'TURN':
                v_t = 0.0
                w_t = max(-1.5, min(1.5, 2.2 * brg))
                if abs(w_t) < float(self.cfg['turn_min_w']):
                    w_t = (float(self.cfg['turn_min_w'])
                           if brg >= 0 else -float(self.cfg['turn_min_w']))
                note = '先转向%+.0f°(主相机[%s])' % (math.degrees(brg), _cn2)
            elif _prim == 'STEER':
                _span = max(1.0, math.degrees(_fov) - math.degrees(_center))
                _k = (math.degrees(_fov) - math.degrees(aim)) / _span
                _vc = float(self.cfg['ao_steer_vmax']) * max(0.0,
                                                             min(1.0, _k))
                if v_t > _vc:
                    v_t = _vc
                w_t = max(-1.4, min(1.4, 2.2 * brg))
                if abs(w_t) < float(self.cfg['turn_min_w']):
                    w_t = (float(self.cfg['turn_min_w'])
                           if brg >= 0 else -float(self.cfg['turn_min_w']))
                note = ('边走边转%+.0f° v<=%.2f'
                        % (math.degrees(brg), _vc))

        # --- 6. 避障层2: 真值地图硬保护 ---"""

s = s.replace(OLD_C, NEW_C, 1)

io.open(P, 'w', encoding='utf-8', newline='\n').write(s)
print('control ok')
