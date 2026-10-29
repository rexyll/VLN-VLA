#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P58g: 修三个问题。

1) 目标在正后方时 bearing 跨 ±180° 翻号 -> `w = kp*brg` 每帧反向 ->
   机器人原地来回抖, 永远转不过去(实测 20s 内距离 3.2m 涨到 4.5m,
   w 在 -1.4/+1.5 之间反复)。原 ALIGN 段有 `_align_dir` 方向锁存,
   我重构时删掉了, 必须补回来 —— 但要做成通用 helper, 路径跟踪同样需要。
2) `_in_front` 原来按主相机名判断, 而 person_vision 有"保持上一主相机"
   的时间一致性策略: 目标明明已在车头 +6°(front 相机物理上覆盖 ±51.5°),
   只因上一帧主相机是 left 就被判成"不在 front 视野" -> 原地空转。
   判据应直接用 bearing 的绝对值, 它已经把相机偏置换算成车头系绝对方位。
3) `detour` 把"机器人到路径首点的偏移"也算进路径长度。路径首点是**规划
   时刻**的位置, 机器人一动这段就变长 -> 明明直达也报 1.4 倍 -> 误进
   路径跟踪模式。
"""
import io

P = '_p58_follower.py'
s = io.open(P, encoding='utf-8').read()

# ---- 1. LocalPlanner.detour: 排除过期路径 ----
OLD1 = """    def detour(self, rx, ry, gx, gy):
        \"\"\"剩余路径长度 / 直线距离。>1.12 说明必须绕行。\"\"\"
        d = math.hypot(gx - rx, gy - ry)
        if d < 1e-6:
            return 1.0
        return self.remaining(rx, ry) / d"""
assert s.count(OLD1) == 1
NEW1 = """    def detour(self, rx, ry, gx, gy):
        \"\"\"剩余路径长度 / 直线距离。>1.12 说明必须绕行。

        P58g: 路径过期(机器人被推到路径之外)时直接返回 1.0 —— 那种情况下
        `remaining` 会把"机器人→旧路径首点"的偏移也算进去, 明明直达也会
        报出 1.4 倍, 于是误进路径跟踪模式(实测原地转圈)。
        过期本来就由 need_replan 负责重建。
        \"\"\"
        d = math.hypot(gx - rx, gy - ry)
        if d < 1e-6:
            return 1.0
        if self._dist_to_path(rx, ry) > 0.35:
            return 1.0
        return self.remaining(rx, ry) / d"""
s = s.replace(OLD1, NEW1, 1)

# ---- 2. 通用转向 helper(带 ±180° 方向锁存) ----
OLD2 = "    # ---------- 局部规划(P58) ----------"
assert s.count(OLD2) == 1
NEW2 = """    def _turn_w(self, err, kp=2.2, wmax=1.5, wmin=0.35):
        \"\"\"按角度误差给角速度, 跨越 ±180° 时锁存转向方向。

        必须锁存的原因: 目标在正后方时, `bearing` 会随机器人微小转动在
        ±180° 两侧翻号(-150° ↔ +146°), 而 `kp*err` 在两侧给出相反的符号,
        于是 w 每帧反向 —— 机器人原地来回抖, 20s 都转不过 90°, 距离从
        3.2m 涨到 4.5m。锁存后一路朝同侧转, 转过 120° 才解锁。
        \"\"\"
        a = abs(err)
        if a > math.radians(120.0):
            if self._align_dir == 0.0:
                self._align_dir = 1.0 if err >= 0 else -1.0
            sgn = self._align_dir
        else:
            self._align_dir = 0.0
            sgn = 1.0 if err >= 0 else -1.0
        return sgn * max(wmin, min(wmax, kp * a))

    # ---------- 局部规划(P58) ----------"""
s = s.replace(OLD2, NEW2, 1)

# ---- 3. _in_front 用 bearing 而非主相机 ----
OLD3 = """        if src.startswith('视觉-'):
            _in_front = False            # 主相机是 left / right / back
        else:
            _in_front = aim <= _fov      # 视觉(front) / 搜索 / 丢失都按方位判"""
assert s.count(OLD3) == 1
NEW3 = """        # 判据只用 bearing 的绝对值: person_vision 有"保持上一主相机"的
        # 时间一致性策略, 目标可能已经在车头 +6° 而主相机还挂着 left ——
        # 按相机名判会把它当成"不在 front 视野", 白白原地空转。
        _in_front = aim <= _fov"""
s = s.replace(OLD3, NEW3, 1)

# ---- 4. TURN / PATH 改用 _turn_w ----
OLD4 = """                v_t = 0.0
                w_t = max(-1.5, min(1.5, 2.2 * brg))
                if abs(w_t) < float(self.cfg['turn_min_w']):
                    w_t = (float(self.cfg['turn_min_w'])
                           if brg >= 0 else -float(self.cfg['turn_min_w']))
                note = '先转向%+.0f°(主相机[%s])' % (math.degrees(brg), _cn2)"""
assert s.count(OLD4) == 1
NEW4 = """                v_t = 0.0
                w_t = self._turn_w(brg, 2.2, 1.5,
                                   float(self.cfg['turn_min_w']))
                note = '先转向%+.0f°(主相机[%s])' % (math.degrees(brg), _cn2)"""
s = s.replace(OLD4, NEW4, 1)

OLD5 = """                _errh = wrap(math.atan2(_lp[1] - ry, _lp[0] - rx) - ryaw)
                w_t = max(-1.45, min(1.45, 2.0 * _errh))"""
assert s.count(OLD5) == 1
NEW5 = """                _errh = wrap(math.atan2(_lp[1] - ry, _lp[0] - rx) - ryaw)
                w_t = self._turn_w(_errh, 2.0, 1.45,
                                   float(self.cfg['turn_min_w']))"""
s = s.replace(OLD5, NEW5, 1)

OLD6 = """                w_t = max(-1.4, min(1.4, 2.2 * brg))
                if abs(w_t) < float(self.cfg['turn_min_w']):
                    w_t = (float(self.cfg['turn_min_w'])
                           if brg >= 0 else -float(self.cfg['turn_min_w']))
                note = ('边走边转%+.0f° v<=%.2f'"""
assert s.count(OLD6) == 1
NEW6 = """                w_t = self._turn_w(brg, 2.2, 1.4,
                                   float(self.cfg['turn_min_w']))
                note = ('边走边转%+.0f° v<=%.2f'"""
s = s.replace(OLD6, NEW6, 1)

# ---- 5. 日志里的模式标签改用基元 ----
OLD7 = """            if self._aligning:
                mode = '对正'"""
assert s.count(OLD7) == 1
NEW7 = """            if self._plan_mode_on:
                mode = '规划'
            elif self._prim_prev == 'TURN':
                mode = '转向'
            elif self._prim_prev == 'STEER':
                mode = '边转边走'
            elif self._prim_prev == 'TRACK':
                mode = '跟踪'
            elif self._aligning:
                mode = '对正'"""
s = s.replace(OLD7, NEW7, 1)

io.open(P, 'w', encoding='utf-8', newline='\n').write(s)
print('g ok')
