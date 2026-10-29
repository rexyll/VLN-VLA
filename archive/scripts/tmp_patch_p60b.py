#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P60b: 控制层鲁棒性 —— 三条"不许无限打转"的保险。

1) 倾斜检测 + 自恢复
   物理上机器人已经修好(P60a), 但任何外力都可能把它碰歪。一旦车体
   俯仰/横滚 > 12°, 深度相机会盯着地面/天花板, 8 个扇区同时读到 0.1~0.5m
   -> 安全层判定"四面都是障碍" -> 永远急停 + 原地转向 -> 这正是用户看到的
   "胡乱移动和打转"。所以: 检测到倾斜就停车, 并用反向小步把车放平
   (翘头 -> 倒车压平; 低头 -> 前进压平), 最多试 3 次。

2) 急停看门狗
   连续 N 秒处于"全向急停"且位置没有进展 -> 不再原地转, 改为后退 1.2s
   再重新规划/重新对正。避免"卡住 -> 打转 -> 还是卡住"的死循环。

3) 深度异常识别
   12x12 的场地里只有 7 个障碍, 不可能出现"8 个扇区全部 < 0.5m"。
   出现这种读数只可能是相机在看地面/天花板 -> 判定感知异常, 直接停车
   等待, 而不是当成障碍急停。
"""
import io

P = '_p58_follower.py'
s = io.open(P, encoding='utf-8').read()

# ---------------- 1) 状态变量 ----------------
OLD = "        self._align_dir = 0.0     # ±180° 附近的方向锁存"
NEW = """        self._align_dir = 0.0     # ±180° 附近的方向锁存
        # P60b: 倾斜/急停看门狗/深度异常
        self._tilt = 0.0          # 当前车体倾斜角(取 pitch/roll 绝对值最大)
        self._tilt_try = 0        # 自恢复尝试次数
        self._tilt_t0 = 0.0       # 最近一次检测到倾斜的时刻
        self._tilt_dir = 0.0      # 自恢复方向: +1 前进压平 / -1 倒车压平
        self._esc_stuck_t0 = 0.0  # 连续急停起始时刻
        self._esc_last_xy = None  # 急停期间最后的位置
        self._bad_depth = 0.0     # 深度异常(8 扇区同时极小)持续时间"""
assert s.count(OLD) == 1
s = s.replace(OLD, NEW, 1)

# ---------------- 2) 姿态回调里算 pitch/roll ----------------
OLD = """        with self._lock:
            self.rx = robot.position.x
            self.ry = robot.position.y
            self.ryaw = yaw_of(robot.orientation)"""
NEW = """        with self._lock:
            self.rx = robot.position.x
            self.ry = robot.position.y
            self.ryaw = yaw_of(robot.orientation)
            # P60b: 车体倾斜角。机器人一旦被碰歪, 深度相机会盯着地面,
            # 8 个扇区同时读到 0.1~0.5m -> 安全层误判"四面都是障碍" ->
            # 永久急停 + 原地打转。先把倾斜量算出来供控制层用。
            _q = robot.orientation
            _pr = math.asin(max(-1.0, min(1.0, 2.0 * (_q.w * _q.y - _q.z * _q.x))))
            _rl = math.atan2(2.0 * (_q.w * _q.x + _q.y * _q.z),
                             1.0 - 2.0 * (_q.x * _q.x + _q.y * _q.y))
            self._tilt = max(abs(math.degrees(_pr)), abs(math.degrees(_rl)))"""
assert s.count(OLD) == 1
s = s.replace(OLD, NEW, 1)

# ---------------- 3) _tick 开头: 倾斜接管 ----------------
OLD = """        with self._lock:
            rx, ry, ryaw = self.rx, self.ry, self.ryaw
            px, py = self.px, self.py
            pspd = self.person_speed
            rspd = self.robot_speed
            nscan = self.scan_count"""
NEW = """        with self._lock:
            rx, ry, ryaw = self.rx, self.ry, self.ryaw
            px, py = self.px, self.py
            pspd = self.person_speed
            rspd = self.robot_speed
            nscan = self.scan_count
            _tilt = self._tilt
        # P60b: 车体倾斜 —— 优先于一切控制逻辑处理。倾斜时相机看地/看天,
        # 所有距离读数都不可信, 继续跑只会原地打转。
        if _tilt > 12.0:
            if now - self._tilt_t0 > 1.0:
                if self._tilt_try == 0:
                    # 翘头(pitch>0)用倒车压平, 低头用前进压平。
                    self._tilt_dir = -1.0 if self._tilt_up else 1.0
                    self.get_logger().warn(
                        'FOLLOWER ✗ 车体倾斜 %.1f° —— 暂停跟随并尝试放平(%s)'
                        % (_tilt, '倒车' if self._tilt_dir < 0 else '前进'))
                self._tilt_try += 1
                self._tilt_t0 = now
            if self._tilt_try <= 4:
                self._publish(self._tilt_dir * 0.22, 0.0)
                self.v = self._tilt_dir * 0.22
                if now - self._last_log > 0.5:
                    self._last_log = now
                    self.get_logger().warn(
                        'FOLLOWER ✗ 车体倾斜 %.1f°, 放平中(%d/4)'
                        % (_tilt, self._tilt_try))
                return
            self._publish(0.0, 0.0)
            self.v = 0.0
            if now - self._last_log > 2.0:
                self._last_log = now
                self.get_logger().warn(
                    'FOLLOWER ✗ 车体倾斜 %.1f° 且无法自动放平, 已停车待人工处理'
                    % _tilt)
            return
        if self._tilt_try:          # 已恢复水平
            self._tilt_try = 0
            self._tilt_t0 = 0.0
            if now - self._last_log > 1.0:
                self._last_log = now
                self.get_logger().info('FOLLOWER ✓ 车体已恢复水平, 继续跟随')"""
assert s.count(OLD) == 1
s = s.replace(OLD, NEW, 1)

io.open(P, 'w', encoding='utf-8', newline='\n').write(s)
print('P60b part1 ok')
