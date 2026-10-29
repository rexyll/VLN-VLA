#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P60b —— 控制层鲁棒性三条保险(一次打完 part1+part2)。

背景(P58 轮灾难的完整根因链已在 P60a 物理修复中解决):
  P60a 已修好"差速机器人俯仰失稳"(前进 3.8s 翘头 31° -> 深度相机看地
  -> 8 扇区同时读 0.33m -> 安全层判"四面都是障碍" -> 永久急停 + 原地
  打转 -> 视觉丢失)。P60b 是**兜底**: 万一车体再被外力碰歪、或感知读数
  再次异常, 绝不允许再出现"无限打转"。

1) 倾斜检测 + 自恢复
   车体 pitch/roll > 12° 时深度相机必然盯着地面/天花板, 所有距离读数
   都不可信。此时不是"急停 + 原地转向", 而是"暂停跟随 + 反向小步放平"
   (翘头→倒车压平, 低头→前进压平), 最多试 4 次。

2) 深度异常识别
   12x12 场地只有 7 个障碍, "8 个扇区距离全部 < 0.75m" 在几何上不可能
   发生 —— 只可能是相机在看地面/天花板。判为感知异常 -> 直接停车等待,
   而不是当成障碍急停(急停会转, 转完还是急停 -> 死循环)。

3) 急停看门狗
   连续处于"全向急停"且位置毫无进展时, 原地转向救不了(转多少度还是
   被挡)。改为后退让位, 避免"卡住 -> 打转 -> 还是卡住"。
"""
import io
import sys

P = '_p58_follower.py'
s = io.open(P, encoding='utf-8').read()
n = 0


def sub(old, new, tag):
    global s, n
    c = s.count(old)
    if c != 1:
        print('!! 锚点[%s] 命中 %d 次, 期望 1 次 —— 中止' % (tag, c))
        sys.exit(1)
    s = s.replace(old, new, 1)
    n += 1
    print('  ok  %s' % tag)


# ============ 1) 状态变量 ============
sub(
    "        self._align_dir = 0.0     # ±180° 附近的方向锁存",
    """        self._align_dir = 0.0     # ±180° 附近的方向锁存
        # --- P60b: 倾斜 / 急停看门狗 / 深度异常 三条兜底保险 ---
        self._tilt = 0.0          # 车体倾斜角(取 |pitch| |roll| 的较大者)
        self._tilt_up = False     # True=抬头(pitch>0, 前轮离地)
        self._tilt_try = 0        # 自恢复尝试次数(最多 4 次)
        self._tilt_t0 = 0.0       # 最近一次尝试的时刻
        self._tilt_dir = 0.0      # 自恢复方向: -1 倒车压平 / +1 前进压平
        self._esc_stuck_t0 = 0.0  # 连续急停起始时刻(0=未急停)
        self._esc_last_xy = None  # 急停期间最后的位置
        self._bad_depth = 0.0     # 深度异常起始时刻(0=正常)""",
    'init/state')

# ============ 2) 姿态回调: 算 pitch/roll ============
sub(
    """        if robot is None or person is None:
            return
        with self._lock:
            self.rx = robot.position.x""",
    """        # P60b: 倾斜角必须在"人不在场"时也能更新。
        # dynamic_pose/info 只播报**运动中**的实体 —— 人一停下, 消息里就
        # 只剩机器人甚至两个都没有。早期版本把倾斜计算放在 robot/person
        # 同时存在的分支之后, 结果"人停下 + 机器人翘头"这个最危险的
        # 组合恰恰测不出倾斜, 兜底保险形同虚设。
        if robot is not None:
            _q = robot.orientation
            _pr = math.asin(max(-1.0, min(
                1.0, 2.0 * (_q.w * _q.y - _q.z * _q.x))))
            _rl = math.atan2(2.0 * (_q.w * _q.x + _q.y * _q.z),
                             1.0 - 2.0 * (_q.x * _q.x + _q.y * _q.y))
            with self._lock:
                self._tilt = max(abs(math.degrees(_pr)),
                                 abs(math.degrees(_rl)))
                # pitch>0 = 抬头(车尾下沉、前轮离地)
                self._tilt_up = (math.degrees(_pr) > 0.0)
        if robot is None or person is None:
            return
        with self._lock:
            self.rx = robot.position.x""",
    'pose_cb/tilt')

# ============ 3) _tick: 倾斜接管(优先于一切控制逻辑) ============
sub(
    """        with self._lock:
            rx, ry, ryaw = self.rx, self.ry, self.ryaw
            px, py = self.px, self.py
            pspd = self.person_speed
            rspd = self.robot_speed
            nscan = self.scan_count
        if now - self._last_pose_mono > 1.5:""",
    """        with self._lock:
            rx, ry, ryaw = self.rx, self.ry, self.ryaw
            px, py = self.px, self.py
            pspd = self.person_speed
            rspd = self.robot_speed
            nscan = self.scan_count
            _tilt = self._tilt
        # --- P60b: 车体倾斜接管 ---
        # 优先级最高: 倾斜时相机看地/看天, 距离读数全部不可信, 再往下
        # 走任何控制分支(对正/绕行/急停)都只是原地打转。
        if _tilt > 12.0:
            if now - self._tilt_t0 > 1.0:
                if self._tilt_try == 0:
                    # 翘头(pitch>0)用倒车压平, 低头用前进压平
                    self._tilt_dir = -1.0 if self._tilt_up else 1.0
                    self.get_logger().warn(
                        'FOLLOWER ✗ 车体倾斜 %.1f° —— 暂停跟随并尝试放平(%s)'
                        % (_tilt, '倒车' if self._tilt_dir < 0 else '前进'))
                self._tilt_try += 1
                self._tilt_t0 = now
            if self._tilt_try <= 4:
                # 后退压平时先看后方净空, 别为了放平把车退进障碍
                _back_ok = True
                if self._tilt_dir < 0.0:
                    _pb = self.prox
                    _bb = min(_pb.get('back', -1.0), _pb.get('bl', -1.0),
                              _pb.get('br', -1.0))
                    _back_ok = (_bb <= 0.0 or _bb > 0.55)
                if _back_ok:
                    self._publish(self._tilt_dir * 0.22, 0.0)
                    self.v = self._tilt_dir * 0.22
                    if now - self._last_log > 0.5:
                        self._last_log = now
                        self.get_logger().warn(
                            'FOLLOWER ✗ 车体倾斜 %.1f°, 放平中(%d/4)'
                            % (_tilt, self._tilt_try))
                    return
                # 后方不安全 -> 原地不动, 绝不为了放平去撞障碍
                self._publish(0.0, 0.0)
                self.v = 0.0
                if now - self._last_log > 1.0:
                    self._last_log = now
                    self.get_logger().warn(
                        'FOLLOWER ✗ 车体倾斜 %.1f°, 但后方净空不足, '
                        '原地等待' % _tilt)
                return
            self._publish(0.0, 0.0)
            self.v = 0.0
            if now - self._last_log > 2.0:
                self._last_log = now
                self.get_logger().warn(
                    'FOLLOWER ✗ 车体倾斜 %.1f° 且无法自动放平, 已停车待处理'
                    % _tilt)
            return
        if self._tilt_try:            # 已恢复水平
            self._tilt_try = 0
            self._tilt_t0 = 0.0
            if now - self._last_log > 1.0:
                self._last_log = now
                self.get_logger().info('FOLLOWER ✓ 车体已恢复水平, 继续跟随')
        if now - self._last_pose_mono > 1.5:""",
    'tick/tilt_guard')

# ============ 4) 深度异常识别 ============
sub(
    """        d_f, d_fl, d_fr = _pv('front'), _pv('fl'), _pv('fr')
        d_l, d_r = _pv('left'), _pv('right')
        d_b = min(_pv('bl'), _pv('back'), _pv('br'))""",
    """        d_f, d_fl, d_fr = _pv('front'), _pv('fl'), _pv('fr')
        d_l, d_r = _pv('left'), _pv('right')
        d_b = min(_pv('bl'), _pv('back'), _pv('br'))
        # --- P60b: 深度异常识别 ---
        # 8 个扇区**同时**读到"全部 < 0.75m" 在 12x12 场地里几何上不可能
        # (只有 7 个障碍, 且彼此相距 2m 以上)。实测 P58 灾难现场就是
        # front=fl=fr=0.319 —— 三个扇区数值几乎相同, 那是相机在看地面/
        # 天花板, 不是障碍。此时若按"障碍"处理 -> 永久急停 + 原地转向
        # -> 还是急停 -> 死循环(就是用户说的"胡乱移动和打转")。
        # 所以判为感知异常 -> 停车等待, 并且不产生任何转向指令。
        _all8 = [d_f, d_fl, d_fr, d_l, d_r,
                 _pv('bl'), _pv('back'), _pv('br')]
        _mx8 = max(_all8)
        if _mx8 < 0.75 and self._tilt < 12.0:
            if self._bad_depth <= 0.0:
                self._bad_depth = now
            elif now - self._bad_depth > 0.8:
                return 0.0, 0.0, ('深度异常(8扇区均<%.2fm, 疑似相机看地/'
                                  '看天) 已停车' % _mx8)
        else:
            self._bad_depth = 0.0""",
    'safety/depth_anomaly')

# ============ 5) 急停看门狗 ============
sub(
    """                    w_t = 1.15 * self._esc_dir
                    note = (f'全向急停 前{d_f:.2f} 左前{d_fl:.2f}'
                            f' 右前{d_fr:.2f}'
                            f' 绕{"左" if self._esc_dir > 0 else "右"}')""",
    """                    w_t = 1.15 * self._esc_dir
                    note = (f'全向急停 前{d_f:.2f} 左前{d_fl:.2f}'
                            f' 右前{d_fr:.2f}'
                            f' 绕{"左" if self._esc_dir > 0 else "右"}')
                    # --- P60b: 急停看门狗 ---
                    # 原地转向救不了"前面被挡"(转多少度还是被挡), 只会
                    # 变成无限打转。连续急停 3s 且车几乎没挪动 -> 停止
                    # 打转, 改为后退让出空间, 再重新对正/重规划。
                    _xy = (self.rx, self.ry)
                    if self._esc_stuck_t0 <= 0.0:
                        self._esc_stuck_t0 = now
                        self._esc_last_xy = _xy
                    elif self._esc_last_xy is not None:
                        _mv = math.hypot(self.rx - self._esc_last_xy[0],
                                         self.ry - self._esc_last_xy[1])
                        if _mv >= 0.10:
                            # 有进展 -> 重新计时
                            self._esc_last_xy = _xy
                            self._esc_stuck_t0 = now
                        elif now - self._esc_stuck_t0 > 3.0:
                            if d_b > stop_d * 0.90:
                                # 后方有空间 -> 后退让位
                                v_t = -0.26
                                w_t = 0.0
                                note = (note + ' 卡死3s→后退让位').strip()
                                self._esc_stuck_t0 = now + 2.0   # 冷却 2s
                                self._esc_last_xy = _xy
                            else:
                                # 后方也堵 -> 换边继续试
                                self._esc_dir = -self._esc_dir
                                self._esc_stuck_t0 = now
                                self._esc_last_xy = _xy""",
    'safety/esc_watchdog')

# ============ 6) 脱离急停时复位看门狗 ============
sub(
    """            if d_head > stop_d:
                self._blk_t0 = 0.0                      # 已脱离受阻""",
    """            if d_head > stop_d:
                self._blk_t0 = 0.0                      # 已脱离受阻
                # P60b: 脱离急停 -> 复位看门狗, 否则下一次急停会被误判
                # 成"已经卡了很久", 直接后退让位(行为变得不可预期)
                self._esc_stuck_t0 = 0.0
                self._esc_last_xy = None""",
    'safety/esc_reset')

io.open(P, 'w', encoding='utf-8', newline='\n').write(s)
print('P60b ALL ok: %d 处替换' % n)
