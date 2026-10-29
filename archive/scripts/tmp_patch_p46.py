#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P46 —— 补回绕障能力: 锁存式 Bug2 状态机。

问题: P39 因为死锁关掉航点绕障, P43b 因为打转关掉切线绕障, 结果**完全
没有横向机动能力**。目标走到障碍后面时:
  车头朝向目标(障碍) → 安全层 d_head<=stop_d 急停 → 原地转 → 转回目标
  → 再急停 …… 实测人站在 ob6 后面, 机器人 28s 只挪 0.3m, w 在
  +1.50/-1.15 之间每 0.5s 翻一次。

另外查出 P43 的切线绕障**从来没有真正生效**: 它把标记写进 `_dn`, 再
`note = _dn`, 于是进入 `if note:` 分支 —— 那段是**航点绕障**的代码,
会用 `_waypoint()` 的切点 tx,ty 覆盖刚设好的 v_t/w_t(w_lim=
recenter_rot_speed=1.5)。日志里的 w=+1.50 就是这么来的。

本补丁:
 A. 新增锁存式绕障的状态与配置
 B. `if note:` 加 use_waypoint 条件 —— 航点关闭时不得接管速度通道
 C. 用"状态机 + 锁存侧别 + 恒定偏转"替换切线绕障(不每 tick 重算)
 D. 绕障态下安全层只停车、不再另起一套 esc_dir 转向(避免两套反向抵消)
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


# ---- A: 配置 ----
sub("""    'use_tangent': False,""",
    """    'use_tangent': False,
    # P46: 锁存式绕障。P39/P43b 把两种绕障都关了, 结果目标走到障碍后
    # 就完全失去横向机动 —— 车头一朝向目标就被安全层急停, 原地打转。
    'use_avoid': True,
    'avoid_deg': 72.0,      # 绕行时车头相对"目标方向"的固定偏转角
    'avoid_min_v': 0.14,    # 最小推进速度(不允许为 0, 否则又变原地转)
    'avoid_trig_s': 0.45,   # 前方持续受阻多久才判定需要绕行
    'avoid_max_s': 9.0,     # 单次绕行最长时间(超时换边重试)
    'avoid_exit_v': 0.90,   # 前方余量超过 slow_d*该系数 视为畅通""",
    'A1 配置')

# ---- B: 状态 ----
sub("""        self._blk_t0 = 0.0            # 前方持续受阻的起始时刻(0=未受阻)""",
    """        self._blk_t0 = 0.0            # 前方持续受阻的起始时刻(0=未受阻)
        # P46: 锁存式绕障状态
        self._avoiding = False        # 是否处于绕行态
        self._avoid_side = 1.0        # +1 左绕 / -1 右绕(进入时锁死, 不重算)
        self._avoid_d0 = 0.0          # 进入绕行时的目标距离(Bug2 的基准)
        self._avoid_t0 = 0.0          # 进入绕行时刻
        self._blk2_t0 = 0.0           # 绕障触发计时(与脱困用的 _blk_t0 分开)""",
    'B1 init')

# ---- C: `if note:` 不得在航点关闭时接管 ----
sub("""        if _dn:            # P43: 切线绕障标记盖过航点的空标记
            note = _dn
        if note:
            err_yaw = wrap(math.atan2(ty - ry, tx - rx) - ryaw)""",
    """        if _dn:
            note = _dn
        # P46: 原判据只看 note 非空 —— 而 P43 的切线绕障会把标记写进
        # `_dn` 再转成 note, 于是"航点绕障"代码被激活, 用 `_waypoint()`
        # 的切点 tx,ty 覆盖刚设好的 v_t/w_t(且 w_lim=1.5)。所以切线绕障
        # 实际从未生效, 一直在跑航点逻辑。必须显式要求 use_waypoint。
        if self.cfg['use_waypoint'] and note:
            err_yaw = wrap(math.atan2(ty - ry, tx - rx) - ryaw)""",
    'C1 note 条件化')

# ---- D: 替换切线绕障 → 锁存式绕障 ----
OLD_TANGENT = '''        # --- 4. 切线绕障(P43) ---
        # P39 为了消除航点死锁把 use_waypoint 关了, 结果机器人彻底没有横向
        # 机动能力: 障碍一旦挡在人机连线中间, 安全层只会把速度限到爬行,
        # 实测整轮 gap 钉在 +0.42~0.51、人被拉到 5.52m。
        # 切线法: 前方受阻 → 目标方向偏转 ±62°, 方向锁存; 侧向余量决定速度。
        _dn = ''
        _pv8 = self.prox or {}
        _age8 = float(_pv8.get('age', 99.0) or 99.0)

        def _q(k):
            v = _pv8.get(k, -1.0)
            return v if (v and v > 0) else 99.0
        _df, _dfl, _dfr = _q('front'), _q('fl'), _q('fr')
        _dl, _dr = _q('left'), _q('right')
        _room_l = min(_dl, _dfl)
        _room_r = min(_dr, _dfr)
        _blocked = (_age8 < 0.5
                    and (_df < 1.05 or min(_dfl, _dfr) < 0.80)
                    and dist > 1.25)
        # P43b: 切线绕障实测会原地打转 —— 侧向余量不足时 v_t 被置 0,
        # 只剩 w=±1.2 旋转（日志: 绕障左 前1.20 指令v=+0.00 w=+1.1,
        # 人被拉到 4.3m）。启发式局部规划不可靠, 默认关闭,
        # 绕障仍需一个真正的局部规划器。保留开关便于后续验证。
        if self.cfg['use_tangent'] and _blocked:
            if self._det_dir == 0.0:
                self._det_dir = 1.0 if _room_l >= _room_r else -1.0
                self._det_t0 = now
            elif now - self._det_t0 > 0.8:
                _cur = _room_l if self._det_dir > 0 else _room_r
                _oth = _room_r if self._det_dir > 0 else _room_l
                if _oth > _cur + 0.35:      # 滞回: 另一侧要明显更空才换边
                    self._det_dir, self._det_t0 = -self._det_dir, now
            _side = _room_l if self._det_dir > 0 else _room_r
            _aim = wrap(brg + math.radians(62.0) * self._det_dir)
            w_t = max(-1.5, min(1.5, 2.0 * _aim))
            v_t = max(0.0, min(self.cfg['approach_speed'] * 0.75,
                               (_side - self.cfg['lidar_stop_m']) / 0.50))
            _dn = ('绕障%s 前%.2f 侧%.2f'
                   % ('左' if self._det_dir > 0 else '右', _df, _side))
        else:
            self._det_dir = 0.0'''

NEW_AVOID = '''        # --- 4. 绕障(P46): 锁存式 Bug2 ---
        # 历史: P39 关航点(切点在临界角左右翻 → 死锁), P43b 关切线(侧向
        # 余量不足时 v=0, 只剩旋转 → 原地打转)。两次都是"启发式不可靠",
        # 但**完全不绕**的代价更大: 目标走到障碍后面时, 车头一朝向目标
        # 就被安全层急停, 机器人原地抖(实测 28s 只挪 0.3m,
        # w 在 +1.50/-1.15 每 0.5s 翻一次)。
        # 这一版的区别: **不在每个 tick 重算切点**。
        #   ① 触发后锁存绕行侧别与"进入时的目标距离 d0";
        #   ② 绕行期间车头恒定偏转 avoid_deg 朝向侧方 —— 这样前向量到的
        #      是侧方余量, 不会因为"目标在障碍后"而持续急停;
        #   ③ 退出条件用 Bug2 判据: 前方畅通 **且** 比进入时更接近目标;
        #   ④ 有 avoid_max_s 超时换边, 不会永久卡在错误侧。
        _dn = ''
        _pv8 = self.prox or {}
        _age8 = float(_pv8.get('age', 99.0) or 99.0)

        def _q(k):
            v = _pv8.get(k, -1.0)
            return v if (v and v > 0) else 99.0
        _df, _dfl, _dfr = _q('front'), _q('fl'), _q('fr')
        _dl, _dr = _q('left'), _q('right')
        _room_l = min(_dl, _dfl)
        _room_r = min(_dr, _dfr)
        _stop = float(self.cfg['lidar_stop_m'])
        _avoid_now = False

        if self.cfg['use_avoid'] and _age8 < 0.5:
            _blk = (_df < _stop * 1.55
                    or min(_dfl, _dfr) < _stop * 1.10) and dist > 1.20
            if not self._avoiding:
                if _blk:
                    if self._blk2_t0 <= 0.0:
                        self._blk2_t0 = now
                    elif now - self._blk2_t0 > float(self.cfg['avoid_trig_s']):
                        self._avoiding = True
                        self._avoid_t0 = now
                        self._avoid_d0 = dist
                        self._avoid_side = 1.0 if _room_l >= _room_r else -1.0
                        self._blk2_t0 = 0.0
                        self.get_logger().info(
                            'FOLLOWER ⟳ 绕障介入(%s绕): 前%.2f 左前%.2f '
                            '右前%.2f 目标%.2fm → 车头偏转%.0f°'
                            % ('左' if self._avoid_side > 0 else '右',
                               _df, _dfl, _dfr, dist,
                               float(self.cfg['avoid_deg'])))
                else:
                    self._blk2_t0 = 0.0
            if self._avoiding:
                _clear = (_df > float(self.cfg['lidar_slow_m'])
                          * float(self.cfg['avoid_exit_v'])
                          and min(_dfl, _dfr) > 0.85)
                _tmo = now - self._avoid_t0 > float(self.cfg['avoid_max_s'])
                if (_clear and dist < self._avoid_d0 - 0.05) or _tmo:
                    self._avoiding = False
                    self._blk2_t0 = 0.0
                    self.get_logger().info(
                        'FOLLOWER ✓ 绕障退出(历时%.1fs, %s, 目标%.2fm) → 恢复指向'
                        % (now - self._avoid_t0,
                           '超时换边' if _tmo else '前方畅通', dist))
                else:
                    _avoid_now = True

        if _avoid_now:
            _side_room = _room_l if self._avoid_side > 0 else _room_r
            _aim = wrap(brg + math.radians(float(self.cfg['avoid_deg']))
                        * self._avoid_side)
            w_t = max(-1.7, min(1.7, 2.4 * _aim))
            _rm = min(_df, _side_room)
            # 侧方越窄走得越慢, 但不允许停下 —— 一停就退回"原地转"
            v_t = min(float(self.cfg['approach_speed']) * 0.72, 0.40,
                      max(float(self.cfg['avoid_min_v']),
                          (_rm - _stop + 0.12) / 0.45))
            _dn = ('绕障%s 前%.2f 侧%.2f 目标%.2f'
                   % ('左' if self._avoid_side > 0 else '右',
                      _df, _side_room, dist))
            note = _dn
        else:
            self._det_dir = 0.0'''

sub(OLD_TANGENT, NEW_AVOID, 'D1 锁存式绕障')

# ---- E: 安全层急停时不与绕障抢 w ----
sub('''            if d_head <= stop_d:
                v_t = 0.0
                # P27: 急停时的转向必须与上层意图协同。原来是"哪侧空就
                # 往哪转", 与上层"目标在右就右转"直接对打 —— 实测 w 在
                # -1.0 -> -1.4 -> +1.1 之间每帧翻号, 车在原地抖。
                room_l, room_r = d_fl, d_fr''',
    '''            if d_head <= stop_d:
                v_t = 0.0
                # P46: 绕障态的车头已经偏转 avoid_deg 朝向侧方, 这里的
                # d_head 量的是**侧方**余量。若再另起一套 esc_dir 转向,
                # 会与绕障的转向反向抵消 —— 实测 w 在 +1.50/-1.15 之间
                # 每 0.5s 翻一次, 机器人原地抖 28s 只挪 0.3m。
                if self._avoiding:
                    note = f'绕障中受阻 前{d_head:.2f}'
                # P27: 急停时的转向必须与上层意图协同。原来是"哪侧空就
                # 往哪转", 与上层"目标在右就右转"直接对打 —— 实测 w 在
                # -1.0 -> -1.4 -> +1.1 之间每帧翻号, 车在原地抖。
                room_l, room_r = d_fl, d_fr''',
    'E1 绕障不抢 w')

# 把 esc_dir 那段的其余部分也包进 else —— 用整体替换更稳
sub('''                if want == 0.0:                      # 直冲, 或意图侧被堵死
                    want = 1.0 if room_l >= room_r else -1.0
                if self._esc_dir == 0.0 or now - self._esc_t0 > 3.0:
                    self._esc_dir, self._esc_t0 = want, now
                elif want != self._esc_dir \\
                        and now - self._esc_t0 > self.cfg['esc_hold_s']:
                    # 滞回: 只有另一侧"明显更空(0.35m 以上)"才允许换边。
                    # 不设这条, d_fl/d_fr 会随自身转向互相追逐, 永远抖。
                    cur = room_l if self._esc_dir > 0 else room_r
                    oth = room_r if self._esc_dir > 0 else room_l
                    if oth > cur + 0.35 and oth > stop_d * 1.05:
                        self._esc_dir, self._esc_t0 = -self._esc_dir, now
                w_t = 1.15 * self._esc_dir
                note = (f'全向急停 前{d_f:.2f} 左前{d_fl:.2f}'
                        f' 右前{d_fr:.2f}'
                        f' 绕{"左" if self._esc_dir > 0 else "右"}')''',
    '''                if self._avoiding:
                    pass                          # 绕障自己管转向, 这里不插手
                else:
                    if want == 0.0:                  # 直冲, 或意图侧被堵死
                        want = 1.0 if room_l >= room_r else -1.0
                    if self._esc_dir == 0.0 or now - self._esc_t0 > 3.0:
                        self._esc_dir, self._esc_t0 = want, now
                    elif want != self._esc_dir \\
                            and now - self._esc_t0 > self.cfg['esc_hold_s']:
                        # 滞回: 只有另一侧"明显更空(0.35m 以上)"才允许换边。
                        # 不设这条, d_fl/d_fr 会随自身转向互相追逐, 永远抖。
                        cur = room_l if self._esc_dir > 0 else room_r
                        oth = room_r if self._esc_dir > 0 else room_l
                        if oth > cur + 0.35 and oth > stop_d * 1.05:
                            self._esc_dir, self._esc_t0 = -self._esc_dir, now
                    w_t = 1.15 * self._esc_dir
                    note = (f'全向急停 前{d_f:.2f} 左前{d_fl:.2f}'
                            f' 右前{d_fr:.2f}'
                            f' 绕{"左" if self._esc_dir > 0 else "右"}')''',
    'E2 esc_dir 收敛到 else')

# 上面 want= 的两行也要在 else 里 —— 原文顺序是 want 赋值在 if want 之前
sub('''                room_l, room_r = d_fl, d_fr
                want = 0.0
                if w_t > 0.05 and room_l > stop_d * 0.92:
                    want = 1.0                       # 想左转且左前有余量
                elif w_t < -0.05 and room_r > stop_d * 0.92:
                    want = -1.0                      # 想右转且右前有余量''',
    '''                room_l, room_r = d_fl, d_fr
                want = 0.0
                if not self._avoiding:
                    if w_t > 0.05 and room_l > stop_d * 0.92:
                        want = 1.0                    # 想左转且左前有余量
                    elif w_t < -0.05 and room_r > stop_d * 0.92:
                        want = -1.0                   # 想右转且右前有余量''',
    'E3 want 赋值收敛')

io.open(SRC, 'w', encoding='utf-8', newline='\n').write(s)
print('P46 应用完成 %d 处' % n)
import ast
ast.parse(io.open(SRC, encoding='utf-8').read())
print('语法 OK')
