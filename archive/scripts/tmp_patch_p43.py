#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P43: (a) 前馈+有界反馈  (b) 切线绕障回归

(a) 积分权限过大
  eval10 日志: 带内(0.9~1.15m)指令 -0.16 ~ +0.53, 极差 0.69, 其中 21/69 个点
  > 0.30 m/s。前馈 v_ff 已经负责人速, 但积分项钳在 ±0.9 —— 它单独就能下一脚
  0.9m/s 的油门。改成"前馈 + 有界反馈": 反馈总幅度钳 ±0.45, 积分钳 ±0.15,
  ki 0.12→0.06。稳态靠前馈, 反馈只修残差。

(b) P39 把航点绕障彻底关掉之后, 机器人丧失了"绕"的能力
  eval10: gap 连续 60s+ 钉在 +0.42~0.51, 指令v 掉到 0, 人被拉到 5.52m,
  far_pct 62.4%。根因是安全层只会"限速到爬行", 没有横向机动。
  这里用最稳的切线法(bug 算法): 前方受阻时, 把目标方向整体偏转 ±62° 并
  **锁存方向(≥0.8s, 另一侧要明显更空 0.35m 以上才换边)**, 侧向余量决定速度。
  与航点的区别: 偏转角连续、方向锁存, 不存在"切点在临界角两侧跳变"的死锁
  —— P39 关掉航点是为了消除这个死锁, 切线法不会复现它。
"""
import ast
import io
import os
import sys

P = os.environ.get('VLA_SRC') or '_p43_follower.py'
src = io.open(P, encoding='utf-8').read()
done = []


def rep(old, new, tag):
    global src
    if new in src:
        done.append(tag + '(已存在)')
        return
    if src.count(old) != 1:
        print('!! [%s] 匹配 %d 次\n%s' % (tag, src.count(old), old[:300]))
        sys.exit(1)
    src = src.replace(old, new)
    done.append(tag)


# ---------- (a) 有界反馈 ----------
rep(
    """        self._int_d = max(-0.9, min(0.9,
                           self._int_d + self.cfg['pid_ki_v'] * err_d * dtc))
        der_d = (err_d - self._prev_err_d) / dtc
        self._prev_err_d = err_d
        v_pid = (self.cfg['pid_kp_v'] * err_d + self._int_d
                 + self.cfg['pid_kd_v'] * der_d + v_ff)
        v_t = max(-0.35, min(v_cap, v_pid))""",
    """        # P43: 反馈总幅度必须钳住。前馈已经负责人速, 反馈只修残差;
        # 原来积分钳 ±0.9, 它单独就能下一脚 0.9m/s 的油门 —— 实测带内
        # (0.9~1.15m)指令在 -0.16~+0.53 之间摆, 极差 0.69。
        self._int_d = max(-0.15, min(0.15,
                           self._int_d + self.cfg['pid_ki_v'] * err_d * dtc))
        der_d = (err_d - self._prev_err_d) / dtc
        self._prev_err_d = err_d
        _fb = (self.cfg['pid_kp_v'] * err_d + self._int_d
               + self.cfg['pid_kd_v'] * der_d)
        _fb = max(-0.45, min(0.45, _fb))
        v_pid = v_ff + _fb
        v_t = max(-0.35, min(v_cap, v_pid))""",
    'a-有界反馈',
)

# ---------- (a2) ki 减小 ----------
rep(
    """    'pid_kp_v': 1.90, 'pid_ki_v': 0.12, 'pid_kd_v': 0.04,   # P40""",
    """    'pid_kp_v': 1.90, 'pid_ki_v': 0.06, 'pid_kd_v': 0.04,   # P43""",
    'a2-ki',
)

# ---------- (b) 状态 ----------
rep(
    """        self._pv_t = None
        self._pv_x = 0.0""",
    """        # P43: 切线绕障状态(方向锁存, 防左右翻转死锁)
        self._det_dir = 0.0
        self._det_t0 = 0.0
        self._pv_t = None
        self._pv_x = 0.0""",
    'b-状态',
)

# ---------- (b) 切线绕障 ----------
rep(
    """        # --- 4. 绕障(仅对正后) ---
        tx, ty, note, dwp = self._waypoint(rx, ry, px, py)""",
    """        # --- 4. 切线绕障(P43) ---
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
        if _blocked:
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
            self._det_dir = 0.0
        # --- 4b. 航点绕障(默认关, 保留开关) ---
        tx, ty, note, dwp = self._waypoint(rx, ry, px, py)""",
    'b-切线绕障',
)

io.open(P, 'w', encoding='utf-8', newline='\n').write(src)
ast.parse(src)
print('P43 OK, 语法通过, %d 字符' % len(src))
for d in done:
    print('  - ' + d)
