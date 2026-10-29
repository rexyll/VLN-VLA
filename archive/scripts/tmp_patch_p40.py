#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P40: 距离环重写 —— 人速前馈 + 带内驻留同速 + 修正 cmd_gain

实测依据(eval8, 150s, P39 之后):
  in_band 21.8% / near 22.7% / far 55.6% / std 0.576 / 指令v均值 0.109

三个根因:
  (1) cmd_gain=1.62 是错的。日志里 93 组 `指令v / 实际v` 配对, 比值的主导
      众数恰好是 1.6(34/93), 过原点最小二乘斜率 1.066 —— 说明 Gazebo 差速
      插件本来就 1:1 跟踪速度指令, 之前用 /robot/odom 标定出的 "×0.62" 是
      odom 计量问题。放大 62% 直接造成冲过头再被拉开的大幅摆动。
  (2) 进入 ±0.1m 带内就 v_t=0 且积分清零。人一直在走, 一停车立刻掉出带,
      然后靠 0.6 的增益重新追 —— 而 kp=0.6 时稳态偏差 = 人速/kp = 0.5m。
      这就是 far 55.6% 的来源。
  (3) 缺前馈。误差环不该同时负责"提供跟随速度"和"修正距离残差"。

改法:
  - cmd_gain: 1.62 → 1.0
  - pid_kp_v: 0.6 → 1.9   (带内不再清零积分, 只做抗饱和)
  - 新增人速估计 _pvx/_pvy (视觉世界坐标差分, EWMA) → 沿车头投影做前馈
  - 带内: v_t = 前馈 (与人同速驻留), 不再置 0
"""
import io
import os
import sys

P = os.environ.get('VLA_SRC') or 'Z:/VLA/vla_ws/src/vla_follow/vla_follow/follower.py'

src = io.open(P, encoding='utf-8').read()
orig = src
applied = []


def rep(old, new, tag, count=1):
    global src
    if new in src:
        applied.append(tag + '(已存在,跳过)')
        return
    n = src.count(old)
    if n != count:
        print('!! [%s] 匹配 %d 次(期望 %d)\n---\n%s\n---' % (tag, n, count, old[:400]))
        sys.exit(1)
    src = src.replace(old, new, count)
    applied.append(tag)


# ---------------- P1: 配置 ----------------
rep(
    """    'approach_speed': 0.52,         # P35: 人速上限 0.55, 需留跟踪余量
    'cmd_gain': 1.62,               # P36: 指令速度→实际速度的补偿(实测×0.62)""",
    """    'approach_speed': 0.62,         # P40: 0.52→0.62, 给人突然加速留追赶余量
    # P40: 1.62 → 1.0。P36 那个 1.62 是错的 —— 日志里 93 组
    # `指令v / 实际v` 配对的比值众数恰好是 1.6, 过原点拟合斜率 1.066,
    # 说明差速插件本来就 1:1 跟踪速度指令; 当初用 /robot/odom 标出来的
    # "×0.62" 是 odom 计量造成的假象。多放大 62% 的结果是机器人冲过头、
    # 再被安全层拉回, 距离在 0.41~2.78m 之间大幅摆动。
    'cmd_gain': 1.00,
    # P40: 距离环前馈使能 —— 带内靠前馈与人同速, 误差只修正残差
    'vel_ff_gain': 0.90,            # 人速前馈系数(0=关)
    'vel_ff_max': 0.72,             # 前馈限幅 m/s""",
    'P1 cfg',
)

rep(
    """    'pid_kp_v': 0.6, 'pid_ki_v': 0.20, 'pid_kd_v': 0.08,""",
    """    'pid_kp_v': 1.90, 'pid_ki_v': 0.12, 'pid_kd_v': 0.04,   # P40""",
    'P2 pid',
)

# ---------------- P3: 人速估计状态 ----------------
rep(
    """        self._vis_prev = None
        self.vis_pspd = 0.0
        self._src_prev = ''""",
    """        self._vis_prev = None
        self.vis_pspd = 0.0
        # P40: 人速估计(视觉世界坐标差分) —— 距离环前馈用。
        # 纯误差反馈的距离环有个固有缺陷: 稳态误差 = 人速/kp。人走 0.30m/s、
        # kp=0.6 时稳态偏差就有 0.5m, 机器人永远"慢半拍" —— 实测 150s 里
        # 55.6% 的时间被拉开到 1.1m 以外。加前馈后误差只需修正残差。
        self._pv_t = None
        self._pv_x = 0.0
        self._pv_y = 0.0
        self._pvx = 0.0
        self._pvy = 0.0
        self._src_prev = ''""",
    'P3 state',
)

# ---------------- P4: 人速估计(视觉分支内) ----------------
rep(
    """            px = rx + math.cos(ryaw + bearing) * vdist
            py = ry + math.sin(ryaw + bearing) * vdist
            self._last_vis_xy = (px, py)      # 只由视觉刷新""",
    """            px = rx + math.cos(ryaw + bearing) * vdist
            py = ry + math.sin(ryaw + bearing) * vdist
            self._last_vis_xy = (px, py)      # 只由视觉刷新
            # P40: 由视觉世界坐标差分估计人速(前馈源)。
            # 注意两个坑: ① 跨相机切换时 bearing 会跳变, px/py 瞬间跳一下,
            # 差分出来是几十 m/s 的假峰, 必须按 |inst| 上限剔除;
            # ② 视觉 10Hz, 但 bearing 量化到 ~1°, 1m 处相当 1.7cm 抖动,
            # 所以用 EWMA(0.35) 压, 且 dt 太短/太长都不采样。
            _v_inst = None
            if self._pv_t is not None:
                dtp = now - self._pv_t
                if 0.09 < dtp < 0.60:
                    _v_inst = math.hypot(px - self._pv_x,
                                         py - self._pv_y) / dtp
                    if _v_inst <= 2.0:        # 剔除切换跳变造成的假峰
                        a = 0.35
                        self._pvx = (1 - a) * self._pvx + \\
                            a * (px - self._pv_x) / dtp
                        self._pvy = (1 - a) * self._pvy + \\
                            a * (py - self._pv_y) / dtp
            if _v_inst is None or _v_inst > 2.0 or (now - self._pv_t) > 0.60:
                self._pv_t, self._pv_x, self._pv_y = now, px, py
            else:
                self._pv_t, self._pv_x, self._pv_y = now, px, py""",
    'P4 pvel',
)

# ---------------- P5: 距离环重写 ----------------
rep(
    """        if abs(err_d) <= tol:
            v_t = 0.0
            self._int_d = 0.0
            self._prev_err_d = err_d
        else:
            self._int_d = max(-1.5, min(1.5,
                               self._int_d + self.cfg['pid_ki_v'] * err_d * dtc))
            der_d = (err_d - self._prev_err_d) / dtc
            self._prev_err_d = err_d
            v_pid = (self.cfg['pid_kp_v'] * err_d + self._int_d
                     + self.cfg['pid_kd_v'] * der_d)""",
    """        # P40: 人速前馈 —— 把人沿车头方向的速度分量直接叠到速度指令上。
        # 这样"跟着走"由前馈提供, "距离对不对"由误差环修正, 两者解耦;
        # 否则 kp 小则稳态差 0.5m(kp=0.6 时的实测), kp 大则过冲抖。
        _vfwd = self._pvx * math.cos(ryaw) + self._pvy * math.sin(ryaw)
        _vg = float(self.cfg['vel_ff_gain'])
        _vmax = float(self.cfg['vel_ff_max'])
        v_ff = max(-0.30, min(_vmax, _vg * _vfwd))
        if lost_dt > 0.6:                     # 视觉陈旧 → 前馈立刻撤掉
            v_ff = 0.0
        if abs(err_d) <= tol:
            # P40: 带内**不能停**。人只要还在走, 一停就掉出带。
            # 原来这里 v_t=0.0 且把积分清零, 于是永远在"停-追-停"之间摆动。
            v_t = max(-0.25, min(v_cap, v_ff))
            self._prev_err_d = err_d
            self._int_d *= 0.995              # 带内缓慢泄放, 不硬清零
        else:
            self._int_d = max(-1.2, min(1.2,
                               self._int_d + self.cfg['pid_ki_v'] * err_d * dtc))
            der_d = (err_d - self._prev_err_d) / dtc
            self._prev_err_d = err_d
            v_pid = (self.cfg['pid_kp_v'] * err_d + self._int_d
                     + self.cfg['pid_kd_v'] * der_d + v_ff)""",
    'P5 dist loop',
)

assert src != orig
io.open(P, 'w', encoding='utf-8', newline='\n').write(src)
print('OK 写入 %s (%d → %d 字节)' % (P, len(orig), len(src)))
for a in applied:
    print('  - ' + a)
