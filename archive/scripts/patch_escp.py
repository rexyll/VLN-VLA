# -*- coding: utf-8 -*-
"""P82: 给 follower.py 打"贴身脱离(ESCP)"补丁。

文件是 CRLF 结尾, 全部锚点按字节匹配(内部用 \r\n)。
"""
import io
import sys

F = sys.argv[1]

with open(F, 'rb') as fh:
    data = fh.read()


def C(s):
    """普通字符串 -> CRLF 字节"""
    return s.replace('\n', '\r\n').encode('utf-8')


def sub(old, new, tag, cnt=1):
    global data
    o, n = C(old), C(new)
    k = data.count(o)
    if k != cnt:
        print('FAIL[%s]: 锚点命中 %d 次, 期望 %d' % (tag, k, cnt))
        print('--- 锚点 ---')
        print(old)
        sys.exit(1)
    data = data.replace(o, n, cnt)
    print('ok  %s' % tag)


# ---------------------------------------------------------------- A. 配置
A_OLD = """    'peek_kp': 2.2,                 # 绕视转向比例
}
"""
A_NEW = """    'peek_kp': 2.2,                 # 绕视转向比例
    # P82: 贴身脱离(ESCP) —— 人贴进近距盲区时的解锁动作。
    # 机理: 距离环的两处让距(back_hold / close_back)都以"后方净空"为前提,
    #   而人绕到机器人**侧后方**时, 后方 0.30m 那个障碍**就是人自己** ——
    #   判据永远不成立 → 永不后退; 同一时刻人已进入近距盲区(det=0),
    #   扫描又只会原地转。两者叠加成死锁。
    #   实测 p81b_1: 77.5s 起卡在 0.41m 一直到 120s 评测结束 —— 接触 589
    #   帧、near 33.2%、最长贴身 65.15s; 而同一时间**前方净空全程
    #   1.7~2.0m 空着没人用**。
    # 解锁: 不再试图"后退"(人挡着退不了), 而是转到背对人的可行航向
    #   **向前**脱离, 把距离重新拉回可见范围, 再交回常规跟踪。
    'escape_on': 1,                 # 0 = 关掉贴身脱离
    'escape_m': 0.60,               # 人距低于此值且退不动 → 脱离
    'escape_free_m': 0.90,          # 拉开到这么远就收工
    'escape_delay_s': 0.40,         # 过近需持续这么久才触发(抗测距抖动)
    'escape_dur_s': 5.0,            # 单次脱离最长时长
    'escape_v': 0.30,               # 脱离平移速度 m/s
    'escape_kp': 2.4,               # 脱离转向比例
    'escape_pass_m': 0.55,          # 脱离方向上要求的净空
}
"""
sub(A_OLD, A_NEW, 'A 配置')

# ---------------------------------------------------------------- B. 状态字段
B_OLD = """        self._peek_n = 0              # 累计绕视次数(diag)
"""
B_NEW = """        self._peek_n = 0              # 累计绕视次数(diag)
        # P82: 贴身脱离(ESCP)
        self._esc_t0 = 0.0            # 本轮脱离开始时刻
        self._esc_near_t = 0.0        # "人距过近"的起始时刻(滞回)
        self._esc_free_t = 0.0        # "已经拉开"的起始时刻(滞回)
        self._esc_n = 0               # 累计脱离次数(diag)
        self._esc_from = 'TRACK'      # 脱离前所在状态
"""
sub(B_OLD, B_NEW, 'B 状态字段')

# ------------------------------------------------- C. 视觉重捕获不打断 ESCP
C_OLD = """            if self._track != 'TRACK':
                gap = (now - self._lost_at) if self._lost else 0.0
"""
C_NEW = """            # P82: 贴身脱离期间**不**被"重新捕获"拉回 TRACK。
            # 否则每一拍都是 视觉→TRACK → 1.9 节→ESCP 的乒乓, 两处日志
            # 各刷一次、nav 也被反复 cancel。
            if self._track not in ('TRACK', 'ESCP'):
                gap = (now - self._lost_at) if self._lost else 0.0
"""
sub(C_OLD, C_NEW, 'C 视觉重捕获豁免')

# ---------------------------------------------------------------- D. 状态机
D_OLD = """        # --- 2. 搜索/扫描状态机 (P32; P65: GOTO 由 Nav2 执行) ---
        if self._track == 'GOTO':
"""
D_NEW = """        # --- 1.9 贴身脱离(ESCP) ---
        # 放在状态机最前面: 0.44m 已经是物理接触半径(裙摆0.26+车体0.18),
        # 这属于**安全**行为, 必须能抢占 GOTO/PEEK/SCAN/ALIGN。
        _pd = math.hypot(px - rx, py - ry)
        if self._track == 'ESCP':
            if _pd > float(self.cfg.get('escape_free_m', 0.90)):
                if self._esc_free_t <= 0.0:
                    self._esc_free_t = now
            else:
                self._esc_free_t = 0.0
            _ok = (self._esc_free_t > 0.0
                   and now - self._esc_free_t > 0.30)
            _to = (now - self._esc_t0) > float(self.cfg.get('escape_dur_s', 5.0))
            if _ok or _to:
                self.get_logger().info(
                    'FOLLOWER 贴身脱离结束(%.1fs, %s) 人距%.2fm → 交回常规处理'
                    % (now - self._esc_t0, '已拉开' if _ok else '超时', _pd))
                self._esc_free_t = 0.0
                self._esc_near_t = 0.0
                self._track = 'TRACK'
            else:
                src = '脱离'
        elif (int(self.cfg.get('escape_on', 1))
              and self._track != 'GOTO'
              and not self.cfg['allow_gt_track']
              and self._last_vis_xy is not None
              and _pd < float(self.cfg.get('escape_m', 0.60))):
            if self._esc_near_t <= 0.0:
                self._esc_near_t = now
            if (now - self._esc_near_t) > float(
                    self.cfg.get('escape_delay_s', 0.40)):
                # 只有"退不动"才需要 ESCP 接管: 后方净空够的话, 距离环的
                # back_hold 自己就把距离让开了, ESCP 抢过去只会多一次转向。
                _rb = self.prox.get('back', -1.0) if self.prox else -1.0
                if _rb is None or _rb < 0.0:
                    _rb = 9.9
                if _rb < float(self.cfg['lidar_stop_m']) + 0.25:
                    self._esc_t0 = now
                    self._esc_free_t = 0.0
                    self._esc_near_t = 0.0
                    self._esc_from = self._track
                    self._esc_n += 1
                    self._track = 'ESCP'
                    src = '脱离'
                    # 与绕视同理: 从 GOTO/SCAN 转入必须先停 Nav2, 否则两侧
                    # 同时下发 /cmd_vel。
                    self.pub_nav_active.publish(Bool(data=False))
                    self._nav_cancel('转入贴身脱离')
                    self.get_logger().warn(
                        'FOLLOWER ⚠ 贴身 %.2fm 且后方仅 %.2fm 退不动 → 脱离'
                        % (_pd, _rb))
        else:
            self._esc_near_t = 0.0

        # --- 2. 搜索/扫描状态机 (P32; P65: GOTO 由 Nav2 执行) ---
        if self._track == 'GOTO':
"""
sub(D_OLD, D_NEW, 'D 状态机')

# ---------------------------------------------------------------- E. 动作
E_OLD = """        if src == '绕视':
"""
E_NEW = """        if src == '脱离':
            # P82: 贴身脱离 —— 目标航向 = "背对人"的方向。
            # 差速底盘没有横移自由度, "远离人"只能拆成"先转到背对人的航向,
            # 再前进"。人绕到侧后方时这个航向大致是当前车头的反方向, 转
            # 180° 约 1.3s(@1.3rad/s) —— 比在原地空转 65s 强得多。
            _ex, _ey = rx - px, ry - py
            _nn = math.hypot(_ex, _ey) or 1e-6
            _hdg = wrap(math.atan2(_ey / _nn, _ex / _nn) - ryaw)
            # 期望方向未必走得通(背对人的一侧可能就是墙)。先在 ±105° 内
            # 找一个**真的能走**的航向, 优先靠近期望值 —— 与绕视同理。
            _pvE = self.prox or {}

            def _qE(kk):
                v = _pvE.get(kk, -1.0)
                return v if (v and v > 0) else 99.0
            _tabE = [(0.0, _qE('front')), (45.0, _qE('fl')),
                     (90.0, _qE('left')), (135.0, _qE('bl')),
                     (180.0, _qE('back')), (225.0, _qE('br')),
                     (270.0, _qE('right')), (315.0, _qE('fr')),
                     (360.0, _qE('front'))]

            def _dminE(b):
                dd = (math.degrees(b) + 360.0) % 360.0
                best = 99.0
                for i in range(len(_tabE) - 1):
                    a0, v0 = _tabE[i]
                    a1, v1 = _tabE[i + 1]
                    if a0 - 22.5 <= dd <= a1 + 22.5:
                        best = min(best, v0, v1)
                return best
            _passE = float(self.cfg.get('escape_pass_m', 0.55))
            for _off in (0, 15, -15, 30, -30, 45, -45, 60, -60, 75, -75,
                         90, -90, 105, -105):
                _cand = wrap(_hdg + math.radians(_off))
                if _dminE(_cand) >= _passE:
                    _hdg = _cand
                    break
            w_t = max(-1.3, min(1.3,
                                float(self.cfg.get('escape_kp', 2.4)) * _hdg))
            v_t = float(self.cfg.get('escape_v', 0.30)) \\
                * max(0.0, math.cos(_hdg)) ** 2
            w_t = self._arb_w(w_t, 'escape', now)
            v_t, w_t, _sn = self._safety_gate(v_t, w_t, now)
            self._publish_smooth(v_t, w_t, now)
            self._log(now, src, dist, pspd, rspd, w_t, brg,
                      '贴身脱离 人%.2fm 航向%+.0f°' % (_pd, math.degrees(_hdg)),
                      nscan, rx, ry, px, py)
            return

        if src == '绕视':
"""
sub(E_OLD, E_NEW, 'E 脱离动作')

# ---------------------------------------------------------------- F. 仲裁优先级
F_OLD = """    _W_PRIO = {'detour': 4, 'avoid': 3, 'peek': 2.5, 'scan': 2, 'align': 1,
               'track': 0}
"""
F_NEW = """    _W_PRIO = {'detour': 4, 'avoid': 3, 'escape': 2.8, 'peek': 2.5,
               'scan': 2, 'align': 1, 'track': 0}
"""
sub(F_OLD, F_NEW, 'F 仲裁优先级')

# ---------------------------------------------------------------- G. 盲走豁免
G_OLD = """                and self._track != 'PEEK':
"""
G_NEW = """                and self._track != 'PEEK' and self._track != 'ESCP':
"""
sub(G_OLD, G_NEW, 'G 盲走豁免')

# ---------------------------------------------------------------- H. diag
H_OLD = """        if self._track == 'PEEK':
            mode = 'PEEK'          # P81: 遮挡期绕视
"""
H_NEW = """        if self._track == 'ESCP':
            mode = 'ESCP'          # P82: 贴身脱离
        elif self._track == 'PEEK':
            mode = 'PEEK'          # P81: 遮挡期绕视
"""
sub(H_OLD, H_NEW, 'H diag')

with open(F, 'wb') as fh:
    fh.write(data)
print('written', F, len(data))
