#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P45 —— 修复"人在障碍物附近时丢目标, 随后胡乱移动/打转"。

根因(均为实测):
 1) `--- 6. 避障层2: 真值地图硬保护 ---` 用**外接圆**判距离、只看夹角
    不看横向净空、停车后**不给转向也不写 note**。机器人卡在
    (-2.60,-5.03) 时 ob6(-1.5,-4.5,1.0x0.5) 在前方 1.22m, 判据阈值
    0.56+0.56+0.15=1.27m → 判"挡路" → v=0; 而 TRACK 的 w 只在 |brg|>3°
    时才设, 对正后 w 恒 0 → 既不前进也不转, 静默死等到 20s 超时。
    实测该点实际间隙 0.73m, 完全能过。
 2) GOTO 目标点不做可达性检查。`_infer_target` 还会外推 1.5s, 目标点
    可能落进障碍里 —— 机器人永远到不了, 只能等超时。
 3) 到点阈值 0.5m 比安全层 stop_d(0.66) 还小, 目标点在障碍旁时数学上
    不可达。
 4) 20s 超时 + 12s 原地扫描 + 再来一轮(scan_rounds=2) = 最多 64s 的
    "原地打转 + 胡乱移动"。
 5) 人在障碍旁的深度取窗会混入障碍, 距离偏小 0.45~0.68m(实测)。
"""
import io
import os
import sys

SRC = os.environ.get('VLA_SRC', '/home/earl/workspace/VLA/vla_ws/src/'
                     'vla_follow/vla_follow/follower.py')
s = io.open(SRC, encoding='utf-8').read()
n_ok = 0


def sub(old, new, tag, count=1):
    global s, n_ok
    if s.count(old) != count:
        print('!! %s 锚点不唯一/未找到 (count=%d)' % (tag, s.count(old)))
        sys.exit(2)
    s = s.replace(old, new, count)
    n_ok += 1
    print('   ok %s' % tag)


# ============ A1: 真实几何表 + 点到障碍表面距离 ============
sub('''INFLATE = ROBOT_R + 0.38  # 绕障膨胀余量''',
    '''INFLATE = ROBOT_R + 0.38  # 绕障膨胀余量
# P45: 真实几何表(与 work_area.sdf 的 ob1..ob7 完全一致)。
# OBSTACLES 存的是**外接圆**半径, 对外接圆做膨胀会把方块的四个角
# 吃掉一大块"实际可通行"的空间 —— ob6 是 1.0x0.5, 外接半径 0.56,
# 判据 0.56+0.56+0.15=1.27m, 而机器人到它的真实表面还有 0.73m。
# 真值兜底必须用 OBB 真实距离, 否则"明明能过却停车"。
# (kind, x, y, a, b, yaw)  box: a/b = 半长/半宽; cyl: a = 半径
GEO = [
    ('box', 3.5, 2.0, 0.40, 0.40, 0.0),      # ob1 0.8x0.8
    ('box', -3.0, 3.2, 0.60, 0.30, 0.5),     # ob2 1.2x0.6 yaw0.5
    ('cyl', 0.5, -3.2, 0.40, 0.0, 0.0),      # ob3 r0.40
    ('cyl', -4.2, -2.5, 0.50, 0.0, 0.0),     # ob4 r0.50
    ('box', 4.2, -3.0, 0.30, 0.30, 0.0),     # ob5 0.6x0.6
    ('box', -1.5, -4.5, 0.50, 0.25, -0.4),   # ob6 1.0x0.5 yaw-0.4
    ('cyl', 2.2, 4.6, 0.35, 0.0, 0.0),       # ob7 r0.35
]


def geo_gap(x, y, g):
    """点到障碍**表面**的距离(欧氏, 负=在内部)。box 按 OBB 解算。"""
    k, ox, oy, a, b, ya = g
    dx, dy = x - ox, y - oy
    if ya:
        c, sn = math.cos(-ya), math.sin(-ya)
        dx, dy = dx * c - dy * sn, dx * sn + dy * c
    if k == 'cyl':
        return math.hypot(dx, dy) - a
    qx, qy = abs(dx) - a, abs(dy) - b
    return math.hypot(max(qx, 0.0), max(qy, 0.0))''', 'A1 GEO 表')
F = 'A1'

# ============ A2: 新增配置项 ============
sub("""    'goto_timeout_s': 20.0,         # 前往最后位置的最长时间""",
    """    'goto_timeout_s': 12.0,         # P45: 20→12, 20s 静默死等纯浪费
    'goto_reach_m': 0.75,           # P45: 到点阈值, 原硬编码 0.5 比安全层
                                    # stop_d(0.66) 还小, 障碍旁数学上不可达
    'goto_stall_s': 3.0,            # P45: 3s 内靠近不足 8cm 即判定"过不去"
    'scan_dur_s2': 7.0,             # P45: 扫描一圈约 5.2s, 12s 是 2.3 圈""",
    'A2 配置')

# ============ A3: __init__ 状态 ============
sub("""        self._scan_round = 0          # "推测→前往→扫描"已完成轮次""",
    """        self._scan_round = 0          # "推测→前往→扫描"已完成轮次
        # P45: GOTO 进度监视 —— 目标点被障碍卡住时提前退出, 别死等超时
        self._goto_best = None        # 历史最近目标点距离
        self._goto_prog_t = 0.0       # 最后一次"确实靠近了"的时刻""",
    'A3 init')

# ============ A4: 重写"真值地图硬保护" ============
sub('''        # --- 6. 避障层2: 真值地图硬保护 ---
        if v_t > 0:
            look = 0.35 + 0.5 * abs(v_t)
            nx = rx + math.cos(ryaw) * look
            ny = ry + math.sin(ryaw) * look
            if abs(nx) > WALL_HARD or abs(ny) > WALL_HARD:
                v_t = 0.0
            else:
                for (ox, oy, r) in OBSTACLES:
                    dx, dy = ox - rx, oy - ry
                    d = math.hypot(dx, dy)
                    if d < r + INFLATE + 0.15 and \\
                            (dx * math.cos(ryaw) + dy * math.sin(ryaw)) / \\
                            max(d, 1e-6) > 0.7:
                        v_t = 0.0
                        break''',
    '''        # --- 6. 避障层2: 真值地图硬保护 ---
        # P45: 重写。原实现三个缺陷叠加成**静默死锁**:
        #  ① 距离用**外接圆**: ob6(1.0x0.5, 外接半径 0.56) 的判据阈值
        #     0.56+0.56+0.15=1.27m, 而机器人到它真实表面还有 0.73m ——
        #     明明能从旁边过去, 却被判"正前方有障碍"。
        #  ② 只看夹角(cos>0.7≈±45°)不看**横向净空**, 障碍偏在侧前方 26°、
        #     侧向有 0.53m 余量时仍算"挡路"。
        #  ③ 停车后**不给转向、不写 note**。而 TRACK 的 w 只在 |brg|>3°
        #     时才设, 对正后 w 恒为 0 → 既不前进也不转, 静默死等到
        #     goto_timeout_s(20s)。日志实证: 卡在(-2.60,-5.03), 目标
        #     (-0.43,-4.95) 正前方 2.18m, 指令v=+0.00 w=+0.0 连续 20s,
        #     随后 12s 原地扫描 + 再来一轮 → 人丢失 38.3s。
        if v_t > 0:
            look = 0.35 + 0.5 * abs(v_t)
            nx = rx + math.cos(ryaw) * look
            ny = ry + math.sin(ryaw) * look
            # 墙体: 只有"已在硬边界外、且这一脚还在继续往外"才拦。
            # 原判据 abs(nx)>WALL_HARD 连**贴墙平行行驶**都会永久禁行
            # (y 分量不变, 仅仅因为 |ny| 超一点点)。
            if (abs(rx) > WALL_HARD and abs(nx) > abs(rx) + 1e-6) or \\
                    (abs(ry) > WALL_HARD and abs(ny) > abs(ry) + 1e-6):
                v_t = 0.0
                note = (note + ' 墙前硬停').strip()
            else:
                _g_hit = None
                _g_lat = 0.0
                for _g in GEO:
                    # 用"向前 look 之后"的位置算真实间隙, 留 0.10m 余量
                    if geo_gap(nx, ny, _g) > ROBOT_R + 0.10:
                        continue
                    _fwd = ((_g[1] - rx) * math.cos(ryaw)
                            + (_g[2] - ry) * math.sin(ryaw))
                    if _fwd < -0.05:
                        continue                  # 已经在身后
                    _lat = (-(_g[1] - rx) * math.sin(ryaw)
                            + (_g[2] - ry) * math.cos(ryaw))
                    _ext = _g[3] if _g[0] == 'cyl' else math.hypot(_g[3], _g[4])
                    if abs(_lat) > _ext + ROBOT_R:
                        continue                  # 侧向净空够, 能擦过去
                    _g_hit, _g_lat = _g, _lat
                    break
                if _g_hit is not None:
                    v_t = 0.0
                    # P45 关键: 必须给转向。停车 + w=0 = 永久死锁。
                    w_t = -1.10 if _g_lat > 0 else 1.10
                    note = (note + ' 真值硬保(%+.1f,%+.1f)%s'
                            % (_g_hit[1], _g_hit[2],
                               '右绕' if _g_lat > 0 else '左绕')).strip()''',
    'A4 真值硬保护')
F = 'A4'

# ============ A5: GOTO 停滞检测 + 放宽到点判定 ============
sub("""        if self._track == 'GOTO':
            if now - self._search_t0 > self.cfg['goto_timeout_s']:
                self.get_logger().warn(
                    'FOLLOWER 前往最后位置超时(%.0fs), 转原地扫描'
                    % self.cfg['goto_timeout_s'])
                self._enter_scan(now, announce=False)
            else:
                gx, gy = self._search_target
                if math.hypot(gx - rx, gy - ry) < 0.5:
                    self.get_logger().info(
                        'FOLLOWER 已到达人最后出现位置(%+.1f,%+.1f), '
                        '原地 360° 扫描确认' % (gx, gy))
                    self._enter_scan(now, announce=False)
                else:
                    px, py = gx, gy
                    brg = wrap(math.atan2(gy - ry, gx - rx) - ryaw)
                    src = '搜索'""",
    """        if self._track == 'GOTO':
            gx, gy = self._search_target
            _gd = math.hypot(gx - rx, gy - ry)
            # P45: 到点阈值原硬编码 0.5m, 比安全层 stop_d(0.66) 还小 ——
            # 目标点在障碍旁边时数学上永远不可达, 只能等 20s 超时。
            if _gd < float(self.cfg['goto_reach_m']):
                self.get_logger().info(
                    'FOLLOWER 已到达人最后出现位置(%+.1f,%+.1f) 距离%.2fm, '
                    '原地 360° 扫描确认' % (gx, gy, _gd))
                self._enter_scan(now, announce=False)
            elif now - self._search_t0 > self.cfg['goto_timeout_s']:
                self.get_logger().warn(
                    'FOLLOWER 前往最后位置超时(%.0fs, 还差%.2fm), 转原地扫描'
                    % (self.cfg['goto_timeout_s'], _gd))
                self._enter_scan(now, announce=False)
            elif (self._goto_best is not None
                  and now - self._search_t0 > 2.0
                  and now - self._goto_prog_t
                  > float(self.cfg['goto_stall_s'])):
                # P45: 3s 内没能再靠近 8cm → 目标点被障碍/墙挡住, 过不去。
                # 与其静默死等 12s, 不如立刻转扫描 —— 人在动, 原地转一圈
                # 看到人的概率远高于对着障碍硬推。
                self.get_logger().warn(
                    'FOLLOWER 前往最后位置受阻(%.1fs 无进展, 还差%.2fm), '
                    '就地转扫描' % (now - self._search_t0, _gd))
                self._enter_scan(now, announce=False)
            else:
                if self._goto_best is None or _gd < self._goto_best - 0.08:
                    self._goto_best = _gd
                    self._goto_prog_t = now
                px, py = gx, gy
                brg = wrap(math.atan2(gy - ry, gx - rx) - ryaw)
                src = '搜索'""",
    'A5 GOTO 停滞检测')
F = 'A5'

# ============ A6: _free_target 可达性修正 ============
sub('''    def _enter_scan(self, now, announce=False):''',
    '''    def _free_target(self, tx, ty, margin=0.22):
        """P45: 把目标点从障碍膨胀区里推出来。

        GOTO 的目标点来自"最后 10 张快照 + 1.5s 外推", 很容易落在障碍
        里 —— 机器人对着障碍推到超时也到不了。这里沿"障碍中心→目标点"
        方向把点推到表面外 0.22m(车体 0.18 + 余量), 最多迭代 6 轮处理
        多个障碍互相重叠的情况。
        """
        for _ in range(6):
            moved = False
            for g in GEO:
                gap = geo_gap(tx, ty, g)
                if gap >= ROBOT_R + margin:
                    continue
                vx, vy = tx - g[1], ty - g[2]
                nn = math.hypot(vx, vy)
                if nn < 1e-6:
                    vx, vy, nn = 1.0, 0.0, 1.0
                push = (ROBOT_R + margin) - gap + 0.02
                tx += vx / nn * push
                ty += vy / nn * push
                moved = True
            if not moved:
                break
        # 别推到墙外: 用 WALL_SOFT(5.15) 而不是 5.3, 给贴墙修正留空间
        tx = max(-WALL_SOFT, min(WALL_SOFT, tx))
        ty = max(-WALL_SOFT, min(WALL_SOFT, ty))
        return tx, ty

    def _enter_scan(self, now, announce=False):''',
    'A6 _free_target')

# ============ A7: _infer_target 结果过可达性 + GOTO 建立时重置进度 ============
sub('''        tx = max(-5.3, min(5.3, tx))
        ty = max(-5.3, min(5.3, ty))
        seq = [h.get('cam', '?') for h in win]''',
    '''        tx = max(-5.3, min(5.3, tx))
        ty = max(-5.3, min(5.3, ty))
        # P45: 外推点常常落进障碍里 → 机器人永远到不了。先推出来。
        tx, ty = self._free_target(tx, ty)
        seq = [h.get('cam', '?') for h in win]''',
    'A7 infer 可达性')

# 建立 GOTO 时重置进度监视(两处: 丢失分支 / 扫描分支)
sub('''                        self._search_target = tgt
                        self._search_t0 = now
                        self._track = 'GOTO\'''',
    '''                        self._search_target = tgt
                        self._search_t0 = now
                        self._goto_best = None
                        self._goto_prog_t = now
                        self._track = 'GOTO\'''',
    'A8 GOTO 进度重置')

sub('''                    self._search_target = tgt
                    self._search_t0 = now
                    self._track = 'GOTO\'''',
    '''                    self._search_target = tgt
                    self._search_t0 = now
                    self._goto_best = None
                    self._goto_prog_t = now
                    self._track = 'GOTO\'''',
    'A9 GOTO 进度重置2')

# ============ A10: 扫描时长缩短 ============
sub("""        self._scan_until = now + float(self.cfg['scan_dur_s'])""",
    """        # P45: 12s 是 2.3 圈(转速 1.2rad/s 时一圈 5.2s), 纯浪费
        self._scan_until = now + float(self.cfg.get(
            'scan_dur_s2', self.cfg['scan_dur_s']))""",
    'A10 扫描时长')

io.open(SRC, 'w', encoding='utf-8', newline='\n').write(s)
print('P45 应用完成: %d 处' % n_ok)

import ast
ast.parse(io.open(SRC, encoding='utf-8').read())
print('语法 OK')
