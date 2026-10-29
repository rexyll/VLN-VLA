#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把全向安全层从 _tick 中间移到"所有出口统一裁剪"。

原因: _tick 有 3 条出口 —— 原地扫描、对正(ALIGN)、距离跟踪。原来的安全层
只长在 TRACK 主路径上, 于是
  · 对正阶段"边走边转"(v=0.6*approach_speed) 完全裸奔
  · 防撞人 / 卡死脱困 会把 v 改成负数, 它们写在安全层之后, 又没看后方
实测 E 场景(斜 40° 冲 ob5) 的 -0.062m 擦碰就是这么来的。
"""
import shutil
import sys

P = '/home/earl/workspace/VLA/vla_ws/src/vla_follow/vla_follow/follower.py'
shutil.copy(P, P + '.bak_gate')
s = open(P, encoding='utf-8').read()

METHOD = '''    # ---------- 全向安全层 ----------
    def _safety_gate(self, v_t, w_t, now):
        """全向 8 方位安全层 —— 最高优先级的硬裁剪。

        P26。数据: /perception/proximity 的 8 个 45° 方位(4 台深度相机按 5
        扇区合成)。为什么不用 /scan: gpu_lidar 只支持 ogre2, 本项目为绕开
        VMware 白屏固定用 ogre1, /scan 的 360 条射线全是 range_min(0.12m);
        CPU 版 type="lidar" 又被 gz-sim8 拒绝("Sensor type LIDAR not
        supported yet")。深度相机是本环境唯一可靠的 360° 测距。

        关键: 必须在**所有**出口调用 —— 对正(ALIGN)、原地扫描、防撞人后退、
        卡死脱困都会直接设速度, 只要有一处绕过它, 就会在那里撞。

        返回 (v_t, w_t, note)
        """
        p8 = self.prox
        if p8.get('age', 99.0) >= self.cfg['lidar_max_age_s']:
            return v_t, w_t, ''

        def _pv(k):
            v = p8.get(k, -1.0)
            return v if v > 0 else 99.0

        def _lr(lo, hi):
            r = self._sector_min(lo, hi)
            return r if r > 0 else 99.0

        d_f, d_fl, d_fr = _pv('front'), _pv('fl'), _pv('fr')
        d_l, d_r = _pv('left'), _pv('right')
        d_b = min(_pv('bl'), _pv('back'), _pv('br'))
        # 雷达若在本环境真的可用(ogre2)则一并纳入, 取更保守值
        if self._lidar_ok and self.scan is not None \\
                and now - self.scan[3] < self.cfg['lidar_max_age_s']:
            d_f = min(d_f, _lr(-26.0, 26.0))
            d_fl = min(d_fl, _lr(26.0, 65.0))
            d_fr = min(d_fr, _lr(-65.0, -26.0))
        stop_d = self.cfg['lidar_stop_m']
        slow_d = self.cfg['lidar_slow_m']
        tau = self.cfg['lidar_tau_s']
        side_d = self.cfg['lidar_side_m']
        note = ''
        # 斜 45° 方向折算到前向要乘 cos45°≈0.72(原来加常数 0.10, 在 1.5m
        # 量级上等于不折算, 斜前方的立柱一直被当成"前方还很空")
        d_head = min(d_f, min(d_fl, d_fr) * 0.72)

        if v_t > 0.02:                                  # ---- 前进 ----
            if d_head <= stop_d:
                v_t = 0.0
                w_t = 1.1 if d_fl >= d_fr else -1.1     # 朝更空的一侧偏转
                note = (f'全向急停 前{d_f:.2f} 左前{d_fl:.2f}'
                        f' 右前{d_fr:.2f}')
            elif d_head < slow_d:
                v_cap = (d_head - stop_d) / tau
                if v_t > v_cap:
                    v_t = max(0.0, v_cap)
                note = (f'全向限速 前{d_f:.2f} 左前{d_fl:.2f}'
                        f' 右前{d_fr:.2f} v<={v_cap:.2f}')
                if v_t > 0.05 and abs(d_fl - d_fr) > 0.25 \\
                        and min(d_fl, d_fr) > stop_d:
                    w_t += 0.55 if d_fl > d_fr else -0.55
            if v_t > 0.05:                              # 侧向防擦
                if d_l < side_d:
                    w_t -= 0.9 * (side_d - d_l) / side_d
                if d_r < side_d:
                    w_t += 0.9 * (side_d - d_r) / side_d
                # P26: 侧翼必须包含 bl/br 两个后翼。绕障时障碍的方位会从
                # 90° 连续转到 135°, 只盯 left/right 会在这段漏掉, 于是
                # "擦着立柱边走" —— 实测 C/E 的 -0.033/-0.059 就是这么来的
                d_wing = min(d_l, d_r, _pv('bl'), _pv('br'))
                if d_wing < stop_d:
                    v_t = min(v_t, 0.10)                # 侧翼已贴死, 别再挤
        elif v_t < -0.05:                               # ---- 后退 ----
            # "防撞人"和"卡死脱困"都会下发负速度, 原来这两条路径完全不看
            # 后方 —— E 场景擦碰就是倒车撞上背后的立柱
            if d_b < stop_d:
                v_t = 0.0
                note = f'后退受阻 后{d_b:.2f}'
            elif d_b < slow_d:
                v_cap_b = -(d_b - stop_d) / tau
                if v_t < v_cap_b:
                    v_t = v_cap_b
                    note = f'后退限速 后{d_b:.2f}'

        if note and now - self._last_avoid_log > 1.0:
            self._last_avoid_log = now
            self.get_logger().info(
                f'FOLLOWER {note} | 左{d_l:.2f} 右{d_r:.2f} 后{d_b:.2f}')
        return v_t, w_t, note

'''

COMMENT = '''        # --- 7.5 全向安全层 ---
        # 已抽成 _safety_gate(), 在 _tick 的每个出口统一调用。原因: 对正、
        # 原地扫描、防撞人后退、卡死脱困都会直接设速度, 只要有一处绕过
        # 安全层, 就会在那里撞 —— 实测 E 场景(斜冲 ob5)的 -0.062m 擦碰
        # 正是"倒车时没看后方"造成的。
'''

# ---- 1. 摘除原来的 7.5 段 ----
start_tok = '        # --- 7.5 避障层3:'
end_tok = "                    f'FOLLOWER {avoid_note} | 左{d_l:.2f} 右{d_r:.2f}')"
try:
    i0 = s.index(start_tok)
    i1 = s.index(end_tok) + len(end_tok)
except ValueError as e:
    print('!! 定位失败:', e)
    sys.exit(1)
s = s[:i0] + COMMENT + s[i1:]

# ---- 2. 插入方法 ----
anchor = '    # ---------- 输出与日志 ----------'
if s.count(anchor) != 1:
    print('!! 方法插入锚点异常')
    sys.exit(1)
s = s.replace(anchor, METHOD + anchor, 1)

# ---- 3. 出口A: 原地扫描 ----
old = """        if src == '扫描':
            w_t = self.cfg['recenter_rot_speed'] * 0.8
            self._publish_smooth(v_t, w_t, now)"""
new = """        if src == '扫描':
            w_t = self.cfg['recenter_rot_speed'] * 0.8
            v_t, w_t, _sn = self._safety_gate(v_t, w_t, now)
            self._publish_smooth(v_t, w_t, now)"""
if s.count(old) != 1:
    print('!! 出口A 匹配失败')
    sys.exit(1)
s = s.replace(old, new, 1)

# ---- 4. 出口B: 对正(ALIGN) ----
old = """            note = (f'对正{math.degrees(brg):+.0f}°'
                    f'{"(边转边走)" if v_t > 0 else ""}'
                    f'{"(方向锁定)" if self._align_dir != 0 else ""}')
            self._publish_smooth(v_t, w_t, now)"""
new = """            note = (f'对正{math.degrees(brg):+.0f}°'
                    f'{"(边转边走)" if v_t > 0 else ""}'
                    f'{"(方向锁定)" if self._align_dir != 0 else ""}')
            v_t, w_t, _sn = self._safety_gate(v_t, w_t, now)
            if _sn:
                note += ' ' + _sn
            self._publish_smooth(v_t, w_t, now)"""
if s.count(old) != 1:
    print('!! 出口B 匹配失败')
    sys.exit(1)
s = s.replace(old, new, 1)

# ---- 5. 出口C: TRACK 末尾(卡死脱困之后) ----
old = """            else:
                self._stuck_t0 = 0.0

        self._publish_smooth(v_t, w_t, now)
        self._log(now, src, dist, pspd, rspd, w_t, brg, note, nscan,
                  rx, ry, px, py)"""
new = """            else:
                self._stuck_t0 = 0.0

        # 最终裁剪: 任何路径算出的速度都要过这一关
        v_t, w_t, _sn = self._safety_gate(v_t, w_t, now)
        if _sn:
            note = _sn if not note else (note + ' ' + _sn)
        self._publish_smooth(v_t, w_t, now)
        self._log(now, src, dist, pspd, rspd, w_t, brg, note, nscan,
                  rx, ry, px, py)"""
if s.count(old) != 1:
    print('!! 出口C 匹配失败')
    sys.exit(1)
s = s.replace(old, new, 1)

open(P, 'w', encoding='utf-8').write(s)
print('OK: 安全层已抽为 _safety_gate, 并在 3 个出口调用')
print('  safety_gate 调用次数:', s.count('self._safety_gate('))
