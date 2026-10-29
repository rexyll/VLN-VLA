#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_patch_gate2.py —— P27: 急停转向去振荡 + 持续受阻脱困

实测日志(避障第5轮后):
    [对正|搜索] 距离=2.27 方位=-37° 指令v=+0.00 w=-1.0
    [对正|搜索] 距离=2.26 方位=-46° 指令v=+0.00 w=-1.4
    [对正|搜索] 距离=2.28 方位=-25° 指令v=+0.00 w=+1.1  ← 符号翻转
    FOLLOWER 全向急停 前0.72 左前2.06 右前0.59

P26 的急停转向是 `w_t = 1.1 if d_fl >= d_fr else -1.1` —— 只看哪侧空,
不看上层意图。目标在 -37°(右侧)时上层要右转, 安全层却因左侧空而左转,
下一帧 d_fl/d_fr 又换过来 → **w 每帧翻号**, 车在原地抖、原地打转, 却没
真正绕开障碍。

修法:
  1) 转向方向优先沿用上层意图, 只要那一侧还有余量(stop*0.92);
     意图侧被堵死才反向。
  2) 加滞回(esc_hold_s): 选定方向后至少保持 0.45s, 期间只有"另一侧
     明显更空 0.35m 以上"才允许换边。
  3) 加持续受阻脱困(blk_s): 原地转 >2.2s 仍过不去且后方有余量 →
     先退 0.18m/s 给转向腾空间, 退 1.8s 后重新计时, 防止无限后退。
"""
import shutil
import sys

P = '/home/earl/workspace/VLA/vla_ws/src/vla_follow/vla_follow/follower.py'
shutil.copy(P, P + '.bak_gate2')
s = open(P, encoding='utf-8').read()

# ---- 1. __init__ 新增状态 ----
old = "        self._last_avoid_log = 0.0    # P26: 全向安全层单独的日志限流"
new = """        self._last_avoid_log = 0.0    # P26: 全向安全层单独的日志限流
        # P27: 急停绕行的方向记忆 —— 没有它 w 会每帧翻号(见 patch 说明)
        self._esc_dir = 0.0           # -1 右转 / +1 左转 / 0 未定
        self._esc_t0 = 0.0            # 当前方向锁定的起始时刻
        self._blk_t0 = 0.0            # 前方持续受阻的起始时刻(0=未受阻)"""
if s.count(old) != 1:
    print('!! __init__ 锚点异常')
    sys.exit(1)
s = s.replace(old, new, 1)

# ---- 2. 替换前进分支的急停处理 ----
OLD = """            if d_head <= stop_d:
                v_t = 0.0
                w_t = 1.1 if d_fl >= d_fr else -1.1     # 朝更空的一侧偏转
                note = (f'全向急停 前{d_f:.2f} 左前{d_fl:.2f}'
                        f' 右前{d_fr:.2f}')"""
NEW = """            if d_head <= stop_d:
                v_t = 0.0
                # P27: 急停时的转向必须与上层意图协同。原来是"哪侧空就
                # 往哪转", 与上层"目标在右就右转"直接对打 —— 实测 w 在
                # -1.0 -> -1.4 -> +1.1 之间每帧翻号, 车在原地抖。
                room_l, room_r = d_fl, d_fr
                want = 0.0
                if w_t > 0.05 and room_l > stop_d * 0.92:
                    want = 1.0                       # 想左转且左前有余量
                elif w_t < -0.05 and room_r > stop_d * 0.92:
                    want = -1.0                      # 想右转且右前有余量
                if want == 0.0:                      # 直冲, 或意图侧被堵死
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
                        f' 绕{"左" if self._esc_dir > 0 else "右"}')
                # P27: 原地转不开 → 先退再绕。静态障碍挡在目标正前方时
                # (人站在立柱后面), 纯原地转是死锁: 转到障碍离开视野,
                # 上层又把车头转回来, 于是永久"转-停-转"。
                if self._blk_t0 <= 0.0:
                    self._blk_t0 = now
                elif now - self._blk_t0 > self.cfg['blk_s']:
                    if d_b > stop_d + 0.40 and \\
                            now - self._blk_t0 < self.cfg['blk_s'] + 1.8:
                        v_t = -0.18
                        note += ' 受阻>%.1fs→后退脱困' % self.cfg['blk_s']
                    else:
                        self._blk_t0 = now       # 退够/后方也堵: 重新计时"""
if s.count(OLD) != 1:
    print('!! 急停分支锚点异常')
    sys.exit(1)
s = s.replace(OLD, NEW, 1)

# ---- 3. 未受阻时清空计时 ----
OLD2 = """                d_wing = min(d_l, d_r, _pv('bl'), _pv('br'))
                if d_wing < stop_d:
                    v_t = min(v_t, 0.10)                # 侧翼已贴死, 别再挤"""
NEW2 = """                d_wing = min(d_l, d_r, _pv('bl'), _pv('br'))
                if d_wing < stop_d:
                    v_t = min(v_t, 0.10)                # 侧翼已贴死, 别再挤
            if d_head > stop_d:
                self._blk_t0 = 0.0                      # 已脱离受阻"""
if s.count(OLD2) != 1:
    print('!! 脱离受阻锚点异常')
    sys.exit(1)
s = s.replace(OLD2, NEW2, 1)

# ---- 4. 两个新参数 ----
OLD3 = "    'escape_s': 0.90,               # 卡死后脱困动作持续时长"
if s.count(OLD3) != 1:
    print('!! 参数锚点异常')
    sys.exit(1)
NEW3 = """    'escape_s': 0.90,               # 卡死后脱困动作持续时长
    'esc_hold_s': 0.45,             # P27 绕障转向方向最短保持时间
    'blk_s': 2.20,                  # P27 原地受阻多久后改后退脱困"""
s = s.replace(OLD3, NEW3, 1)

open(P, 'w', encoding='utf-8').write(s)
print('PATCH_OK')
print('  _safety_gate 调用次数:', s.count('self._safety_gate('))
print('  新状态 _esc_dir:', s.count('_esc_dir'), ' _blk_t0:', s.count('_blk_t0'))
print('  新参数 esc_hold_s:', s.count('esc_hold_s'),
      ' blk_s:', s.count("'blk_s'"))
