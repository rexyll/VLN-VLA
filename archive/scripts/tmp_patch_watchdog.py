#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_patch_watchdog.py —— P30: 感知中断看门狗 + 修掉 P29 的格式串崩溃

两件事:

1) 修 P29 引入的崩溃:
       off = (2 - worst_k) * SECT_DEG      # SECT_DEG = 18.4 (float)
       f'{off:+d}°'                        # -> ValueError: Unknown format
                                                code 'd' for object of type float
   depth_percept 在开机第一条误差日志就抛异常退出, 之后 follower 一直拿
   **冻结的最后一帧** 做避障: 日志里 "前0.83 左前3.10 右前3.90" 45 秒
   一字未变, 全程 全向限速 v<=0.28、0 次急停, 于是 B/C/D 三个场景擦碰。

2) 加看门狗(根因层面的修复):
   安全层原来只用消息 payload 里的 age 判断数据新旧。但 age 是**发布方**
   算出来的"深度帧有多旧", 发布方一旦死掉/卡住, last message 的 age 就永远
   冻在那个小值上 —— 消费者无法察觉。这正是"有时候会撞上去"的隐秘来源。
   现在按**收到消息的时刻**做看门狗:
       数据中断 >= 0.5s -> 禁止前进(可以转), 并且每 3s 报一次错
   因为没数据的避障层无法保证不撞, 停下来是唯一诚实的处理。
"""
import shutil
import sys

# ---------- 1) 修 depth_percept 的格式串 ----------
PD = '/home/earl/workspace/VLA/vla_ws/src/vla_follow/vla_follow/depth_percept.py'
shutil.copy(PD, PD + '.bak_wd')
d = open(PD, encoding='utf-8').read()

OLD = "                    off = (2 - worst_k) * SECT_DEG"
NEW = "                    off = int(round((2 - worst_k) * SECT_DEG))"
if d.count(OLD) != 1:
    print('!! off 锚点异常')
    sys.exit(1)
d = d.replace(OLD, NEW, 1)
open(PD, 'w', encoding='utf-8').write(d)
print('OK 格式串已修')

# ---------- 2) follower 看门狗 ----------
PF = '/home/earl/workspace/VLA/vla_ws/src/vla_follow/vla_follow/follower.py'
shutil.copy(PF, PF + '.bak_wd')
s = open(PF, encoding='utf-8').read()

# 2a. 状态初始化
OLD = "        self._blk_t0 = 0.0            # 前方持续受阻的起始时刻(0=未受阻)"
NEW = """        self._blk_t0 = 0.0            # 前方持续受阻的起始时刻(0=未受阻)
        # P30: 感知看门狗 —— 按**收到消息的时刻**判断, 不能只信 payload 里的
        # age(那是发布方算的, 发布方死了就永远冻在小值上)
        self._prox_rx = 0.0           # 最近一次收到 proximity 的时刻
        self._prox_dead_log = 0.0"""
if s.count(OLD) != 1:
    print('!! init 锚点异常')
    sys.exit(1)
s = s.replace(OLD, NEW, 1)

# 2b. _prox_cb 记录收包时刻
OLD = """        self.prox = {k: p.get(k, -1.0) for k in keys}
        self.prox['age'] = p.get('age', 99.0)"""
NEW = """        self.prox = {k: p.get(k, -1.0) for k in keys}
        self.prox['age'] = p.get('age', 99.0)
        self._prox_rx = time.monotonic()      # P30 看门狗"""
if s.count(OLD) != 1:
    print('!! _prox_cb 锚点异常')
    sys.exit(1)
s = s.replace(OLD, NEW, 1)

# 2c. _safety_gate 开头加看门狗
OLD = """        p8 = self.prox
        if p8.get('age', 99.0) >= self.cfg['lidar_max_age_s']:
            return v_t, w_t, ''"""
NEW = """        p8 = self.prox
        # P30: 先看"多久没收到新数据", 再看 payload 里的 age。
        # 只查 age 是不够的 —— depth_percept 一死, last message 的 age 就
        # 永远冻在最后那个小值上, 安全层会无限期拿过期数据当实时数据用。
        stale = now - self._prox_rx if self._prox_rx > 0.0 else 1e9
        if stale > 0.5:
            if now - self._prox_dead_log > 3.0:
                self._prox_dead_log = now
                self.get_logger().error(
                    'FOLLOWER 感知数据中断 %.1fs —— depth_percept 可能已退出, '
                    '禁止前进(仍可转向)' % stale)
            if v_t > 0.0:                      # 盲开比停下危险
                v_t = 0.0
            return v_t, w_t, '感知中断禁行'
        if p8.get('age', 99.0) >= self.cfg['lidar_max_age_s']:
            return v_t, w_t, ''"""
if s.count(OLD) != 1:
    print('!! gate 锚点异常')
    sys.exit(1)
s = s.replace(OLD, NEW, 1)

open(PF, 'w', encoding='utf-8').write(s)
print('OK 看门狗已加')
print('  _prox_rx:', s.count('_prox_rx'))
