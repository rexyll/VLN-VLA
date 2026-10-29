#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_patch_p33.py —— P33: 人的随机行走更接近真实"时快时慢 + 驻足"

  W1  _pick_walk 速度区间 0.20~0.45 → 0.15~0.60
  W2  "机器人靠近 0.85m 就逃" → 0.45m(贴身才让开)
      原逻辑与 track_dist_m=1.0 直接冲突: 机器人一靠近目标距离人就主动
      远离, 距离环永远收敛不到 1.0m。
  W3  WALK 段内叠加速度抖动(每 2.5s 以 60% 概率重采), 制造"时快时慢"
"""
import io
import sys

P = 'vla_ws/src/vla_follow/vla_follow/person_walk.py'
s = io.open(P, encoding='utf-8').read()
if 'P33' in s:
    print('!! 已打过 P33')
    sys.exit(1)


def rep(old, new, tag):
    global s
    n = s.count(old)
    if n != 1:
        print('!! [%s] 锚点命中 %d 次' % (tag, n))
        sys.exit(1)
    s = s.replace(old, new, 1)
    print('   ok %s' % tag)


# W1 + 说明
rep("""    def _pick_walk(self):
        self.target_speed = random.uniform(0.2, 0.45)""",
    """    def _pick_walk(self):
        # P33: 速度幅值拉开(原 0.20~0.45 变化不明显, 体现不出"时快时慢")
        self.target_speed = random.uniform(0.15, 0.60)""",
    'W1')

# W2
rep("""        # P22: 原阈值 1.3m 与跟随目标距离 track_dist_m=1.0 直接冲突 ——
        # 机器人刚走到 1.0m 人就开逃, 机器人永远在追, 永远停不下来,
        # 表现为"走走停停 + 方位反复甩"。取 0.85 略小于
        # track_dist_m - track_dist_tol_m(0.9), 人才可能被稳定跟随。
        robot_close = math.hypot(x - rx, y - ry) < 0.85""",
    """        # P22: 原阈值 1.3m 与跟随目标距离 track_dist_m=1.0 直接冲突 ——
        # 机器人刚走到 1.0m 人就开逃, 机器人永远在追。
        # P33: 0.85 仍然太高 —— 距离环允许下探到 0.9m, 一旦过冲(0.85)
        # 人就主动远离, 等于机器人被"追着跑"的对象反击, 1.0±0.1 永远收敛
        # 不了。改成 0.45m 只在贴身(机器人压上来)时让开, 正常跟随不干预。
        robot_close = math.hypot(x - rx, y - ry) < 0.45""",
    'W2')

# W3: WALK 段内速度抖动
rep("""        if now >= self.state_until:
            self._next_state()
        amax = 0.35 * dt  # 人的加/减速度, 保证速度变化平滑""",
    """        if now >= self.state_until:
            self._next_state()
        # P33: WALK 段内也做速度抖动, 制造"时快时慢"(原实现每段速度恒定)
        if self.state == 'WALK' and now - self._vmod_t > 2.5:
            self._vmod_t = now
            if random.random() < 0.6:
                self.target_speed = random.uniform(0.15, 0.60)
        amax = 0.35 * dt  # 人的加/减速度, 保证速度变化平滑""",
    'W3')

# __init__ 初始化 _vmod_t
rep("""        self._last_pos_log = 0.0""",
    """        self._last_pos_log = 0.0
        self._vmod_t = 0.0          # P33: WALK 段内速度抖动计时""",
    'W4')

io.open(P, 'w', encoding='utf-8', newline='\n').write(s)
print('P33_APPLIED')
