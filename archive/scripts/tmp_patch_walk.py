#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_patch_walk.py —— 修正 person_walk.py 的两处硬伤

1) 危险区表里有 4 个 world 中不存在的"幽灵障碍", 令人长期处于 ESCAPE 而原地抖动;
2) robot_close 阈值 1.3m 与跟随目标距离 track_dist_m=1.0 直接冲突 ——
   机器人刚走到 1.0m, 人就开始逃, 于是永远在追、永远停不下来。
"""
import io

P = '/home/earl/workspace/VLA/vla_ws/src/vla_follow/vla_follow/person_walk.py'
s = io.open(P, encoding='utf-8').read()

OLD1 = """OBSTACLES = [
    (3.5, 2.0, 0.75),
    (-3.0, 3.2, 0.85),
    (0.5, -3.2, 0.60),
    (-4.2, -2.5, 0.75),
    (4.2, -3.0, 0.55),
    (-1.5, -4.5, 0.80),
    (2.2, 4.6, 0.60),
]"""
NEW1 = """# P22 修正: 只保留 work_area.sdf 里真实存在的 3 根立柱。
# 原表有 4 项在世界中没有任何几何体, 人一走进这些"幽灵危险区"就转 ESCAPE
# 并原地抖动(实测位置长时间卡在 (+1.00,+0.00) 不动)。
# 半径 = 立柱半径 + 0.2m 人体余量。
OBSTACLES = [
    (0.5, -3.2, 0.60),
    (-4.2, -2.5, 0.70),
    (2.2, 4.6, 0.55),
]"""

OLD2 = 'robot_close = math.hypot(x - rx, y - ry) < 1.3'
NEW2 = """# P22: 原阈值 1.3m 与跟随目标距离 track_dist_m=1.0 直接冲突 ——
        # 机器人刚走到 1.0m 人就开逃, 机器人永远在追, 永远停不下来,
        # 表现为"走走停停 + 方位反复甩"。取 0.85 略小于
        # track_dist_m - track_dist_tol_m(0.9), 人才可能被稳定跟随。
        robot_close = math.hypot(x - rx, y - ry) < 0.85"""

for old, new in ((OLD1, NEW1), (OLD2, NEW2)):
    if old not in s:
        raise SystemExit('!! 未找到待替换片段:\n' + old[:60])
    s = s.replace(old, new, 1)

io.open(P, 'w', encoding='utf-8').write(s)
print('PATCH_OK')
