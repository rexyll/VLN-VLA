#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""修正 follower / person_walk 的障碍表。

背景: 这两张表曾被缩减成 3 项，理由是"world 里只有 3 根立柱"。
实际 work_area.sdf 的 <model name="obstacles"> 下有 ob1..ob7 共 7 个几何体，
遗漏的 4 个会导致:
  - 机器人绕障逻辑不认这些柱子 → 直接撞上去, 指令 v=0.12 但实际 v=0.01 (卡死)
  - 人的危险区判定漏项 → 走进柱子区域不触发逃逸
"""
import shutil

FOLLOWER = ('/home/earl/workspace/VLA/vla_ws/src/vla_follow/vla_follow/follower.py')
WALKER = ('/home/earl/workspace/VLA/vla_ws/src/vla_follow/vla_follow/person_walk.py')

OLD_F = """OBSTACLES = [           # (x, y, 障碍半径) —— 与 SDF 一一对应
    (0.5, -3.2, 0.40),
    (-4.2, -2.5, 0.50),
    (2.2, 4.6, 0.35),
]"""

NEW_F = """# P24 修正: 原表只剩 3 项(误判为"幽灵障碍"), 实际 SDF 的 obstacles 模型下
# 有 ob1..ob7 共 7 个几何体。漏项会让绕障直接撞柱子: 实测机器人顶着 ob3
# (0.5,-3.2) 绿柱, 指令 v=+0.12 而实际 v=+0.01, 原地卡死。
# 半径取水平外接圆(方块按半对角), 保证斜向也不会擦碰。
OBSTACLES = [           # (x, y, 外接半径) —— 与 work_area.sdf ob1..ob7 一一对应
    (3.5, 2.0, 0.57),     # ob1 0.8x0.8 方块
    (-3.0, 3.2, 0.67),    # ob2 1.2x0.6 方块(yaw 0.5)
    (0.5, -3.2, 0.40),    # ob3 r0.40 圆柱
    (-4.2, -2.5, 0.50),   # ob4 r0.50 圆柱
    (4.2, -3.0, 0.42),    # ob5 0.6x0.6 方块
    (-1.5, -4.5, 0.56),   # ob6 1.0x0.5 方块(yaw -0.4)
    (2.2, 4.6, 0.35),     # ob7 r0.35 圆柱
]"""

OLD_W = """OBSTACLES = [
    (0.5, -3.2, 0.60),
    (-4.2, -2.5, 0.70),
    (2.2, 4.6, 0.55),
]"""

NEW_W = """# P24 修正: 与 follower 同步为 SDF 里真实存在的 7 个障碍(半径 +0.2m 人体余量)
OBSTACLES = [
    (3.5, 2.0, 0.62),
    (-3.0, 3.2, 0.72),
    (0.5, -3.2, 0.60),
    (-4.2, -2.5, 0.70),
    (4.2, -3.0, 0.62),
    (-1.5, -4.5, 0.76),
    (2.2, 4.6, 0.55),
]"""

for path, old, new, tag in ((FOLLOWER, OLD_F, NEW_F, 'follower'),
                            (WALKER, OLD_W, NEW_W, 'person_walk')):
    s = open(path, encoding='utf-8').read()
    n = s.count(old)
    if n != 1:
        print('%s: 未找到唯一目标段 count=%d' % (tag, n))
        continue
    shutil.copy(path, path + '.bak_obs')
    open(path, 'w', encoding='utf-8').write(s.replace(old, new))
    print('%s: 已更新为 7 项障碍' % tag)

import ast
for path in (FOLLOWER, WALKER):
    ast.parse(open(path, encoding='utf-8').read())
print('语法检查通过')
