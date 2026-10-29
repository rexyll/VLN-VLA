# -*- coding: utf-8 -*-
"""离线校验 path_free: 用 eval14 死锁现场的真实位形, 对照旧判据。"""
import math

src = open('_p57_follower.py', encoding='utf-8').read()
geo = {'math': math, 'WALL_SOFT': 5.15, 'WALL_HARD': 5.75, 'ROBOT_R': 0.18}
exec(src[src.index('GEO = ['):src.index('def wrap(a):')], geo)
path_free, geo_gap = geo['path_free'], geo['geo_gap']
stop_d, slow_d = 0.66, 1.60


def verdict(dh):
    if dh <= stop_d:
        return '停'
    return '限' if dh < slow_d else '行'


CASES = [
    ('eval14 死锁现场 ob2 角旁', -2.24, 2.84, 39, 0.34, 4.19, 1.63),
    ('同上, 车头正对 ob2',        -2.24, 2.84, 115, 0.34, 4.19, 1.63),
    ('ob1 正前 0.60m',             2.50, 2.00, 0, 3.00, 3.00, 0.60),
    ('ob1 侧方 0.45m 平行直行',     3.50, 3.00, 0, 0.60, 3.00, 4.00),
    ('ob7 正前 1.00m',             2.20, 3.25, 90, 4.00, 4.00, 1.00),
    ('ob7 右前 40 度 1.20m 斜穿',   2.98, 3.83, 50, 1.20, 2.00, 1.20),
    ('ob6 正前 1.10m',            -0.40, -4.95, 180, 3.00, 3.00, 1.10),
    ('空旷场心',                    0.00, 0.00, 0, 9.00, 9.00, 9.00),
]

print('%-26s %5s %5s %6s | %-5s %7s | %-5s %7s %6s' % (
    '场景', 'd_fl', 'd_fr', 'd_f', '旧判', '旧d_head', '新判', '新d_head', '间隙'))
for name, x, y, yadeg, d_fl, d_fr, d_f in CASES:
    old_d = min(d_f, min(d_fl, d_fr) * 0.72) if min(d_fl, d_fr) < 0.85 else d_f
    dp, gp, obj = path_free(x, y, math.radians(yadeg))
    nd = min(d_f, dp)
    print('%-26s %5.2f %5.2f %6.2f | %-5s %7.3f | %-5s %7.3f %6.3f %s' % (
        name, d_fl, d_fr, d_f, verdict(old_d), old_d,
        verdict(nd), nd, gp,
        '' if obj is None else (obj if isinstance(obj, str) else '%s%s' % (obj[0], obj[1:3]))))
