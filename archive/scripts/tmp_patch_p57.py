# -*- coding: utf-8 -*-
"""tmp_patch_p57.py —— 把 path_free 拆成"距离"和"走廊宽度"两个量

P53 的 path_free 把两件事塞进一个返回值: 既当"还能直行多远", 又用
ROBOT_R+margin 判定拦截。结果是**重复扣车体半径** —— 正前方 1.0m 的
障碍被算成 d_head=0.65, 再跟 stop_d=0.66 比就变成"停在 1.0m 外"。

P57 拆开:
  d_surf : 沿航向到"车心将触到表面"的距离(= 表面距离), 与 d_f 同量纲,
           直接和 stop_d/slow_d 比较, 不再扣半径。
  tight  : 走廊是否够宽(沿途任一点中心间隙 < ROBOT_R+margin 即 False)。
           专拦"贴着角切过去" —— evalN(seed7) 贴着 ob5 的角以 0.25m
           中心间隙切入, 最后卡死 45s。
"""
import io

SRC = '_p56_follower.py'
DST = '_p57_follower.py'

src = io.open(SRC, encoding='utf-8').read()

i0 = src.index('def path_free(')
i1 = src.index('def wrap(a):')
OLD = src[i0:i1]
print('[1] 替换 path_free (%d 字节)' % len(OLD))

NEW = '''def path_free(x, y, ya, margin=0.10, s_max=1.60, step=0.05):
    """沿车头方向扫掠。返回 (可用直行距离, 走廊是否够宽, 首个拦截物)。

    P53/P57。为什么不再用 `min(d_fl,d_fr)*0.72`: 8 扇区每个方向只有一个
    "最近距离", 拿 45° 扇区值乘 0.72 当"正前方余量"是没有真值地图时代的
    代偿 —— 它假定障碍正好在 45°, 而扇区跨度约 20~70°, 实际角度未知。
    实测 eval14 的死锁现场: ob2 的角在车头左前 64°、车心到表面 0.395m,
    沿 39° 航向通过时法向间隙 0.355m(车体半径 0.18) —— 完全过得去;
    但 0.395*0.72 = 0.284 < stop_d 0.66 直接急停, 急停又强制反向转,
    与上层"转向目标"互掐, 造成极限环。

    两个返回值必须分开, 否则重复扣车体半径:
      d_surf —— 沿航向到"车心将触到表面"的距离, 就是**表面距离**,
                和 d_f 同量纲, 可以直接和 stop_d/slow_d 比较;
      tight  —— 走廊宽度够不够(沿途任一点中心间隙 < ROBOT_R+margin
                即 False)。这一项单独拦"贴着角切过去": 实测 evalN
                (PW_SEED=7) 机器人贴着 ob5(4.2,-3.0) 的角、以 0.25m
                中心间隙切入, 然后卡在角里 —— 清距 0.07m, 既转不动
                (车体外接圆扫进障碍)、也退不出(ob5 在侧后方, d_b 被
                判"后退受阻"), 冻死 45s, 人从 1.4m 走到 5.96m。
    """
    best = 99.0
    tight = True
    s = step
    while s <= s_max + 1e-9:
        nx = x + math.cos(ya) * s
        ny = y + math.sin(ya) * s
        if abs(nx) > WALL_HARD or abs(ny) > WALL_HARD:
            return max(0.0, s - step), tight, 'wall'
        gmin, gobj = 99.0, None
        for g in GEO:
            gp = geo_gap(nx, ny, g)
            if gp < gmin:
                gmin, gobj = gp, g
        if gmin < best:
            best = gmin
        if gmin <= ROBOT_R + margin:
            tight = False
        if gmin <= 0.02:            # 车心即将触到表面 → 这就是可用距离
            return max(0.0, s - step), tight, gobj
        s += step
    return s_max, tight, None


'''
src = src.replace(OLD, NEW, 1)

# ---- 段6 调用点 ----
old6 = "                _dpath, _gap6, _obj6 = path_free(rx, ry, ryaw)\n                if _dpath < 0.33:\n"
new6 = ("                _dpath, _ok6, _obj6 = path_free(rx, ry, ryaw)\n"
        "                if _dpath < 0.33 or not _ok6:\n")
assert old6 in src
src = src.replace(old6, new6, 1)
print('[2] 段6 调用点已更新')

# ---- d_head 调用点 ----
old3 = """        _dpath, _gp, _objp = path_free(self.rx, self.ry, self.ryaw)
        if self._avoiding:
            d_head = d_f             # 绕障时车头已偏向侧方, 看正前方即可
        else:
            d_head = min(d_f, _dpath)
"""
new3 = """        _dpath, _okp, _objp = path_free(self.rx, self.ry, self.ryaw)
        if self._avoiding:
            d_head = d_f             # 绕障时车头已偏向侧方, 看正前方即可
        elif not _okp:
            d_head = 0.0             # 走廊太窄(贴角) → 不许再往前挤
        else:
            d_head = min(d_f, _dpath)
"""
assert old3 in src
src = src.replace(old3, new3, 1)
print('[3] d_head 调用点已更新')

io.open(DST, 'w', encoding='utf-8', newline='\n').write(src)
print('[OK] 写出 %s (%d 行)' % (DST, len(src.split('\n'))))
