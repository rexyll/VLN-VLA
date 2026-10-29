# -*- coding: utf-8 -*-
"""tmp_patch_p53.py —— 用真值几何扫掠求"沿航向的可用直行距离"

根因(2026-10-25 eval14 实证): 安全层用
    d_head = min(d_f, min(d_fl,d_fr) * 0.72)
把"左前 64° 方向 0.34m 的障碍"折成"正前方 0.245m"→ 急停; 急停又强制
往空侧转, 与 ALIGN"转向目标"每 0.5s 互掐一次, 于是 w 在 +1.1/-0.9 之间
翻、距离从 1.05m 一路涨到 2.12m, 60s 没前进 1mm, 最后丢人。

做法: GEO(7 个障碍的精确 OBB/圆) 沿车头方向以 5cm 步长扫到 1.60m, 求
"还能直行多远"; d_head 直接取它, stop_d/slow_d 语义完全不变。8 扇区
fl/fr 降级为"只限速"。
"""
import io

SRC = '_cur_follower.py'
DST = '_p53_follower.py'

src = io.open(SRC, encoding='utf-8').read()

HELPER = '''def path_free(x, y, ya, margin=0.06, s_max=1.60, step=0.05):
    """沿车头方向扫掠, 求"还能直行多远"。返回 (距离, 最紧间隙, 拦截物)。

    P53。为什么不再用 `min(d_fl,d_fr)*0.72`: 8 扇区每个方向只有一个
    "最近距离", 拿 45° 扇区值乘 0.72 当"正前方余量"是没有真值地图时代的
    代偿 —— 它假定障碍正好在 45°, 而扇区跨度约 20~70°, 实际角度未知。
    实测 eval14 的死锁现场: ob2 的角在车头左前 64°、车心到表面 0.395m,
    沿 39° 航向通过时法向间隙 0.355m(车体半径 0.18) —— 完全过得去;
    但 0.395*0.72 = 0.284 < stop_d 0.66 直接急停, 急停又强制反向转,
    与上层"转向目标"互掐, 造成极限环。
    真值地图里 7 个障碍的位置/尺寸精确已知, 直接扫掠即可, 不需要代理。

    返回的"距离"与 d_f 同量纲(车心到障碍表面的米数), 因此可以直接和
    stop_d / slow_d 比较。
    """
    best = 99.0
    s = step
    while s <= s_max + 1e-9:
        nx = x + math.cos(ya) * s
        ny = y + math.sin(ya) * s
        if abs(nx) > WALL_HARD or abs(ny) > WALL_HARD:
            return max(0.0, s - step), best, 'wall'
        for g in GEO:
            gp = geo_gap(nx, ny, g)
            if gp < best:
                best = gp
            if gp <= ROBOT_R + margin:
                return max(0.0, s - step), gp, g
        s += step
    return s_max, best, None


'''
anchor = 'def wrap(a):'
assert anchor in src
src = src.replace(anchor, HELPER + anchor, 1)
print('[1] path_free 已插入')

# ---------- 2. 段6: 单点前瞻+外接圆 → 扫掠真值(仅作贴死兜底) ----------
i0 = src.index("            else:\n                _g_hit = None\n")
tail = "                            '右绕' if _g_lat > 0 else '左绕')).strip()\n"
i1 = src.index(tail) + len(tail)
OLD6 = src[i0:i1]
print('[2] 替换段6 障碍判据 %d 字节' % len(OLD6))

NEW6 = '''            else:
                # P53: 单点前瞻 + 外接圆横向判据 → 扫掠真值几何。
                # 原判据两处过保守: ① 只在前瞻一个点(0.35+0.5v)取样, 对
                # 0.2m 处的障碍视而不见; ② `_lat > _ext + ROBOT_R` 里的
                # _ext 是 box 的**外接圆**半径(ob2 的 1.2x0.6 → 0.671, 真实
                # 半宽只有 0.60/0.30), 横向净空被低估近 0.4m。
                # 这里只做"马上要撞了"的兜底; 分级减速交给 _safety_gate。
                _dpath, _gap6, _obj6 = path_free(rx, ry, ryaw)
                if _dpath < 0.33:
                    v_t = 0.0
                    if isinstance(_obj6, str):
                        note = (note + ' 墙前硬停').strip()
                    elif _obj6 is not None:
                        # P45 关键: 必须给转向。停车 + w=0 = 永久死锁。
                        _lat6 = (-(_obj6[1] - rx) * math.sin(ryaw)
                                 + (_obj6[2] - ry) * math.cos(ryaw))
                        w_t = -1.10 if _lat6 > 0 else 1.10
                        note = (note + ' 真值硬保(%+.1f,%+.1f)间隙%.2f%s'
                                % (_obj6[1], _obj6[2], _gap6,
                                   '右绕' if _lat6 > 0 else '左绕')).strip()
'''
src = src.replace(OLD6, NEW6, 1)

# ---------- 3. d_head: 去掉 0.72 折算 ----------
i0 = src.index("        _d_diag = min(d_fl, d_fr)\n")
last = "            d_head = min(d_f, _d_diag * 0.72) if _d_diag < 0.85 else d_f\n"
i1 = src.index(last) + len(last)
OLD3 = src[i0:i1]
print('[3] 替换 d_head 判据 %d 字节' % len(OLD3))

NEW3 = '''        # P53: "正前方还能走多远" = 真值几何沿航向扫掠的结果, 8 扇区
        # (d_fl/d_fr) 只保留"限速"职责, 不再参与"停不停"。
        # 原 `min(d_f, min(d_fl,d_fr)*0.72)` 在 eval14 造成极限环:
        #   车心到 ob2 表面 0.38m, ob2 的角在车头左前 64°、0.395m 处;
        #   d_fl=0.34 → d_head=0.245 < stop_d 0.66 → 急停 → 强制绕右(w=-0.9)
        #   → ALIGN 把车头转回目标(+1.1) → 左前再次变窄 → 再急停 …
        #   实测 w 每 0.5s 翻号, 距离 1.05→2.12m 单调变大。
        #   而同一位置沿 39° 的法向间隙有 0.355m(车半径 0.18), 过得去。
        _dpath, _gp, _objp = path_free(self.rx, self.ry, self.ryaw)
        if self._avoiding:
            d_head = d_f             # 绕障时车头已偏向侧方, 看正前方即可
        else:
            d_head = min(d_f, _dpath)
'''
src = src.replace(OLD3, NEW3, 1)

io.open(DST, 'w', encoding='utf-8', newline='\n').write(src)
print('[OK] 写出 %s (%d 行)' % (DST, len(src.split('\n'))))
