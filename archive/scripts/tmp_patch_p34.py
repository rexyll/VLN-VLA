#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_patch_p34.py —— P34: 修 person_vision 目标选择

问题(实测):
  画面里同时存在 ①人体(1.1m, area 2534) ②机器人自身近场伪影
  (0.11m, area 929)。原逻辑"先按面积选最大, 再做深度过滤":
    - 伪影面积大于人时(人较远/侧身), 伪影被选中 → 深度 0.11 < DEPTH_MIN
      → 直接 return det=False, 真正的人被完全忽略;
    - 伪影面积小于人时, 人正常检出。
  于是"有时检得到、有时检不到", 表现为机器人偶尔丢失目标、随机走位。

修复:
  C1  DEPTH_MIN 0.45 → 0.55 (人贴到 0.55m 以内本就该后退)
  C2  候选循环内先取深度: 深度无效的候选不参与竞争
  C3  几何一致性校验: 可见像素宽度 w 与深度 dist 必须自洽
      (人体可见宽度先验 W_REAL, 侧身 0.20 / 腿 0.34 / 躯干 0.52)
      d_geo = W_REAL*img_w/(2*w*tan(HFOV/2)), 要求 dist 落在 d_geo±W_TOL
"""
import io
import sys

P = 'vla_ws/src/vla_follow/vla_follow/person_vision.py'
s = io.open(P, encoding='utf-8').read()
if 'P34' in s:
    print('!! 已打过 P34')
    sys.exit(1)


def rep(old, new, tag):
    global s
    n = s.count(old)
    if n != 1:
        print('!! [%s] 锚点命中 %d 次' % (tag, n))
        sys.exit(1)
    s = s.replace(old, new, 1)
    print('   ok %s' % tag)


# ---- C1: 常量 ----
rep("""DEPTH_MIN = 0.45  # 过滤近端伪影与自视伪影(自身轮/后角成超大暗斑)""",
    """DEPTH_MIN = 0.55  # P34: 过滤近端伪影与自视伪影(原 0.45 仍会漏进伪影)
# P34: 人体"可见宽度"先验(米)与几何一致性容差
#   侧身 ~0.20 / 腿部 ~0.34 / 躯干 ~0.52, 取中值 0.36 + 55% 容差覆盖三者
W_REAL = 0.36
W_TOL = 0.55""",
    'C1')

# ---- C2 + C3: 候选选择 ----
OLD = """        best = None
        for c in cnts:
            a = cv2.contourArea(c)
            if a < AREA_MIN or a > AREA_MAX:
                continue
            x, y, w, h = cv2.boundingRect(c)
            asp = w / max(h, 1)
            if not (ASPECT_MIN <= asp <= ASPECT_MAX):
                continue
            if best is None or a > best[0]:
                best = (a, x, y, w, h)
        if best is None:
            return miss
        a, x, y, w, h = best
        u = x + w / 2.0
        y0 = y + int(h * 0.15)
        y1 = y + int(h * 0.60)
        sub = dep[y0:max(y1, y0 + 1), x:x + w]
        valid = sub[np.isfinite(sub) & (sub > DEPTH_MIN) & (sub < DEPTH_MAX)]
        if valid.size < 10:
            return {'det': False, 'dist': -1.0, 'bc': 0.0, 'u': int(u),
                    'area': int(a), 'bbox': (x, y, w, h)}
        dist = float(np.quantile(valid, 0.30))
        img_w = rgb.shape[1]
        # 图像 u 向右增; 方位角正方向=左 → 用(中心-u)
        bc = (img_w / 2.0 - u) * HFOV / img_w
        return {'det': True, 'dist': dist, 'bc': bc, 'u': int(u),
                'area': int(a), 'bbox': (x, y, w, h)}"""

NEW = """        best = None
        img_w = rgb.shape[1]
        tan_h = math.tan(HFOV / 2.0)
        for c in cnts:
            a = cv2.contourArea(c)
            if a < AREA_MIN or a > AREA_MAX:
                continue
            x, y, w, h = cv2.boundingRect(c)
            asp = w / max(h, 1)
            if not (ASPECT_MIN <= asp <= ASPECT_MAX):
                continue
            # P34: 先取深度 —— 深度无效的候选不参与竞争。
            # 原实现"先选最大面积、再做深度过滤", 一旦面积最大的候选是
            # 近场伪影, 它自己会被 DEPTH_MIN 过滤掉, 而真正的人已经被
            # 挤掉了, 整帧直接判未检出(实测导致目标周期性丢失)。
            y0 = y + int(h * 0.15)
            y1 = y + int(h * 0.60)
            sub = dep[y0:max(y1, y0 + 1), x:x + w]
            valid = sub[np.isfinite(sub) & (sub > DEPTH_MIN)
                        & (sub < DEPTH_MAX)]
            if valid.size < 10:
                continue
            dist = float(np.quantile(valid, 0.30))
            # P34: 几何一致性 —— 可见像素宽度与深度必须自洽。
            # 人体可见宽度 0.20~0.52m, 由 bbox 宽度反推距离 d_geo;
            # 若深度图给的距离与 d_geo 相差过大, 说明这个暗斑不是人
            # (典型是机体近场伪影、墙面暗纹、远处小色块)。
            d_geo = W_REAL * img_w / max(2.0 * w * tan_h, 1e-6)
            if (dist < d_geo * (1.0 - W_TOL)
                    or dist > d_geo * (1.0 + W_TOL)):
                continue
            if best is None or a > best[0]:
                best = (a, x, y, w, h, dist)
        if best is None:
            return miss
        a, x, y, w, h, dist = best
        u = x + w / 2.0
        # 图像 u 向右增; 方位角正方向=左 → 用(中心-u)
        bc = (img_w / 2.0 - u) * HFOV / img_w
        return {'det': True, 'dist': dist, 'bc': bc, 'u': int(u),
                'area': int(a), 'bbox': (x, y, w, h)}"""

rep(OLD, NEW, 'C2C3')

io.open(P, 'w', encoding='utf-8', newline='\n').write(s)
print('P34_APPLIED')
