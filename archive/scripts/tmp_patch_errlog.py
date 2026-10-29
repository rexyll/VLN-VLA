#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_patch_errlog.py —— 修正 depth_percept 的误差日志判据

原实现:
    for n, rel in DIRS:                       # 只有 4 个 90° 方位
        truth = ray_min_dist(rx, ry, ryaw + rel)      # 单条中心射线
        err = abs(out[n] - truth)                     # 比的是"扇区最小值"
比较对象粒度不一致: out['front'] 是 5 个扇区(±50°)里的**最小值**, 而真值
只沿正前方 0° 一条射线。只要斜前方有立柱(ob5 常驻 -35°), 最小值就落在斜
向, 于是必然报"front 0.7 / 真 2.7 (Δ2.04)"这种假告警 —— 实测 ob5 表面
理论 0.67m, 感知 0.70m, **误差只有 0.03m**。

改为逐扇区对齐比较: 扇区 k 的中心方位 = 相机朝向 + (2-k)*20°,
用同方位的射线真值去比该扇区的 5% 分位数。日志改为每台相机输出
最大偏差 + 该偏差所在扇区, 再加总超阈计数。
"""
import shutil
import sys

P = '/home/earl/workspace/VLA/vla_ws/src/vla_follow/vla_follow/depth_percept.py'
shutil.copy(P, P + '.bak_errlog')
s = open(P, encoding='utf-8').read()

OLD = """            extra = [(px, py, PERSON_R)]
            parts, bad = [], 0
            for n, rel in DIRS:
                if out[n] and out[n] > 0:
                    truth = ray_min_dist(rx, ry, ryaw + rel, extra)
                    err = abs(out[n] - min(truth, 11.5))
                    parts.append(f'{n}:{out[n]:.1f}/真{truth:.1f}(Δ{err:.2f})')
                    self._err_n += 1
                    if err > 0.3:
                        bad += 1
                        self._err_bad += 1
            if parts:
                self.get_logger().info(
                    'PERCEPT 误差对比 ' + ' '.join(parts) +
                    (f' | 超阈(>0.3m): {bad}向' if bad else ' | 全部≤0.3m'))"""

NEW = """            # P27: 逐扇区对齐比较。原来拿"5 扇区最小值"去比"单条中心射线
            # 真值", 只要斜前方有立柱(ob5 常驻 -35°), 最小值就落在斜向,
            # 必然假报 "front 0.7/真 2.7"。实测 ob5 表面理论 0.67m、感知
            # 0.70m, 真实误差仅 0.03m。
            extra = [(px, py, PERSON_R)]
            parts, bad, worst_all = [], 0, 0.0
            short = {'front': '前', 'left': '左', 'right': '右', 'back': '后'}
            for n, rel in DIRS:
                sec = self.sect.get(n) or []
                worst, worst_k = -1.0, -1
                for k in range(len(sec)):
                    d = sec[k]
                    if not (d and d > 0):
                        continue
                    # 扇区中心方位: section 0 在相机左侧 +40°, 依次 -20°
                    a = ryaw + rel + math.radians((2 - k) * 20.0)
                    truth = min(ray_min_dist(rx, ry, a, extra), 11.5)
                    err = abs(d - min(truth, 11.5))
                    self._err_n += 1
                    if err > 0.3:
                        bad += 1
                        self._err_bad += 1
                    if err > worst:
                        worst, worst_k = err, k
                if worst_k >= 0:
                    off = (2 - worst_k) * 20
                    parts.append(f'{short.get(n, n)}Δ{worst:.2f}({off:+d}°)')
                    worst_all = max(worst_all, worst)
            if parts:
                self.get_logger().info(
                    'PERCEPT 扇区精度 ' + ' '.join(parts) +
                    f' | 最大Δ{worst_all:.2f}m'
                    + (f' 超阈(>0.3m) {bad}扇区' if bad else ' 全部≤0.3m'))"""

if s.count(OLD) != 1:
    print('!! 误差日志锚点异常')
    sys.exit(1)
s = s.replace(OLD, NEW, 1)

open(P, 'w', encoding='utf-8').write(s)
print('PATCH_OK')
print('  含 扇区精度:', s.count('PERCEPT 扇区精度'))
