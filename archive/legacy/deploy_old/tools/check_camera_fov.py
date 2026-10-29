#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从运行中的 Gazebo 反推 4 个 RGB-D 相机的真实视场角, 校核:
   (1) 4 个视野是否完整覆盖 360°
   (2) 相邻视野的重叠角是否约为 10°

做法: 订阅 /depth_<dir>/camera_info, 由内参矩阵 K 的 fx 反推水平视场角
      hfov = 2 * atan(width / (2*fx))
      vfov = 2 * atan(height / (2*fy))
安装方位取自 person_vision.py 的 CAMS(与 work_area.sdf 的 <pose> yaw 一致)。
"""
import math
import sys
import time

import gz.transport13 as gz
from gz.msgs10.camera_info_pb2 import CameraInfo

# (方向名, 安装方位角[度]); 与 SDF <pose> 的 yaw 一一对应
CAMS = [('front', 0.0), ('left', 90.0), ('right', -90.0), ('back', 180.0)]
INFO = {}


def cb(name):
    def _f(msg):
        if name not in INFO and msg.width > 0:
            k = msg.intrinsics.k          # 3x3 行主序: [fx,0,cx, 0,fy,cy, 0,0,1]
            INFO[name] = (msg.width, msg.height, float(k[0]), float(k[4]))
    return _f


def wrap(a):
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a


def main():
    node = gz.Node()
    for name, _ in CAMS:
        node.subscribe(CameraInfo, '/depth_%s/camera_info' % name, cb(name))
    t0 = time.time()
    while len(INFO) < 4 and time.time() - t0 < 12.0:
        time.sleep(0.2)
    if not INFO:
        print('没收到 camera_info —— Gazebo 是否在运行?')
        sys.exit(1)

    print('=' * 76)
    print('%-6s %6s %6s %9s %9s %9s' %
          ('相机', '宽', '高', 'fx', '水平FOV°', '垂直FOV°'))
    print('-' * 76)
    fov = {}
    for name, ang in CAMS:
        if name not in INFO:
            print('%-6s  <无数据>' % name)
            continue
        w, h, fx, fy = INFO[name]
        hf = 2.0 * math.degrees(math.atan(w / (2.0 * fx)))
        vf = 2.0 * math.degrees(math.atan(h / (2.0 * fy)))
        fov[name] = hf
        print('%-6s %6d %6d %9.2f %9.2f %9.2f' % (name, w, h, fx, hf, vf))
    print('=' * 76)
    if len(fov) < 4:
        sys.exit(1)

    # ---- 覆盖与重叠校核 ----
    # 每个相机的覆盖区间 [c-h/2, c+h/2], 归一化到 [-180,180)
    segs = []
    for name, ang in CAMS:
        half = fov[name] / 2.0
        segs.append((name, ang - half, ang + half, ang))

    print('\n覆盖区间(度, 归一化到 [-180,180]):')
    for name, lo, hi, ang in segs:
        print('  %-6s 安装 %+7.1f°  覆盖 [%+7.1f, %+7.1f]  (宽 %.2f°)'
              % (name, wrap(ang), wrap(lo), wrap(hi), hi - lo))

    # 相邻重叠: 相邻两相机覆盖区间的交集长度
    print('\n相邻重叠(4 对, 环绕一圈):')
    order = ['front', 'left', 'back', 'right']      # 逆时针顺序
    fmap = dict((n, (lo, hi)) for n, lo, hi, _ in segs)
    worst = 1e9
    for i in range(4):
        a, b = order[i], order[(i + 1) % 4]
        alo, ahi = fmap[a]
        blo, bhi = fmap[b]
        # 把 b 的区间平移到与 a 最接近的 360° 邻域
        best_ov = None
        for k in (-1, 0, 1):
            lo2, hi2 = blo + 360.0 * k, bhi + 360.0 * k
            ov = min(ahi, hi2) - max(alo, lo2)
            if best_ov is None or ov > best_ov:
                best_ov = ov
        print('  %-6s ↔ %-6s  重叠 %+6.2f°' % (a, b, best_ov))
        worst = min(worst, best_ov)

    # 覆盖并集: 按 0.05° 采样统计
    step = 0.05
    covered = 0
    total = int(round(360.0 / step))
    for i in range(total):
        ang = -180.0 + i * step
        for name, lo, hi, _ in segs:
            for k in (-1, 0, 1):
                if lo + 360.0 * k <= ang <= hi + 360.0 * k:
                    covered += 1
                    break
            else:
                continue
            break
    pct = 100.0 * covered / total
    print('\n360° 覆盖: %.2f%%  (%d/%d 采样点)' % (pct, covered, total))
    print('最小相邻重叠: %.2f°' % worst)
    ok = pct >= 99.99 and 8.0 <= worst <= 12.0
    print('\n结论: %s' % (
        '满足要求 —— 完整覆盖 360°, 相邻重叠约 10°' if ok
        else '不满足要求 —— 请检查 horizontal_fov 与安装方位'))
    return 0 if ok else 2


if __name__ == '__main__':
    sys.exit(main())
