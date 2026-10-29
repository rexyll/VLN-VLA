#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""side_opt.py —— 掌侧近距相机(P100)的**离线**朝向筛选, 不用起仿真。

思路: 构图好坏 90% 是"关键点投到画面的哪儿", 而投影只需要 FK 的掌系几何 ——
所以先在这里把候选朝向扫一遍(目标点居中? 四指/拇指在画面内?), 挑一个再进
仿真验证遮挡。P99 的教训是"靠看图猜朝向"要重启十几次世界, 而每次 60s。

坐标: 全部在**掌坐标系**(x=掌宽向、y=掌心法向=抓握面、z=手指延伸向的反向,
即手指朝 -z 伸出)。
"""
import math
import sys

import numpy as np

sys.path.insert(0, 'vla_ws/src/vla_follow')
from vla_follow import arm_kin as K        # noqa: E402

W, H, HFOV = 192, 144, 1.30


def rpy_from_R(R):
    """R = Rz(y)Ry(p)Rx(r) 的逆解(gz 的 pose 顺序)。"""
    p = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    y = math.atan2(R[1, 0], R[0, 0])
    r = math.atan2(R[2, 1], R[2, 2])
    return r, p, y


def build(cam, target, up):
    """光轴指向 target、画面"上"尽量贴 up → 3x3 (列 = 相机 x/y/z)。"""
    cam = np.array(cam, float)
    f = np.array(target, float) - cam
    f /= np.linalg.norm(f)
    u = np.array(up, float)
    u = u - f * (u @ f)
    if np.linalg.norm(u) < 1e-6:
        return None
    u /= np.linalg.norm(u)
    right = np.cross(u, f)        # 右手系: x × y = z ⇒ y = z × x
    return np.stack([f, right, u], axis=1)


def proj(R, cam, p):
    d = np.array(p, float) - np.array(cam, float)
    xc, yc, zc = d @ R[:, 0], d @ R[:, 1], d @ R[:, 2]
    if xc <= 0.02:
        return None, None, xc
    fx = (W / 2.0) / math.tan(HFOV / 2.0)
    return W / 2.0 - fx * yc / xc, H / 2.0 - fx * zc / xc, xc


def main():
    tips = {'tip%d' % k: (K.FINGER_X[k], 0.0,
                          K.FINGER_BASE_Z - K.PROX_LEN - K.DIST_LEN)
            for k in range(4)}
    tips['palm_far'] = (0.0, 0.0, K.FINGER_BASE_Z)
    obj = (0.0, 0.030, -0.095)      # 抓稳时物体中心(掌系)
    for side in ('R', 'L'):
        sx = -1.0 if side == 'R' else 1.0
        cam = (sx * 0.046, 0.0, -0.036)
        tgt = (0.0, 0.028, -0.095)
        print('==== side %s  cam=%s  target=%s' % (side, cam, tgt))
        for up in ((0, 0, 1), (0, 0, -1), (1, 0, 0), (-1, 0, 0),
                   (0, 1, 0), (0, -1, 0)):
            R = build(cam, tgt, up)
            if R is None:
                continue
            rpy = rpy_from_R(R)
            ou, ov, od = proj(R, cam, obj)
            band = []
            for k, p in sorted(tips.items()):
                u, v, d = proj(R, cam, p)
                band.append('%s(%s,%s)' % (k[-1] if k.startswith('tip') else 'P',
                                           '-' if u is None else '%.0f' % u,
                                           '-' if v is None else '%.0f' % v))
            inside = sum(1 for p in tips.values()
                         if (lambda r: r[0] is not None and 0 <= r[0] < W
                             and 0 <= r[1] < H)(proj(R, cam, p)))
            print('  up=%-8s rpy=(%.2f,%.2f,%.2f)  obj=(%.0f,%.0f)  d=%.3f  '
                  'in=%d/5  %s'
                  % (up, *rpy, ou or -1, ov or -1, od, inside, ' '.join(band)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
