#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""diag_wrist_proj.py —— 腕部相机**外参**标定: 把 FK 算出的指尖投到图像上,
再和深度图里"最近的那块"对一下, 判定相机局部位姿的轴向约定到底是哪一套。

为什么需要它
============
`/wrist_cam_*` 挂在掌 link 里, 局部位姿写的是 `<pose> 0 -0.040 -0.040 0 1.4202
1.5708 </pose>`。这里有两个约定是"猜"出来的:
  ① gz 相机光轴是传感器系 **+x**(不是 -z);
  ② 图像 u 向右对应 -y、v 向下对应 -z(相机系 +y 朝左、+z 朝上)。
猜错的后果是"画面里能看到东西, 但根本不知道看的是哪", 光看小图分不出来。
本探针把**几何上确定知道位置的东西**(四个指尖, 由实测关节角 FK 得到)投进
图像, 再取深度图里最近像素的质心 —— 两者重合就说明整条外参链是对的。

用法: python3 tests/diag/diag_wrist_proj.py [R|L]
"""
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, 'vla_ws', 'src', 'vla_follow'))

import gz.transport13 as gz_transport                      # noqa: E402
import gz.msgs10.image_pb2 as image_pb2                    # noqa: E402
import rclpy                                               # noqa: E402

from vla_follow import arm_kin as K                        # noqa: E402
from vla_follow import drink_motion as M                   # noqa: E402
from vla_follow.drink_task import DrinkTask                # noqa: E402

SIDE = sys.argv[1] if len(sys.argv) > 1 else 'R'
CAM_T = np.array([0.0, -0.040, -0.040])     # 掌系
CAM_RPY = (0.0, 1.4202, 1.5708)            # (roll, pitch, yaw)
W, H, HFOV = 256, 192, 1.2


def rot(rpy):
    r, p, y = rpy
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)],
                   [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0],
                   [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0],
                   [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def grab_depth(node, topic, timeout=8.0):
    box = {}

    def cb(m):
        box.setdefault('m', m)
    node.subscribe(image_pb2.Image, topic, cb)
    t0 = time.time()
    while 'm' not in box and time.time() - t0 < timeout:
        time.sleep(0.05)
    m = box.get('m')
    if m is None:
        return None
    f = np.frombuffer(m.data, dtype=np.float32)
    if f.size != m.width * m.height:
        f = np.frombuffer(m.data, dtype=np.uint16).astype(np.float32) / 1000.0
    return f.reshape(m.height, m.width)


def main():
    rclpy.init()
    task = DrinkTask(observer_only=True)
    for _ in range(30):
        task.spin_for(0.1)
        if M.q_meas(task, SIDE) is not None:
            break
    q = M.q_meas(task, SIDE)
    if q is None:
        print('拿不到 %s 臂关节角' % SIDE)
        return 2
    T_palm = M.wrist(task, SIDE)          # 世界系掌位姿
    print('%s 臂关节角: %s' % (SIDE, np.round(q, 3)))
    print('掌世界位姿 t=(%.4f %.4f %.4f)' % tuple(T_palm[:3, 3]))

    # 指尖(palm 系): 四指远端中心
    tips = [(K.FINGER_X[k], 0.0, K.FINGER_BASE_Z - K.PROX_LEN - K.DIST_LEN)
            for k in range(4)]
    # 掌心 + 腕心 也测一下
    extra = {'palm_origin': (0, 0, 0), 'thumb_tip': (K.THUMB_X[SIDE],
             K.THUMB_BASE[1], K.THUMB_BASE[2] - K.PROX_LEN - K.DIST_LEN)}

    R_cam = rot(CAM_RPY)
    T_cam = np.eye(4)
    T_cam[:3, :3] = R_cam
    T_cam[:3, 3] = CAM_T
    T = T_palm @ T_cam                      # 相机在掌系下的位姿(与 SDF 一致)
    c = T[:3, 3]
    fwd, left, up = R_cam[:, 0], R_cam[:, 1], R_cam[:, 2]
    print('相机世界位置 = (%.4f %.4f %.4f)' % tuple(c))
    print('光轴(fwd) = (%+.3f %+.3f %+.3f)' % tuple(fwd))
    fx = (W / 2) / np.tan(HFOV / 2)
    fy = fx                                 # 方形像素
    print('fx=fy=%.1f px' % fx)

    def project(p_world):
        d = np.asarray(p_world) - c
        xc, yc, zc = d @ fwd, d @ left, d @ up
        if xc <= 0.02:
            return None, xc
        u = W / 2 - fx * yc / xc
        v = H / 2 - fy * zc / xc
        return (u, v), xc

    print('\n--- 预测投影(掌系点 -> 图像像素) ---')
    for k, t in enumerate(tips):
        p = T_palm[:3, :3] @ np.array(t) + T_palm[:3, 3]
        uv, d = project(p)
        print('  finger%d 世界(%.3f %.3f %.3f) -> %s  轴向距离 %.3fm'
              % (k, *p, '图外' if uv is None else '(%.0f, %.0f)' % uv, d))
    for name, t in extra.items():
        p = T_palm[:3, :3] @ np.array(t) + T_palm[:3, 3]
        uv, d = project(p)
        print('  %-12s -> %s  %.3fm'
              % (name, '图外' if uv is None else '(%.0f, %.0f)' % uv, d))

    dep = grab_depth(gz_transport.Node(), '/wrist_cam_%s/depth_image' % SIDE)
    if dep is None:
        print('\n深度图无帧')
        return 2
    valid = np.isfinite(dep) & (dep > 0.06) & (dep < 7.0)
    print('\n--- 深度图 ---')
    print('  有效像素 %.0f%%  最近 %.3fm 最远 %.3fm 中位 %.3fm'
          % (100 * valid.mean(), dep[valid].min(), dep[valid].max(),
             np.median(dep[valid])))
    yy, xx = np.nonzero(valid & (dep < dep[valid].min() + 0.03))
    print('  最近 3cm 内的像素: %d 个, 质心 (%.0f, %.0f)'
          % (len(xx), xx.mean(), yy.mean()))
    print('  最近像素的 8 邻域深度: %s'
          % np.round(dep[max(0, int(yy.mean()) - 1):int(yy.mean()) + 2,
                         max(0, int(xx.mean()) - 1):int(xx.mean()) + 2], 3))
    # 中心一行/一列的深度剖面, 方便看"看到的是不是一块平面"
    print('  中线纵向剖面 v=0,32,64,96,128,160: %s'
          % np.round(dep[[0, 32, 64, 96, 128, 160], W // 2], 3))
    return 0


if __name__ == '__main__':
    sys.exit(main())
