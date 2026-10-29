#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""diag_wrist_grasp.py —— 预抓取位腕部画面与几何预测的对齐诊断。

把右臂摆到壶前方的预抓取位, 然后:
  ① 打印相机世界位姿、壶世界位姿、两者距离
  ② 把壶身圆柱面采样点投到图像, 打印命中率与逐个点的 (u, v, 预测深度)
  ③ 打印深度图里"有限深度"像素的包围盒与中位深度, 以及几个关键像素的实测值

用来判断"投影说得对但画面对不上"到底是外参、base 还是深度图哪一环。
"""
import math
import os
import sys
import time

import numpy as np
import rclpy

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, 'vla_ws', 'src', 'vla_follow'))

from vla_follow import arm_kin as K           # noqa: E402
from vla_follow import drink_poses as P      # noqa: E402
from vla_follow import scene_layout as L     # noqa: E402
from vla_follow.drink_task import DrinkTask  # noqa: E402

sys.path.insert(0, os.path.join(ROOT, 'tests'))
from test_p99 import GzCam, T_of, move_to, q_meas   # noqa: E402

SIDE = 'R'


def main():
    rclpy.init()
    node = DrinkTask()
    for _ in range(40):
        node.spin_for(0.1)
        if q_meas(node, SIDE) is not None and 'kettle' in node.links:
            break

    T_place = K.frame(K.rot_z(0.0), (L.SERVE_POSE[0], L.SERVE_POSE[1], 0.0))
    node.carry.place('robot', T_place)
    node.spin_for(1.0)
    node.sync_base_from_truth()
    node.home(verbose=False)
    Tk0 = T_of(node, 'kettle')
    T_grasp = P.kettle_grasp_palm(Tk0)
    T_pre = T_grasp @ K.trans((0.0, 0.0, 0.10))
    node.hand[SIDE].move(K.grip_values(SIDE, K.GRIP_OPEN))
    ok, res = move_to(node, T_pre)
    node.spin_for(0.4)
    q = q_meas(node, SIDE)
    Tp = K.fk_world(SIDE, q, node.base)
    T_cam = K.wrist_cam_world(SIDE, q, node.base)
    print('到位 %s 残差 %.1f mm' % (ok, res))
    print('base   t=(%.3f %.3f) yaw=%.1f°' % (node.base[0], node.base[1],
                                              math.degrees(node.base[2])))
    print('掌     t=(%.3f %.3f %.3f)' % tuple(Tp[:3, 3]))
    print('相机   t=(%.3f %.3f %.3f)' % tuple(T_cam[:3, 3]))
    print('壶     t=(%.3f %.3f %.3f)' % tuple(Tk0[:3, 3]))
    print('壶把点 t=%s' % np.round(Tk0[:3, 3] + Tk0[:3, :3]
                                    @ np.array(L.KETTLE_GRIP_LOCAL), 3))
    print('相机→壶心 %.3f m' % np.linalg.norm(T_cam[:3, 3] - Tk0[:3, 3]))
    for nm, T in (('掌', Tp), ('相机', T_cam)):
        ax = {k: T[:3, i] for i, k in enumerate(('+x', '+y', '+z'))}
        print('%s 轴: +x=(%+.2f %+.2f %+.2f) +y=(%+.2f %+.2f %+.2f) '
              '+z=(%+.2f %+.2f %+.2f)'
              % (nm, *ax['+x'], *ax['+y'], *ax['+z']))

    print('\n--- 指尖投影(应落在画面上沿) ---')
    dep0 = None
    cam0 = GzCam('R')
    for _ in range(20):
        node.spin_for(0.2)
        if cam0.wait(0.2):
            break
    dep0, _m = cam0.depth_array()
    for k in range(4):
        pl = np.array([K.FINGER_X[k], 0.0,
                       K.FINGER_BASE_Z - K.PROX_LEN - K.DIST_LEN, 1.0])
        pw = Tp @ pl
        u, v, d = K.wrist_cam_project(T_cam, pw[:3])
        dm = (float(dep0[int(np.clip(v, 0, 191)), int(np.clip(u, 0, 255))])
              if u is not None else float('nan'))
        print('  finger%d -> (%5.0f, %5.0f) 预测 %.3f 实测 %.3f'
              % (k, u or 0, v or 0, d, dm))

    print('\n--- 壶身面采样投影 ---')
    print('  (u, v) 预测深度 实测深度 命中')
    dep = None
    cam = GzCam('R')
    for _ in range(20):
        node.spin_for(0.2)
        if cam.wait(0.2):
            break
    dep, _m = cam.depth_array()
    inb = 0
    for ang in np.linspace(0, 2 * math.pi, 8, endpoint=False):
        for zz in (-0.06, 0.0, 0.06):
            pl = np.array([L.KETTLE_BODY_R * math.cos(ang),
                           L.KETTLE_BODY_R * math.sin(ang), zz, 1.0])
            pw = Tk0 @ pl
            u, v, d = K.wrist_cam_project(T_cam, pw[:3])
            if u is None:
                print('  ang=%5.0f° z=%+.2f  在相机背后 (%.3f)' % (
                    math.degrees(ang), zz, d))
                continue
            inside = 0 <= u < 256 and 0 <= v < 192
            dm = float(dep[int(np.clip(v, 0, 191)), int(np.clip(u, 0, 255))])
            inb += 1 if inside else 0
            print('  ang=%5.0f° z=%+.2f  (%5.0f, %5.0f)%s  %.3f  %7.3f  %s'
                  % (math.degrees(ang), zz, u, v, '' if inside else ' 图外',
                     d, dm, 'OK' if (inside and np.isfinite(dm)
                                     and abs(dm - d) < 0.03) else ''))
    print('  画幅内 %d 点' % inb)

    print('\n--- 深度图 ---')
    val = np.isfinite(dep) & (dep > 0.05) & (dep < 7.5)
    print('  有效 %.0f%%  范围 %.3f~%.3f  中位 %.3f'
          % (100 * val.mean(), dep[val].min(), dep[val].max(),
             np.median(dep[val])))
    yy, xx = np.nonzero(val)
    print('  有效像素包围盒 u %d~%d  v %d~%d' % (xx.min(), xx.max(),
                                                 yy.min(), yy.max()))
    for v in (0, 20, 40, 60, 80, 100, 140, 180):
        print('  v=%3d: %s' % (v, np.round(dep[v, ::32], 3)))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
