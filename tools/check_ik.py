#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""check_ik.py —— 喝水任务关键位形的离线逆解体检 (P84)

在**不进仿真**的前提下, 把 drink_poses 里每一个关键位形都用生产同一套
KDL 逆解算一遍, 输出: 位姿误差 / 关节限位余量 / 用了哪个求解器。
位形不可达就先改 scene_layout 的布局数字, 不要等进仿真才发现。

用法:
    python3 tools/check_ik.py            # 体检
    python3 tools/check_ik.py sweep      # 附加: 倒水倾角/壶嘴高度的可达性扫描
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(os.path.dirname(HERE), 'vla_ws', 'src', 'vla_follow')
sys.path.insert(0, SRC)

import numpy as np  # noqa: E402

from vla_follow import arm_kin as K          # noqa: E402
from vla_follow import drink_poses as P      # noqa: E402
from vla_follow import scene_layout as L     # noqa: E402

OK_M = 0.010     # 位置残差上限 10mm
OK_R = 0.030     # 姿态残差上限 1.7°


def fmt(T):
    t = T[:3, 3]
    return '(%+.3f %+.3f %+.3f)' % (t[0], t[1], t[2])


def main():
    print('PyKDL 可用 =', K.HAVE_KDL)
    print('服务位 = %s  人的接水位 = (%.3f, %.3f) 距离 %.3f m'
          % (L.SERVE_POSE, L.PERSON_SPOT[0], L.PERSON_SPOT[1],
             L.PERSON_SPOT_R))
    print('-' * 100)
    bad = 0
    rows = P.report()
    for side, tag, ep, er, marg, how, q in rows:
        flag = ''
        if ep > OK_M or er > OK_R:
            flag = '  <<< 不可达'
            bad += 1
        elif math.degrees(marg) < 3.0:
            flag = '  << 贴限位'
        print('%-2s %-18s 位置%6.1fmm 姿态%5.1f° 余量%6.1f° %-10s%s'
              % (side, tag, ep * 1000.0, math.degrees(er),
                 math.degrees(marg), how, flag))
    print('-' * 100)
    print('不可达位形: %d 个 / 共 %d 个' % (bad, len(rows)))
    if len(sys.argv) > 1 and sys.argv[1] == 'sweep':
        sweep()
    return bad


def _cases():
    Tk = P.kettle_on_table()
    Tc = P.cup_on_table()
    Tcp = P.cup_for_pour()
    Tkp, _tip = P.kettle_for_pour(Tcp)
    bx, by, byaw = L.SERVE_POSE
    out = [
        ('R', '抓壶(桌面)', P.kettle_grasp_palm(Tk)),
        ('L', '抓杯(桌面)', P.cup_grasp_palm(Tc, 'L')),
        ('L', '举杯(倒水)', P.cup_grasp_palm(Tcp, 'L')),
        ('R', '持壶(倒水)', P.kettle_grasp_palm(Tkp)),
        ('R', '放壶(桌面)', P.kettle_grasp_palm(Tk)),
        ('L', '双手捧杯(胸前)', P.cup_cradle_palm(P.carry_cup(bx, by, byaw), 'L')),
        ('R', '双手捧杯(胸前)', P.cup_cradle_palm(P.carry_cup(bx, by, byaw), 'R')),
        ('L', '双手递杯(前伸)', P.cup_cradle_palm(P.present_cup(bx, by, byaw), 'L')),
        ('R', '双手递杯(前伸)', P.cup_cradle_palm(P.present_cup(bx, by, byaw), 'R')),
        ('L', '递杯给人', P.cup_cradle_palm(
            P.present_cup(bx, by, L.HANDOVER_YAW), 'L')),
        ('R', '递杯给人', P.cup_cradle_palm(
            P.present_cup(bx, by, L.HANDOVER_YAW), 'R')),
    ]
    return out


def sweep():
    """倒水位形参数扫描: 倾角 × 壶嘴高度 × 杯心高度"""
    print('\n倒水位形扫描 (tilt, cup_z, gap) -> 右手位置残差 mm')
    Tcp0 = P.cup_for_pour()
    for cup_z in (0.80, 0.84, 0.88, 0.92):
        line = []
        for tilt_deg in (55, 62, 68, 75):
            Tc = P.frame(P.cup_on_table()[:3, :3],
                         (L.CUP_XY[0], L.CUP_XY[1], cup_z))
            Tkp, _ = P.kettle_for_pour(Tc, math.radians(tilt_deg))
            q, e, how = K.ik_best('R', P.kettle_grasp_palm(Tkp), K.Q_REST)
            ep = float(np.linalg.norm(K.pose_err(K.fk('R', q), P.kettle_grasp_palm(Tkp))[:3]))
            line.append('%5.1fmm' % (ep * 1000))
        print('  cup_z=%.2f: %s' % (cup_z, '  '.join(line)))
    del Tcp0


if __name__ == '__main__':
    sys.exit(0 if main() == 0 else 1)
