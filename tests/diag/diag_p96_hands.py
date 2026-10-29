#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""diag_p96_hands.py —— 人的两条手臂真的各自能转吗? 转轴在哪? 幅度对不对?

P96 把 `person_arms` 拆成了"肩架 + 左右两条手臂 link"。拆完必须验三件事,
不然上层发得再对也是在量一个不存在的手:

  ① 两条关节真的存在且被驱动(不是"话题在但关节不动" —— §6.4 #18 那种
     插件名字写错导致静默跳过);
  ② 转轴落在**肩点**上。摆在 `<link><pose relative_to="person_arms">` 里
     而不是 `<joint><pose>`, 是因为 gz-sim 8 会忽略后者(#38); 但"忽略"
     有可能变成"转轴落在父 link 原点", 画面上手臂就会绕着腰甩;
  ③ 幅度符合几何: 手心在 person_arms 系的 y 应该 = ±(0.155 + 0.20·sinθ
     − 0.070·cosθ)。两侧还要**镜像对称**, 否则左右发反号这件事就错了。

用法: bash tools/start_scene.sh; python3 tests/diag/diag_p96_hands.py
"""
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, 'vla_ws', 'src', 'vla_follow'))

import numpy as np  # noqa: E402
import rclpy  # noqa: E402

from vla_follow import drink_motion as M      # noqa: E402
from vla_follow import scene_layout as L      # noqa: E402
from vla_follow.drink_task import DrinkTask   # noqa: E402

# 几何预测: 手心在 person_arms 系的 y(= 上手臂 link 原点 y + 手在 link 系里的 y)
SH_Y = L.PERSON_SHOULDER_Y
DXY = (L.PERSON_HAND_IN_ARML[0], L.PERSON_HAND_IN_ARML[1])


def pred_y(theta):
    """手心在 person_arms 系的 y(左臂, +θ)。"""
    return SH_Y + DXY[0] * math.sin(theta) + DXY[1] * math.cos(theta)


def probe(node, theta, tag):
    node.bus.set(['person_arm_l_joint', 'person_arm_r_joint'],
                 [theta, -theta])
    node.spin_for(2.0)
    Tp = M.object_pose(node, 'person')
    Tl, Tr = M.person_hand_poses(node, Tp)
    if Tl is None or Tr is None:
        print('  %-14s 拿不到手 link(links=%d)' % (tag, len(node.links)))
        return None
    # 把手心换算回 person_arms 系, 才能和几何预测直接比(与人朝哪无关)
    Ta = M.person_hand_world(node, Tp, link='person_arms', local_off=None)
    if Ta is None:
        print('  %-14s 拿不到 person_arms' % tag)
        return None
    inv = np.linalg.inv(Ta)
    pl = (inv @ np.array([Tl[0, 3], Tl[1, 3], Tl[2, 3], 1.0]))[:3]
    pr = (inv @ np.array([Tr[0, 3], Tr[1, 3], Tr[2, 3], 1.0]))[:3]
    gap = float(np.linalg.norm(Tl[:3, 3] - Tr[:3, 3])) * 1000.0
    ql = node.js.get('person_arm_l_joint')
    qr = node.js.get('person_arm_r_joint')
    print('  %-14s θ=%+.3f  手心(arms系) 左(%+.4f,%+.4f,%+.4f) 右(%+.4f,'
          '%+.4f,%+.4f)  间距 %6.1fmm'
          % (tag, theta, pl[0], pl[1], pl[2], pr[0], pr[1], pr[2], gap))
    # 注意: 人的关节**不在** /drink/joint_state 里 —— 那条是 robot 模型的
    # joint_state_publisher, 而 bridge 只桥了这一条。所以"手有没有动"只能
    # 量 link 位姿, 读不回关节角(ql/qr 恒为 None)。这反倒是更硬的判据:
    # 量的是手真的走到哪, 不是"我发了什么"。
    print('  %-14s 关节回读 左 %s / 右 %s(预期 None: 人的关节不在 '
          'joint_state 里)  预测 y 左 %+.4f 右 %+.4f '
          '(实测 %+.4f / %+.4f, 差 %.1f / %.1f mm)'
          % ('', ql, qr,
             pred_y(theta), -pred_y(theta), pl[1], pr[1],
             (pl[1] - pred_y(theta)) * 1000.0,
             (pr[1] + pred_y(theta)) * 1000.0))
    return {'theta': theta, 'gap_mm': gap,
            'yl': pl[1], 'yr': pr[1], 'xl': pl[0], 'xr': pr[0],
            'ql': ql, 'qr': qr}


def main():
    rclpy.init()
    node = DrinkTask()
    t0 = 0.0
    while t0 < 10.0 and len(node.js) < 34:
        node.spin_for(0.2)
        t0 += 0.2
    print('joint_state=%d  links=%d' % (len(node.js), len(node.links)))
    for n in L.PERSON_ARM_LINKS:
        print('  link %-14s %s' % (n, '在' if n in node.links else '不在'))

    print('=' * 74)
    print('[A] 摆臂几何: θ 从 0 扫到 0.60rad')
    print('=' * 74)
    rs = []
    for th in (0.0, 0.10, 0.20, 0.30, 0.42, 0.60):
        r = probe(node, th, 'θ=%.2f' % th)
        if r:
            rs.append(r)

    if rs:
        print('-' * 74)
        print('θ       间距mm   手心 y(左/右)      对称残差mm  行程mm')
        prev = None
        for r in rs:
            asym = abs(r['yl'] + r['yr']) * 1000.0
            travel = 0.0 if prev is None else abs(r['yl'] - prev['yl']) * 1000.0
            print('%.2f   %6.1f   %+.4f/%+.4f   %6.2f      %6.1f'
                  % (r['theta'], r['gap_mm'], r['yl'], r['yr'], asym, travel))
            prev = r
        # 关键判据: 张开时手心必须离得开杯壁(杯 R 0.040 + 手掌 R 0.052)
        need = L.CUP_GRIP_R
        for r in rs:
            print('  θ=%.2f 手到杯轴(若杯在正中) %.1fmm  相切值 %.1fmm  %s'
                  % (r['theta'], abs(r['yl']) * 1000.0, need * 1000.0,
                     '合拢=贴住' if abs(abs(r['yl']) - need) < 0.004
                     else ('张开=够宽' if abs(r['yl']) > need + 0.02
                           else '——')))

    print('=' * 74)
    print('[B] 肩俯仰 + 双手张开叠加(喝水时手是合着的, 这里看耦合有没有把'
          '手甩出身体)')
    print('=' * 74)
    node.bus.set(['person_arm_l_joint', 'person_arm_r_joint'],
                 [L.PERSON_HAND_OPEN, -L.PERSON_HAND_OPEN])
    for q in (0.0, -0.60, -1.00):
        node.bus.set(['person_arm_joint'], [q])
        node.spin_for(2.0)
        Tp = M.object_pose(node, 'person')
        Tl, Tr = M.person_hand_poses(node, Tp)
        if Tl is None:
            print('  肩 %.2f 拿不到手' % q)
            continue
        # 手相对人体中心的水平距离 / 高度
        dl = math.hypot(Tl[0, 3] - Tp[0, 3], Tl[1, 3] - Tp[1, 3])
        dzl = Tl[2, 3] - Tp[2, 3]
        print('  肩 %+.2f  手离人体 %.3fm  高于人体原点 %.3fm  '
              '手心间距 %.1fmm'
              % (q, dl, dzl,
                 float(np.linalg.norm(Tl[:3, 3] - Tr[:3, 3])) * 1000.0))

    node.bus.set(['person_arm_joint'], [0.0])
    node.bus.set(['person_arm_l_joint', 'person_arm_r_joint'],
                 [L.PERSON_HAND_CLOSE, -L.PERSON_HAND_CLOSE])
    node.spin_for(2.0)
    print('  回到合拢: %.1fmm' % (M.hand_gap_mm(node) or -1.0))

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
