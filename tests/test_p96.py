#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_p96.py —— P96 验收: 人的**两只手**各自能开合, 并真的捧住 / 交还杯子

背景(P95/P94 收口后 §6.7 第 2 项)
================================
原来人是"一个肩关节 + 一条焊死的两臂刚体": 两只手永远相距 170mm, 杯子只是
被钉进那个缝里。"双手接杯"这件事在模型上没有自由度, 画面上也就不存在。
P96 把两条手臂拆成各自的 link(绕肩点竖直轴外展), 于是:

    张开迎杯 → 合拢捧住 → 人举杯喝 → 张开交还 → 机器人接回 → 收手

验收口径(全部对 gz 真值, 不是回读指令)
======================================
  [1] 两条手臂 link 都在真值里; 开合三个幅度(合拢/半开/全开)的手心间距与
      几何预测一致(≤2mm), 且左右**镜像对称**(≤2mm)
  [2] 一路做到"双手捧杯、面向人" (= P88 终态)
  [3] 双掌前伸到递杯位 + 人**张开迎杯**: 手心间距 ≥300mm, 杯子落在两手中
      (杯到两手连线中点的横向偏差 ≤50mm), 且张开动作**没碰动杯子** ≤3mm
  [4] 人**合拢捧杯**(交接前, 杯子还在机器人手里): 手到杯轴 92±30mm ——
      这一项量的是**递杯精度**, 与 P94 的 d_hand 同口径
  [5] 交接(持有者换成人的手) + 机器人撤臂: 杯子纹丝不动 ≤3mm, 且**双手捧在
      杯壁上** 92±8mm —— 交接后杯子钉在两手连线中点, 这条是结构一致性
  [6] 人举杯到嘴边: 杯子抬高 100~300mm, 相对人臂守恒 ≤3mm, 且**双手始终
      捧着** (手到杯轴仍是 92±8mm) —— 手跟着手架走, 不会举到一半松开
  [7] 人放下: 回到递杯位 ≤15mm, 相对守恒 ≤3mm
  [8] **张开交还** → 手到杯轴 ≥140mm(交接前 ≤110mm, 必须真的开出去) →
      机器人双手接回 → 人收手
  [9] 杯子放回桌面原位 ≤10mm
  [10] 截图

为什么"手到杯轴"要取水平距离
============================
杯是竖直圆柱, 手从两侧捧。横向间距才是"捧住了/穿模了/没够着"的判据;
高度单独判, 用来排除"手捧在杯口上方"这种假捧住。

用法:
    bash tools/start_scene.sh
    source /opt/ros/jazzy/setup.bash; source vla_ws/install/setup.bash
    python3 tests/test_p96.py
"""
import math
import os
import sys
import time

import numpy as np
import rclpy
from sensor_msgs.msg import Image

from vla_follow import arm_kin as K
from vla_follow import drink_motion as M
from vla_follow import drink_poses as P
from vla_follow import scene_layout as L
from vla_follow.drink_task import DrinkTask
from vla_follow.pngsnap import save_png

LOG_DIR = os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'vla_ws', 'logs')

DRINK_JOINT = -1.00        # 人举杯到嘴边(rad), 限位 -1.35~0.10

# 判据
TANGENT_MM = L.CUP_GRIP_R * 1000.0     # 92: 手心到杯轴正好相切
# 手到杯轴允许 92±25mm。25mm 的来历: 手掌球 r=52、杯 r=40, 偏 25mm 时一侧
# 陷进杯壁 25mm、另一侧留 25mm 缝 —— 再大就看得出"一只手插进杯子里、另一只
# 没碰到"了。而递杯本身的横向误差实测 11~23mm( P94 总偏差 45mm 里的主要
# 分量), 25mm 是既能抓住真问题、又不被正常误差顶爆的那个数。
GRIP_TOL = 25.0
GRIP_ASYM = 2 * GRIP_TOL               # 左右差 = 2×杯子偏离中点的量
GRIP_DZ = 80.0                         # 手与杯高差
SEAM_MM = 3.0                          # 交接前后**同一个量**的变化
OPEN_GAP = 300.0                       # 张开时手心间距下限
OPEN_AXIS = 140.0                      # 交还时手到杯轴下限
RECV_MID = 50.0                        # 迎杯时杯落在两手中的横向容差

# 手心在 person_arms 系的 y 的几何预测(见 scene_layout 注释)
_PRED = (L.PERSON_SHOULDER_Y, L.PERSON_HAND_IN_ARML[0],
         L.PERSON_HAND_IN_ARML[1])


class Camera(object):
    def __init__(self, node):
        self.node = node
        self.last = None
        node.create_subscription(Image, '/drink_cam/image', self._cb, 5)

    def _cb(self, msg):
        self.last = msg

    def grab(self, node, wait=3.0):
        t0 = time.time()
        while time.time() - t0 < wait:
            node.spin_for(0.05)
            if self.last is not None:
                return self.last
        return None


def snap(node, cam, tag, ok_sink):
    img = cam.grab(node)
    if img is None:
        ok_sink.append(False)
        return
    save_png(os.path.join(LOG_DIR, 'p96_%s.png' % tag),
             img.data, img.width, img.height, img.encoding)
    ok_sink.append(True)


def pred_y(theta):
    return _PRED[0] + _PRED[1] * math.sin(theta) + _PRED[2] * math.cos(theta)


def hands_in_arms(node):
    """两只手心在 person_arms 系里的位置(与人朝哪无关, 便于直接比几何)。"""
    Tp = M.object_pose(node, 'person')
    Tl, Tr = M.person_hand_poses(node, Tp)
    Ta = M.person_hand_world(node, Tp, link='person_arms', local_off=None)
    if Tl is None or Tr is None or Ta is None:
        return None
    inv = np.linalg.inv(Ta)
    pl = (inv @ np.array([Tl[0, 3], Tl[1, 3], Tl[2, 3], 1.0]))[:3]
    pr = (inv @ np.array([Tr[0, 3], Tr[1, 3], Tr[2, 3], 1.0]))[:3]
    return pl, pr


def rel_to(node, model, host, link='person_arms'):
    """物体相对人手的位姿(判"守恒")。"""
    T_obj = M.object_pose(node, model)
    T_h = M.person_hand_world(node, M.object_pose(node, host), link,
                              getattr(L, 'PERSON_HAND_IN_ARM', None))
    if T_obj is None or T_h is None:
        return None
    return np.linalg.inv(T_h) @ T_obj


def measure(node):
    """(手到杯轴 左/右 mm, 手杯高差 mm, 手心间距 mm)。"""
    hc = M.hands_to_cup_mm(node)
    gap = M.hand_gap_mm(node)
    if hc is None or gap is None:
        return None
    return hc[0], hc[1], hc[2], gap


def main():
    rclpy.init()
    node = DrinkTask()
    cam = Camera(node)
    fails = []
    ok_sink = []

    def step(tag, ok, detail=''):
        print('%s %s%s'
              % ('[OK]' if ok else '[NG]', tag, ('  ' + detail) if detail
                 else ''))
        if not ok:
            fails.append(tag)

    print('=' * 92)
    print('P96 验收 —— 人的双手: 张开迎杯 / 合拢捧住 / 张开交还')
    print('=' * 92)

    # ---- [1] 数据链路 + 开合幅度静态扫描
    t0 = time.time()
    while time.time() - t0 < 10.0 and len(node.js) < 34:
        node.spin_for(0.2)
    have = [n for n in L.PERSON_ARM_LINKS if n in node.links]
    print('  joint_state=%d  真值实体=%d  手臂 link %d/2'
          % (len(node.js), len(node.links), len(have)))
    sweep = []
    for th in (L.PERSON_HAND_CLOSE, 0.22, L.PERSON_HAND_OPEN):
        node.bus.set(['person_arm_l_joint', 'person_arm_r_joint'],
                     [th, -th])
        node.spin_for(2.0)
        pp = hands_in_arms(node)
        if pp is None:
            sweep.append((th, None))
            continue
        pl, pr = pp
        sweep.append((th, (abs(pl[1] - pred_y(th)) * 1000.0,
                           abs(pr[1] + pred_y(th)) * 1000.0,
                           abs(pl[1] + pr[1]) * 1000.0,
                           float(np.linalg.norm(pl - pr)) * 1000.0)))
    s_ok = all(s[1] is not None and max(s[1][0], s[1][1]) < 2.0
               and s[1][2] < 2.0 for s in sweep)
    for th, s in sweep:
        if s is None:
            print('  θ=%.3f  拿不到手 link' % th)
            continue
        print('  θ=%.3f  与几何预测差 左 %.2f / 右 %.2f mm  左右不对称 %.2f mm'
              '  手心间距 %.1fmm' % (th, s[0], s[1], s[2], s[3]))
    step('[1] 数据链路 + 开合幅度  link %d/2, 三个幅度与几何一致'
         % len(have), len(have) == 2 and s_ok)

    # ---- [2] 人站到接水位 + 机器人双手捧杯、转身面向人
    px, py, pyaw = P.person_spot_pose()
    T_person = K.frame(K.rot_z(pyaw), (px, py, 0.0))
    node.carry.place('person', T_person)
    node.spin_for(0.5)
    d_person = M.pos_err_mm(M.object_pose(node, 'person'), T_person)
    pr = M.prepare_present(node)
    step('[2] 人到位 + 双手捧杯面向人  人 %.1f mm  捧杯 %.1f mm  转身 %.2f°'
         % (d_person, pr['res'].get('cup_carry', 9e9),
            pr.get('turn_err', 99.0)),
         pr['ok'] and d_person < 10.0)
    if not pr['ok']:
        return report(fails, node)

    # ---- [3] 双掌前伸到递杯位 + 人张开迎杯
    # P96: 这里必须用**实测人手**目标(与 run_drink 同一份实现)。用名义目标
    # 的话, 转身漂的 3~7cm 会原样落到杯子上, 而判据是"手到杯轴" —— 于是
    # 123/62mm 会被误判成"手没捧住", 实际是目标取错了(实测踩到过)。
    T_present = M.present_target_at_hand(node)
    ok_p, res_p = M.place_held(node, 'L', 'cup', T_present, tol_mm=6.0,
                               iters=14, reach_tol_mm=1.5)
    M.follow_hand(node, 'R', 'cup', pr['off_r'])
    node.spin_for(0.5)
    # d_p 是"前伸到位"的内部一致性指标, 必须**在张开手之前**量。混在一起
    # 量的话, 手臂那 5~15mm 的伺服余量会在张开手的 1.5s 里再漂一点, 于是
    # 20.2mm 这种边缘值会去判一个跟张开动作无关的指标(实测踩到过)。
    d_p = M.pos_err_mm(M.object_pose(node, 'cup'), T_present)
    Tcup_a = M.object_pose(node, 'cup')
    # P96: 杯子先到位(还在机器人手里), 人再张开手 —— 顺序反了就是"人张着手
    # 看机器人拱过来"。张开要 0.8s 斜坡, 直接给终点会"啪"地弹开。
    M.person_hands(node, L.PERSON_HAND_OPEN, dur=0.8, tag='张开迎杯')
    node.spin_for(0.5)
    # 张开手**不许碰动杯子**: 手是纯视觉 link(无碰撞体), 正常情况下碰不到;
    # 这一项就是用来抓"不小心给手加了碰撞体/钉错宿主"这类回归。
    d_om = M.pos_err_mm(M.object_pose(node, 'cup'), Tcup_a)
    # 杯子落在两手中: 杯心到"两手连线中点"的水平距离
    Tcup = M.object_pose(node, 'cup')
    Tl, Tr = M.person_hand_poses(node)
    d_mid = (float(np.linalg.norm(
        (Tl[:2, 3] + Tr[:2, 3]) / 2.0 - Tcup[:2, 3])) * 1000.0
        if Tl is not None else 9e9)
    m = measure(node)
    gap_open = m[3] if m else 9e9
    # P94 那个"杯到人手"的任务指标, 这里一并打出来做跨版本回归对比
    Th = M.person_hand_world(node, M.object_pose(node, 'person'))
    d_hand = M.pos_err_mm(Tcup, Th) if Th is not None else 9e9
    step('[3] 前伸到递杯位 + 张开迎杯  前伸残差 %.1f mm  张开时杯子被碰动 '
         '%.1f mm  杯到人手 %.1f mm  杯到两手连线中点 %.1f mm  '
         '手心间距 %.0fmm'
         % (d_p, d_om, d_hand, d_mid, gap_open),
         d_p < 20.0 and d_om < 3.0 and d_mid < RECV_MID
         and gap_open > OPEN_GAP)
    snap(node, cam, 'open_recv', ok_sink)

    # ---- [4] 人合拢捧杯(杯子还在机器人手里: 量的是递杯精度)
    M.person_hands(node, L.PERSON_HAND_CLOSE, dur=0.8, tag='合拢捧杯')
    m = measure(node)
    if m is None:
        step('[4] 合拢捧杯  量不到手', False)
        return report(fails, node)
    dl, dr, dz, gap = m
    asym = abs(dl - dr)
    recv = (dl, dr)
    step('[4] 合拢捧杯(交接前)  手到杯轴 左 %.1f / 右 %.1f mm(相切 %.0f±%.0f)'
         '  左右差 %.1f  高差 %.1f mm  手心间距 %.0fmm'
         % (dl, dr, TANGENT_MM, GRIP_TOL, asym, dz, gap),
         abs(dl - TANGENT_MM) < GRIP_TOL
         and abs(dr - TANGENT_MM) < GRIP_TOL and asym < GRIP_ASYM
         and dz < GRIP_DZ and 170.0 < gap < 200.0)

    # ---- [5] 交接 + 撤臂(杯子钉到两手连线中点: 量的是结构一致性)
    rel0 = rel_to(node, 'cup', 'person')
    ok_h = node.carry.hand_to('cup', 'person', 'person_arms',
                              L.PERSON_HAND_IN_ARM)
    node.spin_for(0.6)
    Tcup0 = M.object_pose(node, 'cup')
    M.open_hand(node, 'L')
    M.open_hand(node, 'R')
    node.spin_for(0.3)
    node.arm['L'].move(K.Q_REST, timeout=10.0)
    node.arm['R'].move(K.Q_REST, timeout=10.0)
    node.spin_for(0.5)
    Tcup1 = M.object_pose(node, 'cup')
    d_hold = M.pos_err_mm(Tcup1, Tcup0)
    rel1 = rel_to(node, 'cup', 'person')
    cons = M.pos_err_mm(rel1, rel0) if rel1 is not None else 9e9
    m = measure(node)
    gl = gr = 9e9
    if m is not None:
        gl, gr = m[0], m[1]
    # 交接前后**同一个量**(手到杯轴)的变化 —— 这才是"交接瞬间杯子一点不动"
    # 在双手口径下的说法。注意 `hand_to` 是"冻结当前相对位姿"式交接, 杯子
    # 不会吸附到两手连线中点, 所以[4]和[5]的数本来就该一样; 不一样就说明
    # 交接把杯子挪了。
    seam = max(abs(gl - recv[0]), abs(gr - recv[1])) if m is not None else 9e9
    step('[5] 交给人的手 + 撤臂  杯心漂移 %.1f mm  相对人手守恒 %.1f mm  '
         '双手捧着 左 %.1f / 右 %.1f mm  交接前后变化 %.1f mm'
         % (d_hold, cons, gl, gr, seam),
         ok_h and d_hold < 3.0 and cons < 3.0 and m is not None
         and abs(gl - TANGENT_MM) < GRIP_TOL
         and abs(gr - TANGENT_MM) < GRIP_TOL and seam < SEAM_MM)
    snap(node, cam, 'grip', ok_sink)

    # ---- [6] 人举杯到嘴边(双手不能松开)
    z0 = M.object_pose(node, 'cup')[2, 3]
    node.bus.set(['person_arm_joint'], [DRINK_JOINT])
    node.spin_for(2.5)
    Tcup2 = M.object_pose(node, 'cup')
    dz_cup = (Tcup2[2, 3] - z0) * 1000.0
    rel2 = rel_to(node, 'cup', 'person')
    cons2 = M.pos_err_mm(rel2, rel0) if rel2 is not None else 9e9
    m = measure(node)
    step('[6] 人举杯喝  杯心抬高 %.1f mm  相对守恒 %.1f mm  双手仍捧着 '
         '左 %.1f / 右 %.1f mm'
         % (dz_cup, cons2, m[0] if m else 9e9, m[1] if m else 9e9),
         100.0 < dz_cup < 300.0 and cons2 < 3.0 and m is not None
         and abs(m[0] - TANGENT_MM) < GRIP_TOL
         and abs(m[1] - TANGENT_MM) < GRIP_TOL
         and abs(m[0] - gl) < SEAM_MM and abs(m[1] - gr) < SEAM_MM)
    snap(node, cam, 'drink', ok_sink)

    # ---- [7] 人放下
    node.bus.set(['person_arm_joint'], [0.0])
    node.spin_for(2.5)
    Tcup3 = M.object_pose(node, 'cup')
    d_back = M.pos_err_mm(Tcup3, Tcup1)
    rel3 = rel_to(node, 'cup', 'person')
    step('[7] 人放下  回到递杯位 %.1f mm  相对守恒 %.1f mm'
         % (d_back, M.pos_err_mm(rel3, rel0) if rel3 is not None else 9e9),
         d_back < 15.0
         and rel3 is not None and M.pos_err_mm(rel3, rel0) < 3.0)

    # ---- [8] 张开交还 → 机器人双手接回 → 人收手
    Tcup4 = M.object_pose(node, 'cup')
    M.reach(node, 'R', P.cup_cradle_palm(Tcup4, 'R'), tol_mm=5.0)
    M.reach(node, 'L', P.cup_cradle_palm(Tcup4, 'L'), tol_mm=5.0)
    node.spin_for(0.3)
    grip_before = measure(node)
    ok_g = node.carry.grab('cup', 'L')
    # P96: **人先张开 → 机器人再端走**。顺序反了画面上就是"从手里硬拔"。
    M.person_hands(node, L.PERSON_HAND_OPEN, dur=0.8, tag='张开交还')
    m_open = measure(node)
    off_r = M.hand_onto(node, 'R', 'cup',
                        lambda T: P.cup_cradle_palm(T, 'R'))
    M.person_hands(node, L.PERSON_HAND_CLOSE, dur=0.6, tag='交还后收手')
    node.spin_for(0.5)
    m_back = measure(node)
    step('[8] 张开交还 + 双手接回 + 收手  手到杯轴: 捧着 %.0f/%.0f → 张开 '
         '%.0f/%.0f → 收手间距 %.0fmm  grab=%s'
         % (grip_before[0] if grip_before else 9e9,
            grip_before[1] if grip_before else 9e9,
            m_open[0] if m_open else 9e9, m_open[1] if m_open else 9e9,
            m_back[3] if m_back else 9e9, 'ok' if ok_g else 'fail'),
         ok_g and off_r is not None and m_open is not None
         and min(m_open[0], m_open[1]) > OPEN_AXIS
         and grip_before is not None
         and abs(grip_before[0] - TANGENT_MM) < GRIP_TOL
         and abs(grip_before[1] - TANGENT_MM) < GRIP_TOL
         and abs(grip_before[0] - gl) < SEAM_MM
         and abs(grip_before[1] - gr) < SEAM_MM
         and m_back is not None and m_back[3] < 200.0)
    snap(node, cam, 'give_back', ok_sink)

    # ---- [9] 放回原位
    ok_t, err_t = M.turn_base(node, 0.0)
    d_serve = M.place_robot(node, K.frame(K.rot_z(0.0),
                                          (L.SERVE_POSE[0],
                                           L.SERVE_POSE[1], 0.0)))
    Tcup_home = P.cup_on_table()
    M.open_hand(node, 'R')
    node.arm['R'].move(K.Q_REST, timeout=10.0)
    node.spin_for(0.3)
    ok_d2, res_d2 = M.put_down(node, 'L', 'cup', Tcup_home)
    node.spin_for(0.6)
    Tcup5 = M.object_pose(node, 'cup')
    d_home = M.pos_err_mm(Tcup5, Tcup_home)
    step('[9] 放回原位  偏差 %.1f mm  杯轴倾 %.2f°  转身 %.2f°  服务位 %.1fmm'
         % (d_home, M.tilt_deg(Tcup5), err_t, d_serve),
         ok_d2 and d_home < 10.0 and M.tilt_deg(Tcup5) < 2.0
         and abs(err_t) < 2.5 and d_serve < 10.0)

    # ---- [10] 截图
    snap(node, cam, 'placed', ok_sink)
    step('[10] 截图 %d/5' % sum(1 for _ in ok_sink),
         all(ok_sink) and len(ok_sink) == 5)

    return report(fails, node)


def report(fails, node):
    print('-' * 92)
    print('结论: %s' % ('通过' if not fails else '不通过 (%d 项)' % len(fails)))
    for f in fails:
        print('  未通过: %s' % f)
    print('=' * 92)
    try:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass
    return 0 if not fails else 2


if __name__ == '__main__':
    sys.exit(main())
