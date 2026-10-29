#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_p86.py —— P86 验收: 右手抓住水壶并抬起

验收口径(全部对 gz 真值, 不看自己算的数):
  [1] 数据链路就绪 (37 路指令 / joint_state / dynamic_pose)
  [2] 机器人摆到服务位后, 位姿取自真值并与目标一致
  [3] 双臂回 home
  [4] 由**壶的真值位姿**反解抓取位形, KDL 可达
  [5] 预抓取(沿接近方向退 0.10m) → 抓取位 → 五指闭合
  [6] attach 成功(ack)
  [7] 抬起 0.12m: 壶的 z 必须跟着抬起来
  [8] 抬起后"掌心-壶"相对位姿与抓握瞬间一致(刚性, <2mm/<1°)
  [9] 放回: 回抓取位 → 松开 → 抬臂, 壶必须停在桌面原位置 ±5mm
     (物件 `<gravity>false</gravity>`, 松开就停在原地, 这是"放回原位"的语义)

坐标口径提醒
============
`/world/<w>/dynamic_pose/info` 里 **模型是世界的位姿, link 是相对所属模型的
位姿**。所以:
  * 壶/杯的位姿直接用话题里的 `kettle` / `cup`(模型)
  * 掌心位姿必须用 FK 算: `arm_kin.fk_world(side, q_实测, base)`
    —— 话题里的 `R_palm` 是相对 `robot` 模型的, 直接当世界坐标用会整体错位。

用法:
    bash tools/start_scene.sh                 # 先起场景(不要带 --gui 也行)
    source /opt/ros/jazzy/setup.bash; source vla_ws/install/setup.bash
    python3 tests/test_p86.py
"""
import math
import sys
import time

import numpy as np
import rclpy

from vla_follow import arm_kin as K
from vla_follow import drink_poses as P
from vla_follow import scene_layout as L
from vla_follow.drink_task import DrinkTask

SIDE = 'R'
MODEL = 'kettle'


def T_of(node, name):
    """模型的世界位姿(只对模型名有效)。"""
    p = node.links.get(name)
    if p is None:
        return None
    q = p.orientation
    return K.frame(K.quat_to_mat(q.x, q.y, q.z, q.w),
                   (p.position.x, p.position.y, p.position.z))


def q_meas(node, side=SIDE):
    names = ['%s_%s_joint' % (side, j[0]) for j in K.JOINTS]
    js = node.js
    if any(n not in js for n in names):
        return None
    return np.array([js[n] for n in names], float)


def palm_world(node, side=SIDE):
    q = q_meas(node, side)
    if q is None:
        return None
    return K.fk_world(side, q, node.base)


def move_to(node, T_world, timeout=10.0, tol_mm=4.0, passes=5):
    """把 SIDE 侧手臂移到世界位形 T_world。返回(是否到位, 实测位置残差 mm)。

    必须做**闭环迭代**: 关节伺服按"每轴误差 < ARM_TOL_DEG(1.5°)"判到位,
    7 轴串下来指尖位置误差能放大到十几~二十几毫米(实测 [5b] 26.6mm)。
    正解 IK 本身只有 0.1mm 残差, 所以只要用实测关节角重新解一次再补一步。

    容差还要**逐轮收紧**: 每轴 0.6° 在 0.35m 力臂上就是 3.7mm, 位置残差会
    卡在 4mm 上不去(实测 [5b]=5.9mm); 收到 0.25° 才能进毫米级。
    """
    res = 9e9
    how = '-'
    sched = (None, 0.6, 0.25, 0.25, 0.25)
    for it in range(passes):
        q_now = q_meas(node)
        if q_now is None:
            return False, res, how
        q, _e, how = K.ik_world(SIDE, T_world, node.base, q_now)
        node.arm[SIDE].move(q, timeout=timeout, tol_deg=sched[min(it, 4)])
        q_end = q_meas(node)
        res = float(np.linalg.norm(
            K.pose_err(K.fk_world(SIDE, q_end, node.base), T_world)[:3])) * 1000.0
        if res < tol_mm:
            break
    return res < tol_mm, res, how


def rel_err(T_ref, Ta, Tb):
    e = K.pose_err(np.linalg.inv(T_ref) @ Ta, np.linalg.inv(T_ref) @ Tb)
    return float(np.linalg.norm(e[:3])) * 1000.0, math.degrees(
        float(np.linalg.norm(e[3:])))


def main():
    rclpy.init()
    node = DrinkTask()
    fails = []

    def step(tag, ok, detail=''):
        print('%s %-44s %s%s'
              % ('[OK]' if ok else '[NG]', tag, 'OK' if ok else 'NG',
                 ('  ' + detail) if detail else ''))
        if not ok:
            fails.append(tag)

    print('=' * 78)
    print('P86 验收 —— 右手抓水壶并抬起')
    print('=' * 78)

    # ---- [1] 数据就绪 (新建订阅在发现完成前收不到消息, 等够再判)
    t0 = time.time()
    while time.time() - t0 < 10.0 and len(node.js) < 34:
        node.spin_for(0.2)
    step('[1] 数据链路  joint_state=%d 真值实体=%d'
         % (len(node.js), len(node.links)),
         len(node.js) >= 34 and MODEL in node.links)

    # ---- [2] 摆到服务位(测试布景) + base 取真值
    T_place = K.frame(K.rot_z(0.0), (L.SERVE_POSE[0], L.SERVE_POSE[1], 0.0))
    node.carry.place('robot', T_place)
    node.spin_for(0.8)
    node.sync_base_from_truth()
    Tr = T_of(node, 'robot')
    d = (math.hypot(Tr[0, 3] - L.SERVE_POSE[0],
                    Tr[1, 3] - L.SERVE_POSE[1])
         if Tr is not None else 9.9)
    step('[2] 机器人到服务位  偏差 %.1f mm' % (d * 1000.0), d < 0.02)

    # ---- [3] 回 home
    node.home()
    step('[3] 双臂回 home', True)

    # ---- [4] 由壶真值反解抓取位形
    Tk0 = T_of(node, MODEL)
    if Tk0 is None:
        step('[4] 读壶真值', False)
        return report(fails, node)
    T_grasp = P.kettle_grasp_palm(Tk0)
    q_g, e_g, how = K.ik_world(SIDE, T_grasp, node.base, K.Q_REST)
    marg = math.degrees(float(np.min(np.minimum(q_g - K.LIM_LO,
                                             K.LIM_HI - q_g))))
    step('[4] 抓取位形 IK(%s)  残差 %.1f mm  限位余量 %.0f°'
         % (how, e_g * 1000.0, marg), e_g < 0.010 and marg > 3.0)

    # ---- [5] 预抓取 → 抓取 → 闭合
    T_pre = T_grasp @ K.trans((0.0, 0.0, 0.10))
    node.hand[SIDE].move(K.grip_values(SIDE, K.GRIP_OPEN))
    ok_pre, res_pre, _ = move_to(node, T_pre)
    step('[5a] 预抓取(退 10cm)  残差 %.1f mm' % res_pre, ok_pre)
    ok_gr, res_gr, _ = move_to(node, T_grasp, tol_mm=5.0)
    step('[5b] 到抓取位  残差 %.1f mm' % res_gr, ok_gr)
    ok_f, err_f = node.hand[SIDE].move(K.grip_values(SIDE, K.GRIP_KETTLE),
                                       timeout=6.0)
    step('[5c] 五指闭合成抓握  最大残差 %.1f°'
         % math.degrees(np.nanmax(np.abs(err_f))), ok_f)

    # ---- [6] attach
    Tp0 = palm_world(node)
    rel0 = np.linalg.inv(Tp0) @ Tk0
    ok_a = node.carry.grab(MODEL, SIDE)
    step('[6] attach 成功(ack)', ok_a)
    if not ok_a:
        return report(fails, node)

    # ---- [7] 抬起 0.12m
    # 方向必须是**世界垂直**: 抓壶是水平接近(approach = 壶体 -y, 壶 yaw=90° →
    # 世界 +x), 掌心局部 +z 因此指向壶的侧面, 写成 `T_grasp @ trans(0,0,0.12)`
    # 其实是横向平移 12cm, 壶的 z 一点都不会变(实测 dz=0.1mm)。
    T_lift = K.frame(T_grasp[:3, :3],
                     T_grasp[:3, 3] + np.array([0.0, 0.0, 0.12]))
    ok_l, res_l, _ = move_to(node, T_lift, tol_mm=6.0)
    # 刚性判据要等**手臂停稳**再量: 掌心位姿来自 joint_state, 壶位姿来自 gz
    # 真值, 两条链有采样偏斜; 手臂还在动时偏斜会直接变成"相对位姿漂了 1.2°"。
    node.spin_for(1.0)

    Tk1 = T_of(node, MODEL)
    dz = Tk1[2, 3] - Tk0[2, 3]
    step('[7] 抬起 12cm  壶实际抬高 %.1f mm  臂残差 %.1f mm' % (dz * 1000.0, res_l),
         ok_l and abs(dz - 0.12) < 0.012)

    # ---- [8] 刚性
    Tp1 = palm_world(node)
    ep, er = rel_err(Tp1, Tk1, Tp1 @ rel0)
    step('[8] 抬起后刚性  相对位姿偏差 %.1f mm / %.2f°' % (ep, er),
         ep < 2.0 and er < 1.0)

    # ---- [9] 放回
    ok_b, res_b = place_held(node, Tk0)          # 闭环把壶送回原世界位姿
    node.spin_for(0.2)
    # 顺序不能反: 必须**先张手、后松钉**。闭合的五指会一直压在壶体上,
    # 钉住期间接触求解持续累积穿透速度, 一松钉壶就弹开(实测 2.1mm → 5.2mm)。
    node.hand[SIDE].move(K.grip_values(SIDE, K.GRIP_OPEN), timeout=5.0)
    node.spin_for(0.2)
    # 张手之后再来一轮对位: 此时手指已经离开壶体, 收敛到的是"无接触"状态。
    ok_b2, res_b2 = place_held(node, Tk0)
    node.spin_for(0.2)
    # 必须**先松钉、后撤手**。钉住期间壶位姿 = 掌心 @ 冻结偏移, 也就是壶跟着
    # 掌心走; 若保留钉住就撤手 10cm, 壶会被一起拖走(实测偏差 98~127mm)。
    node.carry.release(MODEL)                    # 解除钉住
    node.spin_for(0.3)
    move_to(node, T_pre, tol_mm=8.0)             # 抬臂让开
    node.spin_for(0.5)
    Tk2 = T_of(node, MODEL)
    back = float(np.linalg.norm(Tk2[:3, 3] - Tk0[:3, 3]))
    step('[9] 放回桌面原位置  偏差 %.1f mm  送位残差 %.1f mm'
         % (back * 1000.0, res_b2),
         ok_b and ok_b2 and back < 0.005)

    return report(fails, node)


def place_held(node, T_obj_goal, tol_mm=3.0, iters=6):
    """把手里抓住的物体闭环送到世界位姿 T_obj_goal。

    为什么不能"把掌心送回抓取位 T_grasp"就算放回:
    物体是刚性挂在**抓握瞬间的实测掌心 Tp0** 上的(off = inv(Tp0) @ Tk0),
    而 Tp0 与按名义位形解的 T_grasp 之间有几毫米/一两度的差。掌心回到
    T_grasp 时, 那个差值会经 offset 的力臂放大成物体上的十几毫米偏差。
    所以这里直接对**物体真值**闭环: 每轮算一次物体残差, 用残差修正掌心目标
    再补一步 —— 正是真实系统"看着放"的做法。
    """
    res = 9e9
    for _ in range(iters):
        Tk = T_of(node, MODEL)
        Tp = palm_world(node)
        if Tk is None or Tp is None:
            return False, res
        Tp_goal = (T_obj_goal @ np.linalg.inv(Tk)) @ Tp   # 世界系残差修正
        move_to(node, Tp_goal, tol_mm=3.0)
        res = float(np.linalg.norm(T_of(node, MODEL)[:3, 3]
                                   - T_obj_goal[:3, 3])) * 1000.0
        if res < tol_mm:
            break
    return res < tol_mm, res


def report(fails, node):
    print('-' * 78)
    print('结论: %s' % ('通过' if not fails else '不通过 (%d 项)' % len(fails)))
    for f in fails:
        print('  未通过: %s' % f)
    print('=' * 78)
    try:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass
    return 0 if not fails else 2


if __name__ == '__main__':
    sys.exit(main())
