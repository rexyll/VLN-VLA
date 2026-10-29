#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""drink_motion.py —— 喝水任务的动作原语（P87 提取自 P86 的验收脚本）

P86 趟出来的三条铁律，这里固化成原语，P87~P89 直接复用
=====================================================
1. **单次 IK + 关节伺服到位远远不够。**
   伺服判据是"每轴 < 1.5°"，7 轴串下来指尖能差 20mm 以上（实测抓取位
   26.6mm）。IK 本身残差只有 0.1mm，所以必须 `reach()`：用实测关节角重新
   解一次再补一步，容差逐轮收紧（1.5° → 0.6° → 0.25°）。

2. **"把手里的东西放到某处"要对物体真值闭环，不是对手即名义位姿。**
   物体是刚性挂在**抓握瞬间的实测掌心**上的（冻结偏移 `off`），而那个掌心
   与名义抓取位差几毫米/一两度，会经 offset 的力臂放大成十几毫米。
   `place_held()` 每轮算一次对象残差、用残差修掌心目标 —— 就是真实系统
   "看着放"的做法。**位置和姿态要一起闭**（P87 补：只闭位置时杯轴倾
   2.75°、壶嘴尖差 14.6mm）。

3. **松手四步顺序不能换：张手 → 对位 → 松钉 → 撤手。**
   ①五指闭合时物体还被压着，先松钉会弹开；
   ②钉住期间物体位姿 = 掌心 @ 冻结偏移，**保留钉住就撤手会把物体拖走**
     （实测偏差 98~127mm）；
   ③张手后再对一次位，松钉时才是"无接触且已收敛"的干净状态。

坐标系约定（踩过的坑，别改）
============================
`/world/<w>/dynamic_pose/info` 里 **模型给的是世界位姿，link 给的是相对
所属模型的位姿**。所以：
  * 物体（壶/杯/人）位姿：按**模型名**直接取；
  * 掌心位姿：必须 `arm_kin.fk_world(side, q实测, base)` 自己算，
    话题里的 `R_palm` 当世界坐标用会整体错位；
  * 底盘 `base` 取自模型 `robot` 的真值，且已锁定来源（odom 不会覆盖）。
"""
import math
import os
import time

import numpy as np

from vla_follow import arm_kin as K
from vla_follow import drink_poses as P
from vla_follow import scene_layout as L

# 到位容差调度：第 0 轮用伺服默认容差，之后逐轮收紧
TOL_SCHED = (None, 0.6, 0.25, 0.25, 0.25)
GRASP_TOL_MM = 5.0
PLACE_TOL_MM = 3.0


# ---------------------------------------------------------------- 基础读量
def q_meas(node, side):
    """实测 7 个臂关节角。缺数据返回 None。"""
    names = ['%s_%s_joint' % (side, j[0]) for j in K.JOINTS]
    js = node.js
    if any(n not in js for n in names):
        return None
    return np.array([js[n] for n in names], float)


def wrist(node, side):
    """掌心世界位姿：实测关节角 → fk → 底盘位姿。"""
    q = q_meas(node, side)
    if q is None:
        return None
    return K.fk_world(side, q, node.base)


def object_pose(node, model):
    """模型的世界位姿。只对模型名有效（link 名给的是相对位姿）。"""
    p = node.links.get(model)
    if p is None:
        return None
    q = p.orientation
    return K.frame(K.quat_to_mat(q.x, q.y, q.z, q.w),
                   (p.position.x, p.position.y, p.position.z))


def pos_err_mm(Ta, Tb):
    return float(np.linalg.norm(np.asarray(Ta)[:3, 3] - np.asarray(Tb)[:3, 3])) * 1000.0


def axis_angle_deg(Ta, Tb):
    """两个位姿的姿态夹角（度）。"""
    Rd = np.asarray(Ta)[:3, :3] @ np.asarray(Tb)[:3, :3].T
    c = max(-1.0, min(1.0, (float(np.trace(Rd)) - 1.0) / 2.0))
    return math.degrees(math.acos(c))


def tilt_deg(T, ref=np.array([0.0, 0.0, 1.0])):
    """物体的 z 轴相对世界竖直方向的夹角（度）—— 用来判"杯口还朝上"。"""
    z = np.asarray(T)[:3, :3] @ np.array([0.0, 0.0, 1.0])
    c = max(-1.0, min(1.0, float(z @ ref)))
    return math.degrees(math.acos(c))


# ---- 人体关节规则体检(P103) -------------------------------------------
# 用户 2026-10-28: "倒水时左臂肘部弯向内侧, 违背人体骨骼运动规律; 顺便检查
# 其他关节有没有违反人体肢体正常运动轨迹"。判据就是下面这几条, 与
# `arm_kin.elbow_metrics` 同源, 验收脚本与运行日志共用一套口径:
#   R1 肘反关节   elbow > 0            (人的肘不能向后折)
#   R2 肘过中线   L: 肘y<0 / R: 肘y>0
#   (原"R3 手过中线"已删: 物体摆偏 100mm 时左手捧杯必然过中线, 那是物体
#    的位置不是手臂的问题; 见 arm_kin.violation_list 的说明)
#   R4 肩内旋过大 |up_roll| > 1.95 rad(112°, 机械限位 2.00)
#   R5 前臂扭转   |fore_roll + wrist_r| > 2.00 rad(115°) —— 人的旋前+旋后
#                 合计约 180°, 但那是"特意去拧"; 端杯子不需要, 而老病态位形
#                 是 171°(2.98), 2.00 拦得住、正常抓杯(1.7~1.9)放得过
#   R6 腕反折     |wrist_p| > 1.50 rad(86°) —— 正好是本臂机械限位, 这条实际
#                 判的是"顶到限位", 而不是"屈伸看着别扭"
#   R7 肘过折     |elbow| > 2.10 rad(120°)
# 阈值只在 `arm_kin.HUMAN_*` / `violation_list` 里维护一份。


def arm_state(node, side):
    """实测关节角 + 人体规则体检量。缺数据返回 None。"""
    q = q_meas(node, side)
    if q is None:
        return None
    m = K.elbow_metrics(side, q)
    el = K.elbow_pos(side, q)
    palm = K.fk(side, q)[:3, 3]
    twist = float(q[4] + q[6])
    # 判据只维护一份: arm_kin.violation_list(R1~R7), 这里不再抄一遍 ——
    # 抄两遍的代价是"日志说 OK、验收说 NG"(P103 就这么踩过一次)。
    bad = K.violation_list(side, q)
    return {'q': [round(float(v), 3) for v in q],
            'elbow_mm': round(float(el[1]) * 1e3, 1),
            'palm_mm': round(float(palm[1]) * 1e3, 1),
            'up_roll': round(float(q[2]), 2),
            'elbow': round(float(q[3]), 2),
            'twist': round(twist, 2),
            'wrist_p': round(float(q[5]), 2),
            'out_line_mm': m['out_line_mm'],
            'out_base_mm': m['out_base_mm'],
            'droop_mm': m['droop_mm'],
            'bad': bad}


def log_arm(node, tag, sides=('L', 'R')):
    """把两侧手臂的关节姿态打一行日志, 有违规则带 ★。返回 dict。"""
    out = {}
    for s in sides:
        st = arm_state(node, s)
        if st is None:
            continue
        out[s] = st
        node.get_logger().info(
            'ARM[%s] %s q=%s 肘y=%+.0f 掌y=%+.0f 外扩线=%+.0f droop=%+.0f'
            ' twist=%+.2f up_roll=%+.2f %s'
            % (tag, s, st['q'], st['elbow_mm'], st['palm_mm'],
               st['out_line_mm'], st['droop_mm'], st['twist'],
               st['up_roll'],
               ('★' + ' '.join(st['bad'])) if st['bad'] else ''))
    return out


# ---------------------------------------------------------------- 到位
def reach(node, side, T_world, tol_mm=3.0, passes=len(TOL_SCHED),
          timeout=10.0, elbow='human', tol_deg=None):
    """闭环把 side 侧手臂的**掌心**移到世界位形 T_world。

    `elbow` 选逆解分支(7 轴对这 6 维位姿冗余 1 维, 同一目标有一整族等价解):
      'human'(默认) 按 `arm_kin.ik_human` 挑"最像人"的解 —— 肘不内收/不过
                    中线、前臂不反拧、肩不顶内旋限位、肘不过度折叠。抓杯、
                    举杯、倒水、捧杯、递杯全都走这条。
      'out'         老的"肘朝外"版(`ik_best_out`), 只优化外扩量, 可能选到
                    up_roll 贴限位的解(P103 实测 L 侧 −1.84/−1.79)。
      None          最朴素的 KDL 种子解, 精度同级但姿态可能很别扭(实测抓杯
                    落到肘折 137°、手腕反拧 186° 的分支上)。

    `tol_deg` 显式给定时**每一轮**都用这个伺服判据(默认走 `TOL_SCHED`
    的逐轮收紧)。为什么要它: 预备位这种"离限位不远、又要求 5mm 内"的位形,
    0.25° 的末轮判据在 7 轴串下来还剩 5~8mm 的不动点(实测 L 侧 7.0mm),
    单纯加轮数没用 —— 只有把伺服判据再逼紧(0.08° ≈ 1.4mrad)才能压下去。

    返回 (是否到位, 实测位置残差 mm, 最后一轮的求解来源标签)。
    """
    # 调试/对照用: P103_ELBOW=none|out|human 可临时覆盖分支策略
    # (验收与回归都跑默认值, 这个开关只给 A/B 对比留口子)
    _ov = os.environ.get('P103_ELBOW')
    if _ov:
        elbow = None if _ov == 'none' else _ov
    res = 9e9
    how = '-'
    for it in range(passes):
        q_now = q_meas(node, side)
        if q_now is None:
            return False, res, how
        # **只在第一轮挑分支**, 之后就地跟踪。7 轴臂的等价解族里, 每轮都重新
        # 挑"最像人"的那个会让手臂在两个分支之间来回跳 —— 实测抓杯时两轮各
        # 选一边, 掌心在中间不动, 残差 115mm 一直不下去。分支是姿态策略,
        # 补位是精度问题, 两件事分开: 第一轮定分支, 后面只用种子做局部修正。
        _mode = elbow if it == 0 else None
        q, _e, how = K.ik_world(side, T_world, node.base, q_now, elbow=_mode)
        node.arm[side].move(
            q, timeout=timeout,
            tol_deg=(tol_deg if tol_deg is not None
                     else TOL_SCHED[min(it, len(TOL_SCHED) - 1)]))
        res = pos_err_mm(wrist(node, side), T_world)
        if os.environ.get('REACH_DEBUG'):
            _qm = q_meas(node, side)
            node.get_logger().info(
                'REACH %s pass%d: %.1fmm (%s) 肘y=%+.0f q=%s 目标q=%s'
                % (side, it, res, how,
                   (K.elbow_pos(side, _qm)[1] * 1e3) if _qm is not None else 0.0,
                   np.round(_qm, 2) if _qm is not None else '-',
                   np.round(q, 2)))
        if res < tol_mm:
            break
    return res < tol_mm, res, how


def sim_now(node):
    """当前**逻辑时间**(秒): 仿真时钟可用时给仿真时间, 否则退回墙上时间。

    底盘这几段(转向/驶入)原来全用墙上时间判超时, RTF 0.3 的机器上 15s 墙
    上时间只等于 4.5s 仿真时间 —— 一次 146° 的转身直接判"转不过去"
    (实测 `turn=False(err=146.6 deg)` 把整个 PREPARE 打成 FAIL)。
    机器人是在仿真时间里动的, 超时也就该按仿真时间算。
    """
    try:
        if bool(node.get_parameter('use_sim_time').value):
            v = node.get_clock().now().nanoseconds * 1e-9
            if v > 1.0:
                return v
    except Exception:
        pass
    return time.time()


def settle_sim(node, s):
    """"量之前先停稳" —— 按**仿真时间**等 s 秒(见 DrinkTask.spin_sim)。

    P102 踩坑: 这些 settle 原来都是墙上时间。VM 上 RTF 0.32, 0.3s 墙上时间只
    等于 0.09s 仿真时间, 手臂还在滑就量, place_held 以为没到位又补一步, 反复
    补到 52.3mm 直接放回失败。仿真里"停稳"当然是仿真秒。
    """
    fn = getattr(node, 'spin_sim', None)
    if fn is None:
        node.spin_for(s)
    else:
        fn(s)


# ---------------------------------------------------------------- 底盘布景
def wrap_deg(a):
    """把角度折到 (-180, 180]。"""
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a


def turn_base(node, yaw_goal, w_max=0.45, tol_deg=1.5, timeout=15.0,
              k=1.2):
    """原地转到 yaw_goal(弧度), 闭环看**真值 yaw**。

    为什么不用 `place` 直接摆过去
    ===========================
    ①teleport 会把整个过程变成一帧, 演示时看不出来"机器人在转身";
    ②`place` 有 0~18mm 的残留抖动(§6.4 #32)。
    走 `/robot/cmd_vel` 让差速底盘真转, 速度取正常的一半(P84 的 0.9rad/s → 0.45),
    近目标按比例减速, 否则惯性会冲过头再来回摆。

    转的过程中手臂关节角不变, 所以手里的杯子是**刚性跟着底盘转**的 —— 不需要
    重解 IK(掌心世界位姿由 fk + base 现算, base 每拍跟着真值刷新)。
    """
    t0 = time.time()
    s0 = sim_now(node)
    err = 180.0
    while (sim_now(node) - s0) < timeout and time.time() - t0 < timeout * 6.0:
        node.sync_base_from_truth(quiet=True)
        err = wrap_deg(math.degrees(yaw_goal - node.base[2]))
        if abs(err) < tol_deg:
            break
        w = max(min(w_max, k * abs(math.radians(err))), 0.05)
        node.drive(0.0, w if err > 0 else -w)
        node.spin_for(0.05)
    node.drive(0.0, 0.0)
    settle_sim(node, 0.5)           # 等惯性停稳(仿真秒)
    node.sync_base_from_truth(quiet=True)
    err = wrap_deg(math.degrees(yaw_goal - node.base[2]))
    return abs(err) < tol_deg, err


def drive_to(node, xy, v_max=0.22, tol_mm=80.0, timeout=30.0, k=1.1,
             v_min=0.06, tag='行驶'):
    """开到世界坐标 (xy), 闭环看**真值 base**。

    与 `place_robot`(teleport)的区别和 `turn_base` 一样: teleport 会把整段
    变成一帧, 画面上看不出"机器人在走"。撤离桌边这段是要给人看的, 所以
    真开过去。

    朝向处理: 目标方位在**车体后方**时(|err|>0.5rad)先原地转、不前进 ——
    否则会一边转一边朝桌子拱上去(撤离目标在服务位正后方 1.15m)。

    `v_min` 是前进速度下限, 原本写死 0.06m/s(防 P 控制在目标前停死)。但它
    决定了**固有残差**: 停稳要等 0.5s, 0.06 × 0.5 = 30mm。递杯补位要补的
    往往只有 38mm, 30mm 的残差等于白挪(P96 yaw+ 组实测), 所以开放成参数,
    精补时给 0.02 —— 残差 ~10mm, 再由调用方多补一轮吃掉。

    返回 (是否到位, 位置残差 mm)。

    P102: `timeout` 的语义从"总时长"改成"**多久没进展**" —— 这台虚拟机的
    RTF 会随负载在 0.15~1.0 之间跳(GUI 一开就掉到 0.3 以下), 用墙钟总时长
    卡的话, 同样的 1.15m 撤离有时 5s 到位、有时 30s 只走了一半就超时放弃
    (实测出现过残差 604mm / 1377mm, 后面所有"以 base 为准"的步骤全废)。
    现在只在"连续 timeout 秒没有前进 50mm"时才收工, 另加 4×timeout 的硬上限。
    """
    t0 = time.time()
    s0 = sim_now(node)
    last_t = 0.0
    best = 9e9
    while True:
        now = sim_now(node) - s0          # 逻辑秒(见 sim_now)
        if now > timeout * 6.0 or time.time() - t0 > timeout * 18.0:
            break
        node.sync_base_from_truth(quiet=True)
        dx, dy = xy[0] - node.base[0], xy[1] - node.base[1]
        dist = math.hypot(dx, dy)
        if dist * 1000.0 < tol_mm:
            break
        if dist < best - 0.05:
            best, last_t = dist, now
        elif now - last_t > timeout:
            node.get_logger().warn(
                'DRIVE %s: 还剩 %.0fmm 且 %.0fs 没有进展, 放弃'
                % (tag, dist * 1e3, timeout))
            break
        cyaw = node.base[2]
        # 目标方向转到车体系: +x 车头
        fwd = dx * math.cos(cyaw) + dy * math.sin(cyaw)
        lat = -dx * math.sin(cyaw) + dy * math.cos(cyaw)
        e = math.atan2(lat, fwd)
        if abs(e) > 0.5:
            v, w = 0.0, max(-0.45, min(0.45, 1.2 * e))
        else:
            v = max(min(v_max, k * dist), v_min)
            w = max(-0.30, min(0.30, 1.0 * e))
        node.drive(v, w)
        node.spin_for(0.05)
    node.drive(0.0, 0.0)
    settle_sim(node, 0.5)               # 等惯性停稳(仿真秒)
    node.sync_base_from_truth(quiet=True)
    res = math.hypot(xy[0] - node.base[0], xy[1] - node.base[1]) * 1000.0
    node.get_logger().info(
        'DRIVE %s -> (%.2f,%.2f)  残差 %.0fmm' % (tag, xy[0], xy[1], res))
    return res < tol_mm, res


def place_robot(node, T_goal, tol_mm=8.0, tries=3, settle=0.25):
    """把底盘 teleport 到 T_goal, 用真值核对, 不够就重放。

    `CarryBus.place` 是"下一帧 SetWorldPoseCmd", 但 gz 的 teleport **不清零
    速度**, 残留量实测 0.0~18.4mm 浮动(写死 20mm 阈值会贴边假 NG)。多放两针
    就收敛; base 由真值锁定, 重放不会污染 FK。

    返回最终位置残差 mm。
    """
    # P98 补: 收敛判据原来**只查位置**。teleport 不清零速度(§6.4 #35 同源),
    # follower 正在驱动时被传送到服务位, 车会带着残余角速度继续转 —— 实测
    # yaw 漂 0.182rad(10.4°), IK 在偏转后的基座系里解到限位, reach 差 318mm
    # 假性失败。这里把 yaw 一并卡进收敛判据。
    res = 9e9
    yaw_err = 9e9
    for _ in range(max(1, tries) * 2):
        node.carry.place('robot', T_goal)
        settle_sim(node, settle)
        res = pos_err_mm(object_pose(node, 'robot'), T_goal)
        yaw_err = abs(yaw_err_of(object_pose(node, 'robot'), T_goal))
        if res < tol_mm and yaw_err < math.radians(2.0):
            break
    node.sync_base_from_truth(quiet=True)
    return res


def yaw_err_of(T_obj, T_goal):
    """两个位姿的 yaw 差(弧度, 绝对值)。"""
    yaw_a = math.atan2(T_obj[1, 0], T_obj[0, 0])
    yaw_b = math.atan2(T_goal[1, 0], T_goal[0, 0])
    d = yaw_a - yaw_b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


# ---------------------------------------------------------------- 手部
def open_hand(node, side, timeout=5.0):
    return node.hand[side].move(K.grip_values(side, K.GRIP_OPEN),
                                timeout=timeout)


def close_hand(node, side, grip, timeout=6.0):
    return node.hand[side].move(K.grip_values(side, grip), timeout=timeout)


# ---------------------------------------------------------------- 抓取
def grasp(node, side, model, palm_fn, grip, approach=0.10, verbose=True):
    """抓取并附着。palm_fn(T_obj) -> 掌心目标位姿（kettle/cup 几何不同）。

    流程：张手 → 退到接近位 → 到抓取位 → 闭五指 → 钉住（ack 确认）。
    返回 (是否成功, 抓取位残差 mm)。
    """
    open_hand(node, side)
    Tk = object_pose(node, model)
    if Tk is None:
        return False, 9e9
    T_grasp = palm_fn(Tk)
    T_pre = T_grasp @ K.trans((0.0, 0.0, approach))
    reach(node, side, T_pre, tol_mm=8.0)
    ok, res, how = reach(node, side, T_grasp, tol_mm=GRASP_TOL_MM)
    # P98 补: reach 严重未到位还照旧 attach, 会把物体钉到 0.3m 外(实测把杯
    # 钉飞、场景全污)。阈值取 3x 容差: 边缘差几毫米照常抓(IK 已收敛, 手指
    # 闭合余量足够), 300mm 级的灾难失败才拒绝, 让上游走 FAIL 路径。
    if res > GRASP_TOL_MM * 3.0:
        if verbose:
            node.get_logger().warn(
                'grasp %s (%s): reach 未到位 %.1fmm (IK=%s), 跳过 attach'
                % (model, side, res, how))
        return False, res
    close_hand(node, side, grip)
    ok_a = node.carry.grab(model, side)
    if verbose:
        node.get_logger().info(
            'grasp %s (%s): 到位 %.1fmm (IK=%s)  attach %s  掌心倾斜 %.1f°'
            % (model, side, res, how, 'ok' if ok_a else 'fail',
               tilt_deg(wrist(node, side))))
    return ok_a, res


# ---------------------------------------------------------------- 送位
def held_palm_target(node, model, T_obj_goal):
    """让手里抓住的物体落在 T_obj_goal 时，掌心该在哪。"""
    off = node.carry.items[model]['offset']
    return np.asarray(T_obj_goal, float) @ np.linalg.inv(off)


def place_held(node, side, model, T_obj_goal, tol_mm=PLACE_TOL_MM,
               tol_deg=2.0, iters=8, settle=0.30, reach_tol_mm=None,
               elbow='human', budget=60.0):
    """把手里抓住的物体闭环送到世界位姿 T_obj_goal(**位置 + 姿态**)。

    返回 (是否到位, 位置残差 mm)。

    `reach_tol_mm` 是**每轮补位**用的容差(默认沿用 max(tol_mm,2.0))。
    前伸到递杯位(0.62m)时手臂接近伸直, 每轴 0.25° 的伺服判据串下来指尖会
    留 12~14mm 的不动点 —— 这时候单纯加迭代没用, 只有把每轮补位逼紧
    (1.5mm)才能再往下压一点。

    为什么必须闭姿态(P87 踩坑)
    ==========================
    P86 的版本只闭位置, 姿态没人管 —— 实测杯轴倾 2.75°(阈值 2°)、壶嘴尖差
    14.6mm。壶嘴尖在模型原点外 ~0.19m, 2° 姿态误差就是 6.6mm, 这才是嘴尖
    偏差的主项。根因是"掌心名义位姿"和"实际到位掌心"差的那一两度, 会经冻结
    偏移 `off` 原样传给物体。

    刚体偏移是不变量, 所以位姿增量可以整体左乘:
        T_obj = T_palm @ off  ⟹  想让 T_obj 变成 goal, 就令
        T_palm ← (goal @ inv(T_obj)) @ T_palm
    位置和姿态一次闭合, 不用分两套逻辑。

    为什么每轮先 spin 再测
    ======================
    伺服 `move()` 是"每轴进入容差就返回", 手臂其实还在滑行。P86 就吃过这个
    亏: 闭环报 3.0mm, 停 0.3s 再量变成 5.9mm。所以每一轮**先等停稳再量**。

    姿态容差为什么是 2° 而不是 1°
    ===========================
    物体位姿 = 掌心实测位姿 @ 冻结偏移, 所以杯子的姿态误差**就是**掌心姿态
    误差。掌心的姿态由伺服决定: 每轴 0.25° 的判据在 7 轴串联后实测残留
    ~1.6° —— 这不是闭环能补掉的(补一步还是同一个绝对目标, 收敛到同一个不动
    点), 只有把每轴判据再收紧才行, 代价是每次 move 慢一倍。1.6° 的杯轴倾对
    水面的影响是 1.6°, 肉眼看不出, 也不洒。
    """
    res, ang = 9e9, 9e9
    good = 0
    t0 = time.time()
    for _ in range(iters):
        # 总预算(墙上秒): 每轮的 settle 已经是仿真秒、`reach` 里的 move 也按
        # 仿真秒判超时, 低 RTF 下一轮可能就要 15~30s 墙上时间。不设总闸时,
        # 一条收敛不了的路径会把 8 轮全走完(实测 PUT_BACK 一段吃掉 100s,
        # 把喝水锁和验收的总时长一起拖爆)。到点就带着当前残差返回, 由上层
        # 决定"不松手"。
        if budget is not None and time.time() - t0 > budget:
            node.get_logger().warn(
                'PLACE %s 超预算 %.0fs, 当前残差 %.1fmm', side, budget, res)
            break
        settle_sim(node, settle)
        Tk = object_pose(node, model)
        Tp = wrist(node, side)
        if Tk is None or Tp is None:
            return False, res
        res = pos_err_mm(Tk, T_obj_goal)
        ang = axis_angle_deg(Tk, T_obj_goal)
        if ang > 180.0:
            ang = 360.0 - ang
        if res < tol_mm and ang < tol_deg:
            # 踩坑(P88): 一轮达标就返回, 手臂可能还在滑行 —— 实测退出时 0.6mm,
            # 停 0.5s 后变 2.17°。所以要求**连续两轮**达标才算收敛。
            # 而且第二轮达标时**不能再补一步 reach**: 已经 0.9mm 了还去动一次,
            # 等于自己把它推开几毫米(实测壶就位从 1.2 变 5.8mm)。
            good += 1
            if good >= 2:
                return True, res
            continue
        good = 0
        dT = np.asarray(T_obj_goal, float) @ np.linalg.inv(Tk)
        if os.environ.get('PLACE_DEBUG'):
            node.get_logger().info(
                'PLACE %s it: 物 %s / 目标 %s  增量 %.1fmm %.1f°  掌心目标 %s'
                % (side, np.round(Tk[:3, 3], 3), np.round(
                    np.asarray(T_obj_goal)[:3, 3], 3),
                   pos_err_mm(Tk, T_obj_goal), axis_angle_deg(Tk, T_obj_goal),
                   np.round((dT @ Tp)[:3, 3], 3)))
        reach(node, side, dT @ Tp,
              tol_mm=(max(tol_mm, 2.0) if reach_tol_mm is None
                      else reach_tol_mm), elbow=elbow)
    return (res < tol_mm and ang < tol_deg), res


def put_down(node, side, model, T_obj_goal, retreat=0.10,
             tol_mm=PLACE_TOL_MM):
    """松手放回。四步顺序见模块头注释。返回 (是否, 松钉前残差 mm)。"""
    ok1, res = place_held(node, side, model, T_obj_goal, tol_mm)
    open_hand(node, side)
    node.spin_for(0.2)
    # 张手后再对一次位：此时手指已离开物体，收敛到的是"无接触"状态
    ok2, res2 = place_held(node, side, model, T_obj_goal, tol_mm)
    T_back = wrist(node, side) @ K.trans((0.0, 0.0, retreat))
    node.carry.release(model)
    node.spin_for(0.3)
    reach(node, side, T_back, tol_mm=8.0)
    return (ok1 and ok2), res2


# ---------------------------------------------------------------- 双手捧
def _blend_pose(Ta, Tb, a):
    """位姿插值: 位置线性、姿态按轴角比例转(a=0 给 Ta, 1 给 Tb)。"""
    Ra, Rb = np.asarray(Ta, float)[:3, :3], np.asarray(Tb, float)[:3, :3]
    d = np.asarray(Tb, float)[:3, 3] - np.asarray(Ta, float)[:3, 3]
    R = Ra.T @ Rb
    c = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
    ang = math.acos(c)
    if ang < 1e-6:
        Rn = Ra.copy()
    else:
        if abs(math.pi - ang) < 1e-4:          # 近 180°: 取对称轴
            A = (R + np.eye(3)) / 2.0
            v = np.sqrt(np.maximum(np.diag(A), 0.0))
            k = int(np.argmax(v))
            axis = A[:, k] / max(1e-9, np.linalg.norm(A[:, k]))
        else:
            axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0],
                             R[1, 0] - R[0, 1]]) / (2.0 * math.sin(ang))
        Rn = Ra @ K.rot(tuple(axis), ang * a)
    T = np.eye(4)
    T[:3, :3] = Rn
    T[:3, 3] = np.asarray(Ta, float)[:3, 3] + d * a
    return T


def put_down_two(node, model, T_obj_goal, tol_mm=PLACE_TOL_MM, steps=4,
                 retreat=0.10, elbow='human', info=None):
    """**双手一起**把杯子放回桌面: 全程谁都不许先松, 落稳后才同时张开。

    为什么不能沿用 `put_down` 的"单臂放回"(2026-10-28 用户指出)
    ==========================================================
    原来的 PUT_BACK 是"右手先 open → 右臂撤到 Q_REST → 左手 put_down":
    画面上就是杯子还悬在半空、一只手已经松开了。右手虽然没钉住杯子(附着只有
    主手一路), 但它是**捧着**的 —— 先松手既不真实, 张指那一下还会蹭到杯壁。

    这里的顺序:
        ① 两只手各自冻结"物体系 → 掌心"的偏移, 按 steps 分步一起下降
           (主手 L 负责把杯子带到目标, 副手 R 跟着同一目标走, 谁也离不开杯子)
        ② 主手 place_held 收尾到 tol_mm
        ③ **落稳复核**: 停 settle 后杯子仍贴着桌面目标(不是被手托着)
        ④ 两只手**同时**张开 → 松开附着 → 一起撤离
    返回 (是否, 松钉前残差 mm)。
    """
    T_obj = object_pose(node, model)
    Tl, Tr = wrist(node, 'L'), wrist(node, 'R')
    if T_obj is None or Tl is None or Tr is None:
        return False, 9e9
    # 副手要是根本没贴在杯子上(hand_onto 失败过), 就别拿一个 20cm 的偏移去
    # "跟", 那会把整条右臂甩出去 —— 退回单臂放回, 但**仍然不提前松手**。
    d_r = float(np.linalg.norm(Tr[:3, 3] - T_obj[:3, 3]))
    if d_r > 0.20:
        node.get_logger().warn(
            'PUT_DOWN_TWO 右手离杯子 %.0fmm(没贴上), 退回单臂放回' % (d_r * 1e3))
        return put_down(node, 'L', model, T_obj_goal, retreat=retreat,
                        tol_mm=tol_mm)
    off_l = np.linalg.inv(T_obj) @ Tl
    off_r = np.linalg.inv(T_obj) @ Tr
    for i in range(1, steps + 1):
        Ti = _blend_pose(T_obj, T_obj_goal, i / float(steps))
        reach(node, 'L', Ti @ off_l, tol_mm=4.0, elbow=elbow)
        reach(node, 'R', Ti @ off_r, tol_mm=6.0, elbow=elbow)
        settle_sim(node, 0.12)
    ok1, res = place_held(node, 'L', model, T_obj_goal, tol_mm, elbow=elbow)
    settle_sim(node, 0.5)
    Tc = object_pose(node, model)
    res = pos_err_mm(Tc, T_obj_goal) if Tc is not None else 9e9
    resting = (Tc is not None) and res < tol_mm
    # 落稳那一刻两只手离杯子多远(验收用): 都还在杯旁 => 没提前松手。
    # 两个口径:
    #   `hand_gap_rest`  = 掌心到**目标杯心**的 3D 距离(含高度差, 所以天然
    #                      偏大: 手掌捧在杯身中部, 到杯底原点有 60mm 级高差)
    #   `hand_axis_rest` = 掌心到**杯子竖轴**的水平距离 —— 杯是竖直圆柱,
    #                      手从两侧捧, 这个才是"还捧在杯上"的判据(相切
    #                      CUP_GRIP_R=92mm; 松手撤走会到 200mm 外)
    d_rest = {}
    d_axis = {}
    Tc0 = Tc if Tc is not None else np.asarray(T_obj_goal)
    for s in ('L', 'R'):
        Tp = wrist(node, s)
        if Tp is None:
            d_rest[s] = 9e9
            d_axis[s] = 9e9
            continue
        d_rest[s] = float(np.linalg.norm(Tp[:3, 3] - np.asarray(T_obj_goal)[:3, 3]))
        d_axis[s] = float(np.hypot(Tp[0, 3] - Tc0[0, 3], Tp[1, 3] - Tc0[1, 3]))
    if not resting:
        node.get_logger().warn(
            'PUT_DOWN_TWO 杯子还没到位(%.1fmm > %.1f), 两只手都不松'
            % (res, tol_mm))
    elif info is not None:
        # P103: 落稳那一刻(手还捧着杯子、还没张指)的关节姿态 —— 验收要量的是
        # **放回这个动作**的位形, 不是随后 `hands_off` 抬手的位形
        info['arm_rest'] = dict((s, arm_state(node, s)) for s in ('L', 'R'))
        info.update(rest_mm=round(res, 1),
                    cup_z_rest=round(float(np.asarray(T_obj_goal)[2, 3]), 4),
                    hand_gap_rest=dict((k, round(v * 1e3, 1))
                                       for k, v in d_rest.items()),
                    hand_axis_rest=dict((k, round(v * 1e3, 1))
                                        for k, v in d_axis.items()))
    # 同时张开(指头离开杯壁), 再收一次位, 然后才松钉
    open_hand(node, 'L')
    open_hand(node, 'R')
    settle_sim(node, 0.25)
    # 张手之后**只压实、不重挑分支**(elbow=None)。这一步的目标就是"再对一次位",
    # 手已经贴在杯壁上; 要是走 `elbow='human'` 就会在酒杯旁边重新做一次分支决策,
    # 实测有一轮把左臂挑成 `q=[−1.53,−1.30,+1.90,−2.42,0.42,0.71,+1.57]`
    # (up_roll 顶 R4、肘折 2.42 越 R7)—— 姿态难看之外, 这只"重新拧过"的手
    # 随后撤手/倒车时把杯子蹭走 20mm。压实就该沿用当前分支, 做局部跟踪。
    ok2, res2 = place_held(node, 'L', model, T_obj_goal, tol_mm, elbow=None)
    node.carry.release(model)
    settle_sim(node, 0.6)
    # 松钉后**没有再掉**: 正数=往下掉了(说明松手时还没真落桌)
    if info is not None:
        Tc3 = object_pose(node, model)
        if Tc3 is not None:
            info['drop_mm'] = round(
                (float(np.asarray(T_obj_goal)[2, 3]) - float(Tc3[2, 3])) * 1e3, 1)
    # 撤手: **侧向外张 120mm + 上抬 120mm**(不是后撤, 见 hands_off 的实测)。
    # elbow=None(= 只做局部跟踪, 不重挑分支): 撤手是"把手掌从杯子上拿开",
    # 姿态该跟松手那一刻一模一样。在这里重新做分支决策, 实测会挑出
    # `up_roll=+1.91 / 肘折 −2.36`(R4/R7 双越)的拧手位形。
    ok_off, off_mm = hands_off(node, model, out=0.12, up=0.12,
                               tol_mm=10.0, elbow=None, info=info)
    return (ok1 and resting and ok2 and ok_off), res2


# ------------------------------------------------------------------ 收臂/抬臂
def hands_off(node, model, out=0.12, up=0.12, tol_mm=10.0, elbow=None,
              info=None):
    """松手之后把两只手**先向两侧张开、再抬起**, 然后才让底盘倒车。

    为什么必须有这一步(P102 踩坑)
    ============================
    原来的顺序是"松钉 → 直接倒车"。松钉那一刻两只手还捧在杯壁高度上
    (掌心 z=0.773 = 杯心高度, 杯口 0.82), 车一倒退, 手臂带着掌心整体往后
    平移, 掌心就**贴着杯壁刮过去** —— 实测端到端里杯子被带偏 **47.9mm**
    (松手时 2.2mm), 画面上就是"机器人一退, 杯子跟着滑一下"。
    人不会这样: 松手后必然先把手从杯子上拿开、抬一抬, 再转身/后退。

    动作(pitch 成两步): ①**先抬** `up`(掌心底面 773 → 893mm, 指尖随之越过
    杯口 820); ②**再向两侧外张** `out` —— 掌心本来只离杯轴 **74mm**(杯半径
    40 + 掌厚), 左右各外张 120mm 就到 **194mm**, 彻底离开杯壁。这一步不是
    "收臂", 只是"离开杯子"; 真正的往下放仍然发生在退到位之后
    (`hang_natural`), 顺序没变。

    **顺序不能反**(离线扫过 27 种组合, 局部跟踪逐步判定):
    "先外张再抬"时, 第一步 L 的 `up_roll` 会冲到 **−1.95~−1.97**(R4 上限 1.90),
    画面上是"抽手的瞬间肩膀往里拧一下"; "先抬再外张"则每一步都干净
    (L 终: 腕 −1.73 / 肘 −1.86 / 扭 −1.59, R 终: 腕 +1.10 / 肘 −1.90 / 扭 −0.67)。
    原因是掌心还停在杯旁、高度没变时, 肩到掌的连线几乎水平, 侧向平移要
    肩关节**内旋**让肘扫出去; 先抬起来之后肩已经上旋, 侧向平移就变成
    单纯的肘/上臂转动。

    为什么是"外张"而不是"后撤"(P103 第五轮实测)
    ==========================================
    原来用的是"沿 杯→车 方向后撤 120mm + 抬 90mm"。这个位形在人体规则下
    **一条合规分支都没有**(离线扫 324 组种子 x 20 个位移组合):

        后撤 0.10     up 0.00/0.03/0.05/0.07/0.09 → 合规 0 / 2 / 32 / 38 / 44 条
        后撤 0.12 及更大  任何 up                    → 合规 **0** 条
        纯外张 0.12 + 抬 0.12                        → 合规 **55** 条, 代价 0.15

    原因: 杯子离身体很近, "后撤"是把掌心往胸口方向推, 肩到掌的距离一路缩短,
    肘只能越折越深 —— 实测那一轮撤手后 L 是 `elbow=−2.19`(R7 越)/ `twist=
    −2.13`(R5 越) / `wrist_p=1.50`(R6 顶限), R 是 `elbow=−2.23`。而"侧向外张"
    是把掌心**推远**肩, 肘自然松开, 代价反而最低(同一位形 0.15 vs 1.01)。

    返回 (是否到位, 最大残差 mm)。
    """
    worst = 0.0
    lifts = [[np.array([0.0, 0.0, up]), np.array([0.0, s, 0.0]) * out]
             for s in (1.0, -1.0)]          # L 在 +y 侧, R 在 −y 侧
    # 两段都只做**局部跟踪**(`elbow` 原样传下去, 默认 None): 撤手是"把手从
    # 杯子上拿开", 姿态必须跟松手那一刻一致。试过第一段改用 human 重挑分支,
    # 结果更差(实测右肘 −2.20, 而纯跟踪是 −1.97) —— 因为"手在杯旁、掌心朝下、
    # 桌面高度"这个位形族里, 代价最优的几条本来就折得深, 重挑反而往深里走。
    for k in range(2):                      # 第 0 段全抬, 第 1 段全外张
        for i, s in enumerate(('L', 'R')):
            Tp = wrist(node, s)
            if Tp is None:
                continue
            T = np.asarray(Tp, float).copy()
            T[:3, 3] = T[:3, 3] + lifts[i][k]
            _ok, e, _how = reach(node, s, T, tol_mm=tol_mm, elbow=elbow)
            worst = max(worst, float(e))
    if info is not None:
        info['handoff_mm'] = round(worst, 1)
        info['handoff_up_mm'] = round(up * 1e3, 1)
        info['handoff_out_mm'] = round(out * 1e3, 1)
    return worst <= tol_mm, worst


# ------------------------------------------------------------------ 收臂/抬臂
def table_gap_m(node, pad=0.0):
    """车到**桌沿**的净距(m): 底盘外接圆已经扣掉, 就是"车壳到桌子"的空隙。

    用户说的"机器人与桌子的距离大于臂+手全长"就是这个量 —— 只有当它大于
    `K.ARM_REACH`(0.715m)时, 整条臂放下来(以及前臂绕肘回收的那段圆弧)才
    不可能扫到桌子。`pad` 是额外要留的余量。
    """
    bx, by, _byaw = node.base
    return L.table_clearance(bx, by, pad=pad)


def hang_natural(node, side, fold=None, info=None):
    """按**人体顺序**把一侧手臂从当前位形收到悬垂(见 K.hang_path)。

    返回 (结束时实测 q, 过程信息 dict)。信息里有:
        elbow_drift_mm  第①段前后**肘点**的世界位移 —— 应当在 0.5mm 量级,
                        这就是"前臂以肘为支点"的直接证据;
        fold_deg        第①段把前臂折了多少度;
        wrist_deg       全程腕部关节的总摆动(>90° 基本等于"翻转");
        hand_min_z/max_x 掌心轨迹的最低点 / 最前点(确认没往桌子方向甩)。
    """
    q0 = q_meas(node, side)
    if q0 is None:
        node.arm[side].move(K.Q_HANG, timeout=12.0)
        return K.Q_HANG, {}
    fold_v = K.HANG_FOLD if fold is None else fold
    path = K.hang_path(q0, fold=fold_v)
    el0 = K.elbow_pos(side, q0)
    pw0 = K.fk_world(side, q0, node.base)[:3, 3]
    # 第①段的终点 = "折肘 + 腕归零"之后, 此时 pan/lift/up_roll 还没动
    q_fold = q0.copy()
    q_fold[3] = min(q0[3], fold_v)
    q_fold[4] = q_fold[5] = q_fold[6] = 0.0
    el1 = K.elbow_pos(side, q_fold)
    zmin = 9e9
    xmax = -9e9
    wstep = 0.0
    prev = q0.copy()
    for q in path:
        node.arm[side].move(q, timeout=6.0, tol_deg=3.0)
        wstep = max(wstep, float(np.max(np.abs(q[[4, 5, 6]] - prev[[4, 5, 6]]))))
        prev = q
        pw = K.fk_world(side, q, node.base)[:3, 3]
        zmin = min(zmin, float(pw[2]))
        xmax = max(xmax, float(pw[0] - node.base[0]))
    node.arm[side].move(K.Q_HANG, timeout=8.0)
    node.hand[side].move(K.grip_values(side, K.GRIP_HANG), timeout=6.0)
    q1 = q_meas(node, side)
    out = {'elbow_drift_mm': round(float(np.linalg.norm(el1 - el0)) * 1e3, 1),
           'fold_deg': round(math.degrees(min(q0[3], fold_v) - q0[3]), 1),
           # 腕部: 总摆幅 + **单步**最大摆幅。总摆幅大不等于"翻转"(carry 位里
           # 前臂本来就拧着 1.2rad, 归零是应该的); 单步大才是"一下子翻过去"。
           'wrist_deg': round(math.degrees(abs(q0[5]) + abs(q0[6])
                                           + abs(q0[4])), 1),
           'wrist_step_deg': round(math.degrees(wstep), 1),
           'hand_min_z': round(zmin, 3), 'hand_max_x': round(xmax, 3),
           'q0': [round(float(v), 2) for v in q0],
           'palm0_m': [round(float(pw0[0] - node.base[0]), 3),
                       round(float(pw0[1] - node.base[1]), 3),
                       round(float(pw0[2]), 3)],
           # 收臂这一刻车离桌沿多远 —— 验收用它证明"先退到位再收臂"
           'gap_m': round(table_gap_m(node), 3),
           'reach_m': round(K.ARM_REACH, 3),
           'waypoints': len(path)}
    if info is not None:
        info[side] = out
    node.get_logger().info(
        'HANG[%s] 自然收臂: q0=%s 掌%s → 折肘 %.0f°(肘点漂移 %.1fmm) '
        '腕总摆 %.0f°/单步 %.0f° 掌心轨迹 z≥%.2f x≤%.2f  %d 段'
        % (side, np.round(q0, 2), out['palm0_m'], out['fold_deg'],
           out['elbow_drift_mm'], out['wrist_deg'], out['wrist_step_deg'],
           out['hand_min_z'], out['hand_max_x'], out['waypoints']))
    return q1, out


def ready_palm_target(node, side, r=None, z=None, y=None):
    """抬臂预备位的掌心目标(世界系): 前臂端平、**掌心朝下、手指朝前**。

    几何取"车前 `READY_R`、高 `READY_Z`" —— 用户要求"双手抬到与水壶/水杯
    同等高度", 所以 z 对齐桌面器物(0.72 台面 + 壶身把握 = 0.82), 这样车
    再开近时手不需要再上下找高度。

    姿态为什么是"掌心朝下"而不是抓杯时的"掌心朝内": 后者在 0.38m/0.82m 这个
    近身位形下, 左臂的 KDL 解会把 wrist_r 顶到 −2.7(限位 −2.8), 后面抓杯的
    闭环 IK 就动不了了(实测举杯直接发散 1166mm)。掌心朝下这一族解左右都落在
    行程中部(应力 0.55), 抓杯时手腕再翻 90° 也很自然 —— 人走到桌前也是先
    把手放平, 再翻腕去握。
    """
    bx, by, byaw = node.base
    rr = K.READY_R if r is None else r
    zz = K.READY_Z if z is None else z
    yy = (K.READY_Y if y is None else y) * (1.0 if side == 'L' else -1.0)
    # 掌系约定: 手指朝 -z, 掌心法向 +y。这里 y=世界 −z(掌心向下)、
    # z=世界 −x(手指朝前) ⇒ x = y×z = 世界 +y。
    zax = np.array([-1.0, 0.0, 0.0])
    curl = np.array([0.0, 0.0, -1.0])
    R = np.column_stack([np.cross(curl, zax), curl, zax])
    return K.frame(R, np.array(L.base_to_world(rr, yy, zz, bx, by, byaw)))


def lift_ready(node, sides=('L', 'R'), info=None, r=None, z=None):
    """悬垂 → 抬臂预备位: 肩往上抬, **同时**以肘为支点弯曲(用户要求)。

    返回 (是否都到位, 逐侧残差 mm)。IK 走 `ik_natural`(关节居中优先) ——
    这个高度上手肘本来就该在体侧下方, 不追求"肘朝外"。
    """
    ok_all = True
    errs = {}
    for s in ('R', 'L'):
        if s not in sides:
            continue
        T = ready_palm_target(node, s, r=r, z=z)
        q0 = q_meas(node, s)
        if q0 is None:
            q0 = K.Q_HANG.copy()
        T_loc = np.linalg.inv(K.base_pose(*node.base)) @ T
        q, err, how = K.ik_natural(s, T_loc, K.Q_HANG.copy())
        if err > 2e-3:                      # 换 Q_REST 种子再试一次
            q, err, how = K.ik_natural(s, T_loc, K.Q_REST.copy())
        ok = err <= 2e-3
        if not ok:
            node.get_logger().warn(
                'LIFT[%s] 预备位 IK 未收敛 %.1fmm (%s)' % (s, err * 1e3, how))
        for qq in K.raise_path(q0, q):
            node.arm[s].move(qq, timeout=6.0, tol_deg=3.0)
        node.arm[s].move(q, timeout=6.0)
        # 路点走完只保证"关节进了 3° 容差", 7 轴串下来指尖还有 20mm 级残差
        # (实测 17~25mm)。预备位是后面抓杯的起点, 差 20mm 就要靠抓取那一步
        # 自己补 —— 用与全局一致的闭环再压一次。
        _ok, e, _how2 = reach(node, s, T, tol_mm=5.0, passes=3)
        if e > 4.0:
            # 还有 5~8mm 的不动点(见 reach 的 tol_deg 说明): 把伺服判据逼紧
            # 再补一轮。预备位是抓杯的起点, 差 7mm 就得让抓取那一步自己补。
            _ok2, e2, _how3 = reach(node, s, T, tol_mm=3.0, passes=2,
                                     tol_deg=0.08)
            if e2 < e:
                e = e2
        errs[s] = round(float(e), 1)
        ok_all = ok_all and ok
        node.get_logger().info(
            'LIFT[%s] 抬臂预备位: 掌心高 %.2fm 前伸 %.2fm 残差 %.1fmm (%s) '
            'q=%s 应力 %.2f'
            % (s, T[2, 3], math.hypot(T[0, 3] - node.base[0],
                                      T[1, 3] - node.base[1]), e, how,
               np.round(q, 2), K.joint_stress(q)))
    if info is not None:
        info['lift_mm'] = errs
    return ok_all, errs


def hand_onto(node, side, model, palm_fn, tol_mm=5.0, elbow='human'):
    """把 side 侧的手掌贴到手里这个物体的对侧, 返回**冻结偏移** inv(T_obj)@T_palm。

    为什么要冻结偏移而不是每拍重解 IK
    ==============================
    附着只有一路(插件把物体钉在**主手**掌心)。如果两只手各自去"抓"一次,
    两路目标会互相打架。所以第二只手只做"跟着走": 贴上去那一刻量一次
    物体→掌心的相对位姿, 之后物体动多少它就动多少。
    """
    T_obj = object_pose(node, model)
    if T_obj is None:
        return None
    reach(node, side, palm_fn(T_obj), tol_mm=tol_mm, elbow=elbow)
    settle_sim(node, 0.30)                      # 等停稳再冻结, 否则量到滑行中的值
    T_obj = object_pose(node, model)
    Tp = wrist(node, side)
    if T_obj is None or Tp is None:
        return None
    return np.linalg.inv(T_obj) @ Tp


def follow_hand(node, side, model, off, tol_mm=5.0, elbow='human'):
    """第二只手按冻结偏移跟上物体当前位置。"""
    T_obj = object_pose(node, model)
    if T_obj is None or off is None:
        return False, 9e9
    Tp_goal = np.asarray(T_obj, float) @ off
    ok, res, _how = reach(node, side, Tp_goal, tol_mm=tol_mm,
                          elbow=elbow)
    return ok, res


def hand_gap(node, side, model):
    """一只手的掌心在**物体坐标系**下的位置 (x, y, z), 米。

    判"双手捧"必须看物体系分量, 不能只看世界距离: 两只手都偏到同一侧时,
    世界距离也可以"很近"。绝对深度(x) 应该一致, 横向(y) 应该**分居两侧**。
    """
    T_obj = object_pose(node, model)
    Tp = wrist(node, side)
    if T_obj is None or Tp is None:
        return None
    p = np.append(Tp[:3, 3], 1.0)                 # 齐次点才能乘 4x4
    d = (np.linalg.inv(T_obj) @ p)[:3]
    return (float(d[0]), float(d[1]), float(d[2]))


# ---------------------------------------------------------------- 倒水流程
def pour_into_cup(node, fill=0.65, jet_len=0.16, ramp=True):
    """P87 的整条倒水链: 抓杯 → 举杯 → 抓壶 → 壶到倒水位 → 水柱 + 水位。

    返回 dict, P87/P88/P89/P91 都从"倒完水"这个状态往下走:
        {'cup': 杯目标位姿, 'kettle': 壶目标位姿, 'tip': 壶嘴尖目标点,
         'ok': 全链是否都成功, 'res': 各步残差}
    """
    out = {'ok': False, 'res': {}, 'arm': {}}
    ok_c, res_c = grasp(node, 'L', 'cup',
                        lambda T: P.cup_grasp_palm(T, 'L'), K.GRIP_CUP)
    out['res']['grasp_cup'] = res_c
    # 用 log_arm 而不是 arm_state: 后者返回**单侧**的状态字典(键是 q/bad/...),
    # 直接塞进 out['arm'] 会让验收脚本把它当成 {side: state} 去遍历, 拿 q 的
    # list 当字典调 .get() 而崩(AttributeError: 'list' object has no attribute 'get')。
    out['arm']['grasp_cup'] = log_arm(node, '抓杯', ('L',))
    if not ok_c:
        return out
    Tcup_pour = P.cup_for_pour()
    # P103: 举杯要走"肘朝外"的解 —— 与捧杯/递杯同一套(P101 的经验)。默认的
    # KDL 种子解在这里会给出肘内收/前臂反向拧的分支, 画面上就是"肘拐进怀里"。
    ok_l, res_l = place_held(node, 'L', 'cup', Tcup_pour, tol_mm=3.0)
    out['res']['lift_cup'] = res_l
    out['arm']['lift_cup'] = log_arm(node, '举杯', ('L',))
    if not ok_l:
        return out
    ok_k, res_k = grasp(node, 'R', 'kettle', P.kettle_grasp_palm, K.GRIP_KETTLE)
    out['res']['grasp_kettle'] = res_k
    if not ok_k:
        return out
    Tk_pour, tip_tgt = P.kettle_for_pour(object_pose(node, 'cup'))
    ok_p, res_p = place_held(node, 'R', 'kettle', Tk_pour, tol_mm=4.0)
    out['res']['kettle_pour'] = res_p
    out['arm']['pour'] = log_arm(node, '倒水', ('L', 'R'))
    if not ok_p:
        return out
    # 水柱先伸出, 水位再升 —— 顺序反了会先看见"杯里有水但没水柱"
    # P98: 有液面控制器时水位由体积积分驱动(水柱由 liquid 逐帧铺),
    # 没有时(离线单点脚本)退回原来的行程插值。
    liq = getattr(node, 'liquid', None)
    if liq is not None:
        liq.pour_start(fill)
    if ramp:
        if not node.carry.joint_ramp('kettle', 'jet_joint', jet_len, dur=0.5):
            # P98: ramp 的 ack 在通道拥塞时可能超时(指令本身多半已到达),
            # 补一发直达终值; 插件每 tick 重发目标, 丢一帧无碍。
            node.carry._send('joint kettle jet_joint %.6f' % jet_len)
        if liq is None:
            node.carry.joint_ramp('cup', 'water_joint',
                                  L.CUP_WATER_MAX * fill, dur=1.2)
    else:
        node.carry.joint('kettle', 'jet_joint', jet_len)
        if liq is None:
            node.carry.joint('cup', 'water_joint', L.CUP_WATER_MAX * fill)
    if liq is not None:
        liq.pour_wait()
    node.spin_for(0.6)
    out.update({'cup': Tcup_pour, 'kettle': Tk_pour, 'tip': tip_tgt,
                'ok': True})
    return out


def pour_approach(node, v_max=0.16, info=None):
    """去倒水的**入场**: 站远 → 抬臂屈肘 → 驶入桌边(用户 2026-10-28)。

    原来这里是 `place_robot(服务位)` 一步到位: 车带着"垂着的手"直接贴到桌前
    0.21m, 然后在桌子上方抬头找壶 —— 既不像人, 手臂上抬的圆弧也压着桌沿。
    现在照人的做法分三步:
        ① 停在离桌沿 `POUR_STANDOFF_GAP`(0.80m > 臂+手全长 0.715m)的位置。
           这一停是**瞬移**到位(它只是布景, 不是要给人看的动作);
        ② 复位后在原地把两条臂抬起来: 肩抬 + 以肘为支点弯, 双手升到 0.82m
           (壶/杯同高)。此时手臂整条在桌子够不到的地方;
        ③ **开**进服务位(0.59m, 画面上看得见车走), 再在服务位上方开始倒水。
    """
    out = {'ok': False}
    T_stand = K.frame(K.rot_z(0.0), (L.POUR_STANDOFF_X, L.SERVE_POSE[1], 0.0))
    d0 = table_gap_m(node)
    # 一律摆到待入场位: 不论车从哪儿来, 每次的入场几何都一致(验收可复现),
    # 而且"停下 → 抬臂 → 开进去"这段动作每次都看得见。
    place_robot(node, T_stand)
    node.home()                       # 在远处复位(臂 Q_REST、十指张开、水流归零)
    out['gap_before'] = round(table_gap_m(node), 3)
    out['gap_at_arrival'] = round(d0, 3)
    out['reach_m'] = K.ARM_REACH
    ok_l, errs = lift_ready(node)
    out['lift'] = errs
    node.get_logger().info(
        'POUR 入场: 车→桌沿 %.2fm(臂+手 %.2fm, 余量 %+.0fmm) 双手抬到 %.2fm'
        % (out['gap_before'], K.ARM_REACH,
           (out['gap_before'] - K.ARM_REACH) * 1e3, K.READY_Z))
    ok_d, res_d = drive_to(node, (L.SERVE_POSE[0], L.SERVE_POSE[1]),
                           v_max=v_max, tol_mm=25.0, v_min=0.03,
                           timeout=30.0, tag='驶入桌边')
    # 行驶有 ~20mm 的停稳残差, 而后面所有"以 base 为准"的位形都吃这个误差,
    # 所以在服务位上方再摆正一次(几厘米的瞬移, 画面上看不出来)。
    snap = place_robot(node, K.frame(K.rot_z(0.0),
                                     (L.SERVE_POSE[0], L.SERVE_POSE[1], 0.0)))
    out.update(gap_after=round(table_gap_m(node), 3),
               drive_mm=round(res_d, 1), snap_mm=round(snap, 1),
               ok=bool(ok_l and ok_d))
    if info is not None:
        info.update(out)
    return out


def prepare_present(node, fill=0.65, verbose=True):
    """一路做到"双手捧杯、面向人"(= P88 的终态), 返回 dict。

    P89/P91 都从这里往下走, 所以把 P88 那一串固定动作打包, 免得三份脚本各写
    一遍、然后各错一处:
        摆正底盘 → home → 倒水 → 收水 → 放壶 → 右臂回 home → 摆正底盘
        → 双手捧杯(左握 + 右贴) → 原地转身面向人
    注意**摆正底盘做了两次**: 倒水那段要跑一分多钟, 轮子会被机械臂的反作用力
    一点点推走(§6.4 #35), 不摆正的话后面所有"以 base 为准"的位形都会偏。
    """
    T_serve = K.frame(K.rot_z(0.0), (L.SERVE_POSE[0], L.SERVE_POSE[1], 0.0))
    out = {'ok': False}
    # P102: 入场改成"站远 → 抬臂屈肘 → 驶入桌边"(见 pour_approach)。原来的
    # `place_robot(T_serve) + home()` 是带着垂着的手一步贴到桌前, 用户要求
    # 先在臂长之外把双手抬到壶/杯高度, 再靠近。
    out['entry'] = pour_approach(node)
    if not out['entry']['ok']:
        node.get_logger().warn('PREPARE 入场: 抬臂或驶入有告警, 继续尝试倒水')
    Tk_home = object_pose(node, 'kettle')
    pour = pour_into_cup(node, fill=fill)
    out.update(pour)
    if not pour['ok']:
        return out
    # 收水: 水柱缩回, 水位保持
    node.carry.joint_ramp('kettle', 'jet_joint', 0.0, dur=0.5)
    if getattr(node, 'liquid', None) is not None:
        node.liquid.pour_stop()      # P98: 收水完成, 水柱熄灭
    ok_d, res_d = put_down(node, 'R', 'kettle', Tk_home)
    out['res']['kettle_back'] = res_d
    node.arm['R'].move(K.Q_REST, timeout=10.0)
    node.spin_for(0.4)
    place_robot(node, T_serve)
    bx, by, byaw = node.base
    Tcup_carry = P.cup_in_base(bx, by, byaw, L.CUP_CARRY_R, L.CUP_CARRY_Z)
    # P101: 捧杯/贴手都走"肘朝外"解 —— 抱着杯子时肘得像人一样架在体侧,
    # 而不是垂在下面(原来 KDL 用 home 当种子, 两侧肘都朝下)。
    ok_c, res_c = place_held(node, 'L', 'cup', Tcup_carry, tol_mm=5.0)
    out['res']['cup_carry'] = res_c
    off_r = hand_onto(node, 'R', 'cup', lambda T: P.cup_cradle_palm(T, 'R'))
    # P94: 转身朝向优先用**实测人的位置**, 人不在接水位附近才退回名义
    # PERSON_SPOT。原来写死 PERSON_SPOT: 人站偏 0.15m 转身就差 9.3°, 后面
    # `_present_target` 虽然会按实测人手方向把杯子送过去, 但机器人是斜着
    # 对人的 —— 既别扭, 也白白吃掉手臂的横向余量。
    #   只认"站在接水位 ±0.5m 内"的人: test_p88 那类脚本里人根本不在场
    #   (停在初始位姿), 那时用实测位置会转向错误的方向。
    _Tp = object_pose(node, 'person')
    yaw_face = None
    if _Tp is not None:
        if math.hypot(_Tp[0, 3] - L.PERSON_SPOT[0],
                      _Tp[1, 3] - L.PERSON_SPOT[1]) < 0.50:
            yaw_face = math.atan2(_Tp[1, 3] - node.base[1],
                                  _Tp[0, 3] - node.base[0])
    if yaw_face is None:
        yaw_face = math.atan2(L.PERSON_SPOT[1] - node.base[1],
                              L.PERSON_SPOT[0] - node.base[0])
    ok_t, err_t = turn_base(node, yaw_face)
    # 转身是 cmd_vel 原地转, 轮子会平移漂 3~7cm (§6.4 #35 的同一个病)。而
    # 递杯位是"从 base 前伸 0.62"算的、人手点却是世界固定点 —— 不摆正的话
    # 两者不共线, 实测杯子会停在离人手 65.9mm 的地方。转完把底盘摆回服务位
    # (**朝向保留**), 让 base 与人手点严格落在同一条射线上。
    T_face = K.frame(K.rot_z(yaw_face),
                     (L.SERVE_POSE[0], L.SERVE_POSE[1], 0.0))
    place_robot(node, T_face)
    out.update({'ok': bool(ok_d and ok_c and off_r is not None and ok_t),
                'turn_ok': ok_t, 'turn_err': err_t,
                'kettle_home': Tk_home,
                'cup': object_pose(node, 'cup'),
                'off_r': off_r})
    if not out['ok']:
        # P98 补: 四个收尾标志谁挂了原来不留痕, FAIL_PREPARE 只能靠猜
        node.get_logger().error(
            'PREPARE flags: put_down=%s cup_carry=%s hand_onto=%s '
            'turn=%s(err=%.1f deg) base=%s'
            % (ok_d, ok_c, off_r is not None, ok_t,
               math.degrees(err_t) if err_t is not None else -1.0,
               np.round(node.base, 3)))
    return out


def cup_present_target(node, r=None, z=None):
    """递杯目标位形: 用**真值 base** 现算, 杯子在机器人正前方 r 处。

    `P.present_cup(bx,by,byaw)` 只在 base 恰好等于名义服务位时才等价; 底盘一
    漂就整体错位(§6.4 #35), 所以这里统一走真值 base。
    """
    bx, by, byaw = node.base
    return P.cup_in_base(bx, by, byaw,
                         L.CUP_PRESENT_R if r is None else r,
                         L.CUP_PRESENT_Z if z is None else z)


def present_target_at_hand(node):
    """递杯目标: **朝实测人手**递, 而不是固定朝 base 的 +x。

    人正好站在 `PERSON_SPOT` 时两者等价(机器人前伸 0.62 = 人身前 0.30 的
    手); 但只要车身在服务位附近漂了 3~7cm(转身必然漂, §6.4 #35),
    "朝 +x 前伸 0.62" 就把这 3~7cm 原样落在杯子上 —— 实测杯子离人手
    34.6mm。P96 判的是"手到杯轴", 这个偏差会直接把两只手的间距推到
    123/62mm(相切值 92), 一眼看上去像"手没捧住", 其实是目标取错了。

    所以: 方向指向实测人手、距离取 min(前伸上限, 车→手)。杯柄朝向由
    `cup_in_base` 跟着这个方向走, 仍然是"柄朝人"。
    """
    bx, by, byaw = node.base
    Th = person_hand_world(node, object_pose(node, 'person'))
    if Th is None:
        return cup_present_target(node)
    dx, dy = Th[0, 3] - bx, Th[1, 3] - by
    d_hb = math.hypot(dx, dy)
    if d_hb < 0.30:            # 数据不可信(人贴太近), 退回名义位形
        return cup_present_target(node)
    # P94: 上限从 CUP_PRESENT_R(0.62) 放宽到 CUP_PRESENT_R_MAX(0.70) ——
    # 0.62 是"人站准"的解; 人往外站 0.15m 时车→人手 0.77m, 卡在 0.62
    # 会把杯子停在离手 150mm 处。0.70 是 diag_p94_reach.py 扫出来的
    # 安全上限(再远就顶臂长)。不够的部分由 `_approach_if_far` 补。
    r = max(0.50, min(L.CUP_PRESENT_R_MAX, d_hb))
    return P.cup_in_base(bx, by, math.atan2(dy, dx), r, L.CUP_PRESENT_Z)


def person_hand_world(node, T_person, link='person_arms', local_off=None):
    """人的双手世界位姿: 人的模型位姿 ∘ (link 相对模型的位姿) ∘ 手点偏置。

    P86 #22: 这个话题里 link 给的是**相对所属模型**的位姿。`person_arms`
    还是个**关节子 link**(绕 `person_arm_joint` 转), 所以只有"模型 ∘ link"
    才是它的世界位姿, 直接拿 link 当世界位姿会错到 1 米开外。

    P89: `person_arms` 的**原点在肩关节**, 钉"两手之间"必须再叠
    `scene_layout.PERSON_HAND_IN_ARM`; 默认就取它, 省得每处调用都忘。
    """
    if local_off is None:
        local_off = getattr(L, 'PERSON_HAND_IN_ARM', None)
    p = node.links.get(link)
    if p is None or T_person is None:
        return None
    q = p.orientation
    Tl = K.frame(K.quat_to_mat(q.x, q.y, q.z, q.w),
                 (p.position.x, p.position.y, p.position.z))
    T = np.asarray(T_person, float) @ Tl
    if local_off is not None:
        T = T @ K.trans(local_off)
    return T


def person_hands(node, openness, dur=0.6, dt=0.05, tag=''):
    """人的两只手开合（斜坡，别瞬移）。

    P96：人的两条手臂各有自己的外展关节（绕肩点的竖直轴）。`openness` 是
    **幅度**，两侧发**反号**值 —— 两条关节的轴在 SDF 里都写 (0,0,1)，
    不反号就会变成"左手张开、右手更合拢"（§6.4 #60）。

    为什么走斜坡：位置指令是跳变的，直接给终点画面上就是手"啪"地弹开，
    而杯子还在机器人手里 —— 看上去像手被电了一下。0.6s 走完 0.385rad，
    单关节角速度 0.64rad/s，远在 JPC 的 cmd_max(2.0) 之下。
    """
    cur = float(node.bus.val.get('person_arm_l_joint', 0.0))
    n = max(1, int(dur / max(dt, 1e-3)))
    for i in range(1, n + 1):
        v = cur + (float(openness) - cur) * float(i) / n
        node.bus.set(['person_arm_l_joint', 'person_arm_r_joint'], [v, -v])
        node.spin_for(dt)
    node.spin_for(0.25)          # 让 JPC 把最后一段走完再返回
    node.get_logger().info(
        '双手%s: 幅度 %.3f rad (左 %+.3f / 右 %+.3f)'
        % (tag, openness, openness, -openness))
    return True


def person_hand_poses(node, T_person=None):
    """两只手心的**世界位姿**（真值），取自各自的手臂 link。

    注意钉杯子用的是 `person_arms`（肩架），手是它的子 link —— 钉点不跟手
    走，这是 P96 的约束 1。所以"手到杯子多远"必须单独从两条手臂 link 量，
    不能拿 `PERSON_HAND_IN_ARM` 那个中点当手。
    """
    if T_person is None:
        T_person = object_pose(node, 'person')
    Tl = person_hand_world(node, T_person, link='person_arm_l',
                           local_off=L.PERSON_HAND_IN_ARML)
    Tr = person_hand_world(node, T_person, link='person_arm_r',
                           local_off=L.PERSON_HAND_IN_ARMR)
    return Tl, Tr


def hand_gap_mm(node, T_person=None):
    """两手心的间距(mm)，真值测量。合拢 ≈184mm，张开 ≈345mm。"""
    Tl, Tr = person_hand_poses(node, T_person)
    if Tl is None or Tr is None:
        return None
    return float(np.linalg.norm(Tl[:3, 3] - Tr[:3, 3])) * 1000.0


def hands_to_cup_mm(node, T_person=None, T_cup=None):
    """(左手到杯轴, 右手到杯轴, 手杯高差) 三项，单位 mm，全真值。

    "到杯轴"取**水平**距离：杯是竖直的圆柱，手从两侧捧，横向间距才是
    "捧住了 / 穿模了 / 没够着"的判据；高度差单独给，用来判手有没有捧在
    杯身高度上（捧到杯口上方等于没捧）。
    """
    if T_cup is None:
        T_cup = object_pose(node, 'cup')
    Tl, Tr = person_hand_poses(node, T_person)
    if Tl is None or Tr is None or T_cup is None:
        return None
    c = T_cup[:3, 3]
    dl = float(np.hypot(Tl[0, 3] - c[0], Tl[1, 3] - c[1])) * 1000.0
    dr = float(np.hypot(Tr[0, 3] - c[0], Tr[1, 3] - c[1])) * 1000.0
    dz = float(abs(Tl[2, 3] - c[2])) * 1000.0
    return dl, dr, dz
