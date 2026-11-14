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


def _base_pose(node, max_truth_age_s=0.5):
    """优先用新鲜 Gazebo robot 模型真值刷新底盘位姿，陈旧时保留原值。

    odom 是轮速积分，不能覆盖 Gazebo 模型真值；但此前 truth 回调只保存
    snapshot，reach/wrist 仍可能继续使用旧 node.base。机械臂移动期间底盘
    受反作用力发生微动时，这会让世界系 FK/IK 使用过期底座。
    """
    snapshot = getattr(node, 'truth_base_pose_snapshot', None)
    if snapshot is not None:
        try:
            pose, sample_t = snapshot
            age = time.monotonic() - float(sample_t)
            candidate = tuple(float(v) for v in pose)
            if (0.0 <= age <= float(max_truth_age_s)
                    and len(candidate) == 3
                    and all(math.isfinite(v) for v in candidate)):
                node.base = candidate
                node._truth_base = True
                node._have_odom = True
        except (TypeError, ValueError, OverflowError):
            pass
    return tuple(float(v) for v in node.base)


def wrist(node, side):
    """掌心世界位姿：实测关节角 → fk → 最新可信底盘位姿。"""
    q = q_meas(node, side)
    if q is None:
        return None
    return K.fk_world(side, q, _base_pose(node))


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

def _dynamic_pose_matrix(pose):
    """Convert one Gazebo dynamic_pose/info pose message to a homogeneous matrix."""
    q = pose.orientation
    return K.frame(K.quat_to_mat(q.x, q.y, q.z, q.w),
                   (pose.position.x, pose.position.y, pose.position.z))


def fk_truth_diagnostics(node, side, q=None):
    """Compare joint FK against base and Gazebo palm truth without changing control.

    dynamic_pose/info link poses have historically appeared in either world or
    model-relative form across tools/consumers. Report both interpretations so
    a residual cannot be misdiagnosed as an IK/servo problem or hidden by
    changing tolerances.
    """
    if q is None:
        q = q_meas(node, side)
    if q is None:
        return {'reason': 'joint_state_missing'}
    q = np.asarray(q, float)
    base = tuple(float(v) for v in node.base)
    T_fk = K.fk_world(side, q, base)
    _diag_now = time.monotonic()
    _js_t = float(getattr(node, 'js_sample_t', 0.0) or 0.0)
    _truth_pose = getattr(node, 'truth_base_pose_snapshot', None)
    _truth_pose_t = (float(_truth_pose[1]) if _truth_pose is not None else 0.0)
    result = {
        'base': [round(v, 4) for v in base],
        'fk_xyz': np.round(T_fk[:3, 3], 4).tolist(),
        'joint_state_age_s': (round(max(0.0, _diag_now - _js_t), 3)
                              if _js_t else None),
        'truth_base_pose_age_s': (
            round(max(0.0, _diag_now - _truth_pose_t), 3)
            if _truth_pose_t else None),
        'truth_base_pose_xyz_yaw': (
            [round(float(v), 5) for v in _truth_pose[0]]
            if _truth_pose is not None else None),
    }
    # Odometry pose can be stale after teleport; instantaneous wheel twist is
    # still valuable evidence for distinguishing chassis drift from FK error.
    odom_twist = getattr(node, 'odom_twist', None)
    if odom_twist is not None:
        result['odom_twist'] = [round(float(v), 5) for v in odom_twist]
        twist_t = float(getattr(node, 'odom_twist_t', 0.0) or 0.0)
        result['odom_twist_age_s'] = round(
            max(0.0, time.monotonic() - twist_t), 3) if twist_t else None
    truth_twist = getattr(node, 'truth_base_twist', None)
    if truth_twist is not None:
        result['truth_base_twist'] = [
            round(float(v), 5) for v in truth_twist]
        truth_t = float(getattr(node, 'truth_base_twist_t', 0.0) or 0.0)
        result['truth_base_twist_age_s'] = round(
            max(0.0, time.monotonic() - truth_t), 3) if truth_t else None
    wheel_velocity = getattr(node, 'js_velocity', {})
    if wheel_velocity:
        result['wheel_joint_velocity'] = {
            name: round(float(wheel_velocity[name]), 5)
            for name in ('left_wheel_joint', 'right_wheel_joint')
            if name in wheel_velocity}
    T_robot = object_pose(node, 'robot')
    if T_robot is not None:
        model_yaw = math.atan2(T_robot[1, 0], T_robot[0, 0])
        yaw_gap = math.atan2(math.sin(base[2] - model_yaw),
                             math.cos(base[2] - model_yaw))
        result['base_truth_xy_mm'] = round(
            float(np.linalg.norm(np.asarray(base[:2]) - T_robot[:2, 3])) * 1e3, 2)
        result['base_truth_yaw_deg'] = round(math.degrees(yaw_gap), 3)
    link = getattr(node, 'links', {}).get('%s_palm' % side)
    if link is not None:
        T_link_raw = _dynamic_pose_matrix(link)
        result['palm_link_raw_xyz'] = np.round(T_link_raw[:3, 3], 4).tolist()
        result['fk_to_palm_raw_mm'] = round(pos_err_mm(T_fk, T_link_raw), 2)
        result['fk_to_palm_raw_deg'] = round(axis_angle_deg(T_fk, T_link_raw), 3)
        if T_robot is not None:
            T_link_model = T_robot @ T_link_raw
            result['palm_link_model_xyz'] = np.round(
                T_link_model[:3, 3], 4).tolist()
            result['fk_to_palm_model_mm'] = round(
                pos_err_mm(T_fk, T_link_model), 2)
            result['fk_to_palm_model_deg'] = round(
                axis_angle_deg(T_fk, T_link_model), 3)
    else:
        result['palm_link_truth'] = 'missing'
    return result



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
#   R4 肩内旋过大 |up_roll| > 2.00 rad(114.6°, 机械限位 2.00)
#   R5 前臂扭转   |fore_roll + wrist_r| > 2.00 rad(115°) —— 人的旋前+旋后
#                 合计约 180°, 但那是"特意去拧"; 端杯子不需要, 而老病态位形
#                 是 171°(2.98), 2.00 拦得住、正常抓杯(1.7~1.9)放得过
#   R6 腕反折     |wrist_p| > 1.50 rad(86°) —— 正好是本臂机械限位, 这条实际
#                 判的是"顶到限位", 而不是"屈伸看着别扭"
#   R7 肘过折     |elbow| > 2.25 rad(129°)
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


def human_kinematics_ok(states, min_out_line_mm=None,
                        max_abs_droop_mm=None, max_abs_twist=None,
                        required_sides=None):
    """关键搬运动作必须具备指定侧的实测姿态，并通过 P103 规则/质量门槛。

    required_sides: 此动作必须有姿态数据的手臂侧，如抓杯('L')、倒水('L','R')。
    min_out_line_mm: 可选的逐侧肘外扩线下限（mm），如举杯左臂 100、倒水右臂 60。
    max_abs_droop_mm/max_abs_twist: 可选的逐侧绝对值上限（mm/rad）。
    """
    if not states:
        return False
    if required_sides is not None and any(
            side not in states or not isinstance(states[side], dict)
            for side in required_sides):
        return False
    min_out_line_mm = min_out_line_mm or {}
    max_abs_droop_mm = max_abs_droop_mm or {}
    max_abs_twist = max_abs_twist or {}
    for side, st in states.items():
        if not isinstance(st, dict) or st.get('bad'):
            return False
        if side in min_out_line_mm:
            value = st.get('out_line_mm')
            if value is None or float(value) < float(min_out_line_mm[side]):
                return False
        if side in max_abs_droop_mm:
            value = st.get('droop_mm')
            if value is None or abs(float(value)) > float(max_abs_droop_mm[side]):
                return False
        if side in max_abs_twist:
            value = st.get('twist')
            if value is None or abs(float(value)) > float(max_abs_twist[side]):
                return False
    return True


# ---------------------------------------------------------------- 桌面/桌腿碰撞门控（P104）
def _segment_hits_aabb(p0, p1, lo, hi, radius=0.0):
    """胶囊线段与 AABB 的保守相交：将盒子按半径外扩后做 slab 检查。"""
    lo = np.asarray(lo, float) - float(radius)
    hi = np.asarray(hi, float) + float(radius)
    a = np.asarray(p0, float)
    d = np.asarray(p1, float) - a
    t0, t1 = 0.0, 1.0
    for k in range(3):
        if abs(d[k]) < 1e-10:
            if a[k] < lo[k] or a[k] > hi[k]:
                return False
            continue
        u0 = (lo[k] - a[k]) / d[k]
        u1 = (hi[k] - a[k]) / d[k]
        if u0 > u1:
            u0, u1 = u1, u0
        t0 = max(t0, u0)
        t1 = min(t1, u1)
        if t0 > t1:
            return False
    return True


def _table_collision_boxes():
    """与 work_area.sdf 同源的桌面板及四条桌腿 AABB（世界坐标）。"""
    cx, cy = L.TABLE_XY
    boxes = [
        ('table_top', (cx - L.TABLE_W / 2.0, cy - L.TABLE_D / 2.0,
                       L.TABLE_TOP_Z - L.TABLE_PLATE_T),
         (cx + L.TABLE_W / 2.0, cy + L.TABLE_D / 2.0, L.TABLE_TOP_Z)),
        # SDF 中台面外沿 lip 只有 visual，但也纳入 10mm 净空，避免视觉穿模。
        ('table_lip', (cx - L.TABLE_W / 2.0 - 0.025,
                       cy - L.TABLE_D / 2.0 - 0.020,
                       L.TABLE_TOP_Z - L.TABLE_PLATE_T - 0.009),
         (cx + L.TABLE_W / 2.0 + 0.025,
          cy + L.TABLE_D / 2.0 + 0.020,
          L.TABLE_TOP_Z - L.TABLE_PLATE_T + 0.009)),
    ]
    for sx in (-0.410, 0.410):
        for sy in (-0.270, 0.270):
            x, y = cx + sx, cy + sy
            boxes.append(('table_leg', (x - 0.025, y - 0.025, 0.0),
                          (x + 0.025, y + 0.025, L.TABLE_TOP_Z - L.TABLE_PLATE_T)))
    return boxes


def _obb_hits_aabb(center, rotation, half_extents, lo, hi, margin=0.015):
    """OBB-AABB SAT 相交测试；margin 只扩张桌面盒，避免球形手掌假碰撞。"""
    c_a = np.asarray(center, float)
    axes = np.asarray(rotation, float)
    h_a = np.asarray(half_extents, float)
    lo = np.asarray(lo, float) - float(margin)
    hi = np.asarray(hi, float) + float(margin)
    c_b = 0.5 * (lo + hi)
    h_b = 0.5 * (hi - lo)
    # SAT: 3 个 OBB 轴、3 个 AABB 轴、9 个叉积轴。
    # axes 的列是 OBB 局部 x/y/z 轴在世界系中的方向；
    # SAT 公式使用 R[i,j] = dot(OBB_axis_i, AABB_axis_j)，所以必须转置。
    R = axes.T
    abs_R = np.abs(R) + 1e-9
    t_world = c_b - c_a
    t = R @ t_world
    for i in range(3):
        ra = h_a[i]
        rb = float(h_b @ abs_R[i, :])
        if abs(t[i]) > ra + rb:
            return False
    for j in range(3):
        ra = float(h_a @ abs_R[:, j])
        rb = h_b[j]
        if abs(float(t_world[j])) > ra + rb:
            return False
    for i in range(3):
        for j in range(3):
            i1, i2 = (i + 1) % 3, (i + 2) % 3
            j1, j2 = (j + 1) % 3, (j + 2) % 3
            ra = h_a[i1] * abs_R[i2, j] + h_a[i2] * abs_R[i1, j]
            rb = h_b[j1] * abs_R[i, j2] + h_b[j2] * abs_R[i, j1]
            value = abs(t[i2] * R[i1, j] - t[i1] * R[i2, j])
            if value > ra + rb:
                return False
    return True


def _arm_geometry_world(side, q, base):
    """由 FK 生成连杆胶囊、掌部 OBB 与逐指胶囊，避免把手指按 55mm 球体膨胀。"""
    Tb = K.base_pose(*base)
    Tp = K.fk(side, np.asarray(q, float))
    R = Tp[:3, :3]
    p_palm = Tp[:3, 3]
    p_wrist = p_palm + R @ np.array([0.0, 0.0, K.PALM_OFF])
    p_shoulder = np.asarray(K.SHOULDER[side], float)
    p_elbow = K.elbow_pos(side, q)
    to_world = lambda p: (Tb @ K.trans(p))[:3, 3]
    sh, el, wr, palm = [to_world(p) for p in
                        (p_shoulder, p_elbow, p_wrist, p_palm)]
    T_palm_center = to_world(p_palm + R @ np.asarray(K.PALM_CENTER, float))
    R_world = Tb[:3, :3] @ R

    segments = [
        ('upper_arm', sh, el, 0.034),
        ('forearm', el, wr, 0.030),
        ('wrist', wr, palm, 0.030),
    ]
    # Shadow Hand 的四指分别按几何轴线检查，半径按实际 14mm 指宽取值。
    for index, x in enumerate(K.FINGER_X):
        p0 = p_palm + R @ np.array([float(x), 0.0, K.FINGER_BASE_Z])
        p1 = p_palm + R @ np.array([
            float(x), 0.0,
            K.FINGER_BASE_Z - K.PROX_LEN - K.DIST_LEN])
        segments.append(('finger_%d' % index, to_world(p0), to_world(p1),
                         K.FINGER_W / 2.0))
    # 拇指另算，避免四指包络忽略掌侧的拇指扫掠。
    thx = float(K.THUMB_X[side])
    th0 = p_palm + R @ np.array([thx, K.THUMB_BASE[1],
                                 K.THUMB_BASE[2]])
    th1 = p_palm + R @ np.array([
        thx, K.THUMB_BASE[1],
        K.THUMB_BASE[2] - K.PROX_LEN - K.DIST_LEN])
    segments.append(('thumb', to_world(th0), to_world(th1), 0.012))
    palm_obb = (T_palm_center, R_world, 0.5 * np.asarray(K.PALM_SIZE, float))
    return segments, palm_obb


def arm_path_table_clear(node, side, path_q, base=None,
                         ignore_initial=True, max_joint_step=0.02,
                         q_start=None):
    """收臂路径逐点/插值检查桌面和桌腿；返回 (clear, first_collision)。

    base/q_start 可由 reach 在 IK 时刻冻结后传入，确保 IK、路径插值起点
    与世界系碰撞几何使用同一组状态快照；外部旧调用仍沿用实时状态。
    """
    if base is None:
        base = node.base
    q0 = (q_meas(node, side) if q_start is None
          else np.asarray(q_start, float).copy())
    if q0 is None:
        return False, {'reason': 'joint_state_missing'}
    if q0.shape != (K.NJ,) or not np.all(np.isfinite(q0)):
        return False, {'reason': 'joint_state_invalid'}
    samples = []
    prev = np.asarray(q0, float)
    for target in path_q:
        target = np.asarray(target, float)
        n = max(1, int(math.ceil(
            float(np.max(np.abs(target - prev))) / max_joint_step)))
        for i in range(1, n + 1):
            samples.append(prev + (target - prev) * (i / float(n)))
        prev = target
    if not ignore_initial:
        samples.insert(0, np.asarray(q0, float))
    boxes = _table_collision_boxes()
    for i, q in enumerate(samples):
        segments, palm = _arm_geometry_world(side, q, base)
        pc, pr, ph = palm
        for box_name, lo, hi in boxes:
            if _obb_hits_aabb(pc, pr, ph, lo, hi, margin=0.015):
                return False, {'sample': i, 'segment': 'palm_obb',
                               'box': box_name}
            for seg_name, p0, p1, radius in segments:
                check_radius = float(radius) + 0.010
                if _segment_hits_aabb(p0, p1, lo, hi, check_radius):
                    # Preserve the exact world-space geometry behind a
                    # fail-closed decision. This is diagnostic only: the
                    # collision proxy and acceptance threshold are unchanged.
                    return False, {
                        'sample': i, 'segment': seg_name, 'box': box_name,
                        'radius_m': float(radius),
                        'check_radius_m': check_radius,
                        'segment_start_m': np.round(
                            np.asarray(p0, float), 5).tolist(),
                        'segment_end_m': np.round(
                            np.asarray(p1, float), 5).tolist(),
                        'box_min_m': np.round(np.asarray(lo, float), 5).tolist(),
                        'box_max_m': np.round(np.asarray(hi, float), 5).tolist(),
                        'expanded_box_min_m': np.round(
                            np.asarray(lo, float) - check_radius, 5).tolist(),
                        'expanded_box_max_m': np.round(
                            np.asarray(hi, float) + check_radius, 5).tolist(),
                    }
    return True, None


def _wait_fresh_joint_state(node, timeout=2.0, max_age=0.5):
    """Wait for a recent joint-state sample before using it as an IK seed.

    Cached finite joint values are not sufficient: a stale sample can select
    an obsolete redundant-arm branch and send the servo toward an unsafe pose.
    Wait briefly for the ROS callback to refresh; if it does not, fail closed.
    """
    t0 = time.monotonic()
    age = None
    while True:
        sample_t = float(getattr(node, 'js_sample_t', 0.0) or 0.0)
        now = time.monotonic()
        age = max(0.0, now - sample_t) if sample_t > 0.0 else None
        if age is not None and age <= max_age:
            return True, age
        remaining = timeout - (now - t0)
        if remaining <= 0.0:
            return False, age
        spin_for = getattr(node, 'spin_for', None)
        if callable(spin_for):
            spin_for(min(0.05, remaining))
        else:
            time.sleep(min(0.01, remaining))


def _wait_base_still(node, timeout=3.0, max_age=0.25,
                     linear_mps=0.005, angular_rps=math.radians(0.5),
                     stable_for=0.20):
    """Wait for fresh Gazebo chassis twist to remain below strict stillness limits."""
    started, stable_since, last = time.monotonic(), None, None
    while True:
        now = time.monotonic()
        twist = getattr(node, 'truth_base_twist', None)
        sample_t = float(getattr(node, 'truth_base_twist_t', 0.0) or 0.0)
        age = now - sample_t if sample_t > 0.0 else float('inf')
        if twist is not None and 0.0 <= age <= max_age:
            try:
                vx, vy, wz = (float(v) for v in twist[:3])
                linear, angular = math.hypot(vx, vy), abs(wz)
                if all(math.isfinite(v) for v in (linear, angular)):
                    last = {'age_s': age, 'linear_mps': linear, 'angular_rps': angular}
                    if linear <= linear_mps and angular <= angular_rps:
                        if stable_since is None:
                            stable_since = now
                        if now - stable_since >= stable_for:
                            return True, last
                    else:
                        stable_since = None
                else:
                    stable_since = None
            except (TypeError, ValueError, OverflowError, IndexError):
                stable_since = None
        else:
            stable_since = None
        remaining = timeout - (now - started)
        if remaining <= 0.0:
            return False, last
        spin_for = getattr(node, 'spin_for', None)
        if callable(spin_for):
            spin_for(min(0.02, remaining))
        else:
            time.sleep(min(0.01, remaining))


# ---------------------------------------------------------------- 到位
def _reach_impl(node, side, T_world, tol_mm=3.0, passes=len(TOL_SCHED),
          timeout=10.0, elbow='human', tol_deg=None, check_table=False,
          preflight_only=False, require_base_still=False):
    """闭环把 side 侧手臂的**掌心**移到世界位形 T_world。

    每轮 IK 和伺服后掌心测量均优先刷新同一接收端时钟下的新鲜 Gazebo
    robot 模型真值，避免底盘受机械臂反作用微动时继续使用旧 node.base。

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

    `preflight_only=True` 是**无动作预检**：只求 IK 并检查整段插值路径，
    必须同时设置 `check_table=True`；路径安全后直接返回，不向关节伺服下发目标。

    返回 (是否到位/预检安全, 实测或预测位置残差 mm, 最后一轮的求解来源标签)。
    """
    # 调试/对照用: P103_ELBOW=none|out|human 可临时覆盖分支策略
    # (验收与回归都跑默认值, 这个开关只给 A/B 对比留口子)
    _ov = os.environ.get('P103_ELBOW')
    if _ov:
        elbow = None if _ov == 'none' else _ov
    if preflight_only and not check_table:
        return False, 9e9, 'preflight-requires-table-check'
    res = 9e9
    how = '-'
    first_how = None
    pass_trace = []
    q_target_last = None
    for it in range(passes):
        if require_base_still:
            still, base_motion = _wait_base_still(node)
            if not still:
                node.get_logger().warn(
                    'REACH %s 底盘未在 3.0s 内稳定；停止精调，保持抓持；last_twist=%s'
                    % (side, base_motion))
                return False, res, 'base-not-still'
            node.get_logger().info(
                'REACH %s BASE_STILL pass=%d linear=%.4fm/s yaw_rate=%.4fdeg/s truth_age=%.3fs'
                % (side, it + 1, base_motion['linear_mps'],
                   math.degrees(base_motion['angular_rps']), base_motion['age_s']))
        # Never derive a redundant-arm IK branch from cached joint values.
        # In E2E run #114 the second-round joint cache was 60.654s old while
        # Gazebo base truth continued updating; the stale seed led to a
        # 100.51deg servo miss. Refresh first, otherwise preserve the grasp.
        fresh_js, js_age = _wait_fresh_joint_state(node)
        if not fresh_js:
            node.get_logger().warn(
                'REACH %s joint_state 过期 %.3fs，等待刷新超时；拒绝使用旧关节状态求 IK'
                % (side, js_age if js_age is not None else float('inf')))
            return False, res, 'joint-state-stale'
        q_now = q_meas(node, side)
        if q_now is None:
            return False, res, how
        # **只在第一轮挑分支**, 之后就地跟踪。7 轴臂的等价解族里, 每轮都重新
        # 挑"最像人"的那个会让手臂在两个分支之间来回跳 —— 实测抓杯时两轮各
        # 选一边, 掌心在中间不动, 残差 115mm 一直不下去。分支是姿态策略,
        # 补位是精度问题, 两件事分开: 第一轮定分支, 后面只用种子做局部修正。
        _mode = elbow if it == 0 else None
        # Freeze the exact receive-side evidence used by this IK iteration.
        # The later post-servo ages alone cannot prove that the base pose used
        # by FK/IK was aligned with the latest Gazebo truth snapshot.
        ik_sample_t = time.monotonic()
        base_at_ik = _base_pose(node)
        js_sample_t_at_ik = float(getattr(node, 'js_sample_t', 0.0) or 0.0)
        truth_snapshot_at_ik = getattr(
            node, 'truth_base_pose_snapshot', None)
        truth_pose_at_ik = (
            tuple(float(v) for v in truth_snapshot_at_ik[0])
            if truth_snapshot_at_ik is not None else None)
        truth_pose_t_at_ik = (
            float(truth_snapshot_at_ik[1])
            if truth_snapshot_at_ik is not None else 0.0)
        # Snapshot chassis motion and both command channels at the exact IK
        # boundary. Post-servo telemetry alone cannot establish who moved the
        # base while a multi-pass reach was converging.
        truth_twist_at_ik = getattr(node, 'truth_base_twist', None)
        truth_twist_t_at_ik = float(
            getattr(node, 'truth_base_twist_t', 0.0) or 0.0)
        drink_cmd_at_ik = getattr(node, '_drink_cmd_snapshot', None)
        robot_cmd_at_ik = getattr(node, '_robot_cmd_snapshot', None)
        base_truth_xy_mm_at_ik = None
        base_truth_yaw_deg_at_ik = None
        if truth_pose_at_ik is not None:
            base_truth_xy_mm_at_ik = float(np.linalg.norm(
                np.asarray(base_at_ik[:2]) -
                np.asarray(truth_pose_at_ik[:2])) * 1e3)
            _yaw_gap = math.atan2(
                math.sin(base_at_ik[2] - truth_pose_at_ik[2]),
                math.cos(base_at_ik[2] - truth_pose_at_ik[2]))
            base_truth_yaw_deg_at_ik = math.degrees(_yaw_gap)
        q, _e, how = K.ik_world(side, T_world, base_at_ik, q_now, elbow=_mode)
        T_ik_fk = K.fk_world(side, q, base_at_ik)
        ik_fk_mm = pos_err_mm(T_ik_fk, T_world)
        ik_fk_deg = axis_angle_deg(T_ik_fk, T_world)
        if first_how is None:
            first_how = how
        q_target_last = q.copy()
        if check_table:
            def _path_clear(q_candidate):
                _n = max(1, int(math.ceil(
                    float(np.max(np.abs(q_candidate - q_now))) / 0.02)))
                _path = [q_now + (q_candidate - q_now) * (i / float(_n))
                         for i in range(1, _n + 1)]
                return arm_path_table_clear(
                    node, side, _path, base=base_at_ik,
                    ignore_initial=True, q_start=q_now)

            _clear, _hit = _path_clear(q)
            _fallback = None
            # Enforce the established 4mm IK convergence gate before motion.
            # If the primary branch misses it, search existing human-valid
            # seeds and accept only a candidate that also clears the full path.
            if ik_fk_mm > 4.0:
                _ik_seed_candidates = []
                _ik_seed_hits = []
                for _seed_index, _seed in enumerate(
                            K.HUMAN_SEEDS + K.HUMAN_SEEDS_EXTENDED):
                    _seed_q = K.human_seed_joints(_seed)
                    _q_seed, _e_seed, _how_seed = K.ik_world(
                        side, T_world, base_at_ik, _seed_q, elbow=None,
                        refine=False)
                    if (not np.all(np.isfinite(_q_seed))
                            or not math.isfinite(float(_e_seed))
                            or float(_e_seed) > 4e-3
                            or K.violation_list(side, _q_seed)):
                        continue
                    _seed_fk = K.fk_world(side, _q_seed, base_at_ik)
                    _seed_fk_mm = pos_err_mm(_seed_fk, T_world)
                    if (not math.isfinite(float(_seed_fk_mm))
                            or _seed_fk_mm > 4.0):
                        continue
                    _seed_clear, _seed_hit = _path_clear(_q_seed)
                    if not _seed_clear:
                        if len(_ik_seed_hits) < 3:
                            _ik_seed_hits.append({
                                'seed': _seed_index, 'hit': _seed_hit})
                        continue
                    _score = (K.human_penalty(side, _q_seed)
                              + 0.12 * float(np.max(np.abs(_q_seed - q_now))))
                    _ik_seed_candidates.append(
                        (_score, _seed_index, _q_seed, _seed_fk_mm, _how_seed))
                if _ik_seed_candidates:
                    (_score, _seed_index, q, ik_fk_mm,
                     _seed_how) = min(_ik_seed_candidates, key=lambda x: x[0])
                    how = '%s/human-seed-%d' % (_seed_how, _seed_index)
                    T_ik_fk = K.fk_world(side, q, base_at_ik)
                    ik_fk_deg = axis_angle_deg(T_ik_fk, T_world)
                    q_target_last = q.copy()
                    _clear, _hit = True, None
                    _fallback = 'ik-residual-human-seed-%d' % _seed_index
                else:
                    node.get_logger().warn(
                        'REACH %s IK 候选未满足 4mm 收敛门槛，拒绝执行: '
                        'primary=%.1fmm collision_clear_seeds=0 hits=%s '
                        'target_xyz=%s base=%s'
                        % (side, ik_fk_mm, _ik_seed_hits,
                           np.round(np.asarray(T_world)[:3, 3], 4).tolist(),
                           [round(float(v), 5) for v in base_at_ik]))
                    return False, ik_fk_mm, 'ik-residual'
            # 目标掌心不变时，7 轴臂仍有冗余姿态可选。human 分支的关节
            # 插值若撞桌，不立即放弃：尝试肘外分支，但必须同时满足人体
            # 关节规则和同一套完整路径碰撞检查。只接受可验证的整段候选；
            # 不拆掉碰撞门控、不缩小几何半径，也不执行部分碰撞路径。
            if not _clear and _mode == 'human':
                q_alt, e_alt, how_alt = K.ik_world(
                    side, T_world, base_at_ik, q_now, elbow='out')
                alt_bad = K.violation_list(side, q_alt)
                if not alt_bad:
                    alt_clear, alt_hit = _path_clear(q_alt)
                    if alt_clear:
                        q = q_alt
                        how = how_alt
                        q_target_last = q.copy()
                        _clear, _hit = True, None
                        _fallback = 'out'
                    else:
                        _fallback = 'out-collision:%s' % alt_hit
                else:
                    _fallback = 'out-human-rule:%s' % alt_bad
                # human 与 out 两个预设分支都可能在桌边被前臂几何挡住。
                # 7 轴臂仍有冗余解：逐一尝试人体规则种子，先筛 IK 残差与
                # R1~R7，再对完整插值路径做同一套胶囊/掌部 OBB 碰撞检查。
                # 只执行整段无碰撞候选；没有候选通过时仍 fail-closed。
                if not _clear:
                    _seed_candidates = []
                    _seen_q = set()
                    for _seed_index, _seed in enumerate(
                            K.HUMAN_SEEDS + K.HUMAN_SEEDS_EXTENDED):
                        _seed_q = K.human_seed_joints(_seed)
                        _q_seed, _e_seed, _how_seed = K.ik_world(
                            side, T_world, base_at_ik, _seed_q, elbow=None,
                            refine=False)
                        if (not np.all(np.isfinite(_q_seed))
                                or not math.isfinite(float(_e_seed))
                                or float(_e_seed) > 4e-3
                                or K.violation_list(side, _q_seed)):
                            continue
                        _key = tuple(np.round(_q_seed, 3))
                        if _key in _seen_q:
                            continue
                        _seen_q.add(_key)
                        _score = (K.human_penalty(side, _q_seed)
                                  + 2.0 * float(_e_seed)
                                  + 0.12 * float(np.max(np.abs(_q_seed - q_now))))
                        _seed_candidates.append(
                            (_score, _seed_index, _q_seed, _e_seed, _how_seed))
                    _seed_candidates.sort(key=lambda item: item[0])
                    _seed_hits = []
                    _endpoint_seed_clear = 0
                    _endpoint_seed_hits = []
                    for (_score, _seed_index, _q_seed,
                         _e_seed, _how_seed) in _seed_candidates:
                        _seed_clear, _seed_hit = _path_clear(_q_seed)
                        if _seed_clear:
                            q = _q_seed
                            how = '%s/human-seed-%d' % (_how_seed, _seed_index)
                            q_target_last = q.copy()
                            _clear, _hit = True, None
                            _fallback = 'human-seed-%d' % _seed_index
                            break
                        # Diagnose endpoint-only collision separately for each
                        # valid human IK seed. Rejected candidates stay rejected.
                        _endpoint_seed_ok, _endpoint_seed_hit = (
                            arm_path_table_clear(
                                node, side, [_q_seed], base=base_at_ik,
                                ignore_initial=True, q_start=_q_seed))
                        if _endpoint_seed_ok:
                            _endpoint_seed_clear += 1
                        elif len(_endpoint_seed_hits) < 3:
                            _endpoint_seed_hits.append({
                                'seed': _seed_index,
                                'hit': _endpoint_seed_hit})
                        if len(_seed_hits) < 3:
                            _seed_hits.append({
                                'seed': _seed_index, 'hit': _seed_hit})
                    if not _clear:
                        _fallback = (
                            'human-seed-search-no-clear-path candidates=%d '
                            'endpoint_clear_seeds=%d path_hits=%s endpoint_hits=%s'
                            % (len(_seed_candidates), _endpoint_seed_clear,
                               _seed_hits, _endpoint_seed_hits))
                node.get_logger().info(
                    'REACH %s 碰撞候选重规划: fallback=%s ik_error=%.5f'
                    % (side, _fallback, float(e_alt)))
            if not _clear:
                # Separate an intrinsically colliding IK endpoint from a
                # collision that exists only along the current-to-target
                # interpolation. This is diagnostic only: it never changes
                # the fail-closed motion decision or collision geometry.
                _endpoint_clear, _endpoint_hit = arm_path_table_clear(
                    node, side, [q], base=base_at_ik,
                    ignore_initial=True, q_start=q)
                # 碰撞门控在执行前拒绝整段关节路径。这里的 res 仍是初始化
                # 哨兵 9e9，不能作为实际末端误差；读取当前掌心并计算到本步
                # 目标的真实距离，同时保留关节端点和首次碰撞几何证据。
                T_current = wrist(node, side)
                if T_current is not None:
                    res = pos_err_mm(T_current, T_world)
                    current_xyz = np.round(T_current[:3, 3], 4).tolist()
                else:
                    current_xyz = None
                _js_age_at_ik = (
                    max(0.0, ik_sample_t - js_sample_t_at_ik)
                    if js_sample_t_at_ik > 0.0 else None)
                _truth_age_at_ik = (
                    max(0.0, ik_sample_t - truth_pose_t_at_ik)
                    if truth_pose_t_at_ik > 0.0 else None)
                node.get_logger().warn(
                    'REACH %s 桌面碰撞门控拒绝路径: hit=%s fallback=%s '
                    'current_xyz=%s target_xyz=%s residual=%.1fmm '
                    'q_now=%s q_target=%s base_at_ik=%s truth_base_at_ik=%s '
                    'base_truth_xy_mm_at_ik=%s base_truth_yaw_deg_at_ik=%s '
                    'joint_age_at_ik_s=%s truth_age_at_ik_s=%s '
                    'endpoint_clear=%s endpoint_hit=%s'
                    % (side, _hit, _fallback, current_xyz,
                       np.round(np.asarray(T_world)[:3, 3], 4).tolist(),
                       res, np.round(q_now, 3).tolist(),
                       np.round(q, 3).tolist(),
                       np.round(base_at_ik, 6).tolist(),
                       (np.round(truth_pose_at_ik, 6).tolist()
                        if truth_pose_at_ik is not None else None),
                       (round(base_truth_xy_mm_at_ik, 3)
                        if base_truth_xy_mm_at_ik is not None else None),
                       (round(base_truth_yaw_deg_at_ik, 4)
                        if base_truth_yaw_deg_at_ik is not None else None),
                       (round(_js_age_at_ik, 4)
                        if _js_age_at_ik is not None else None),
                       (round(_truth_age_at_ik, 4)
                        if _truth_age_at_ik is not None else None),
                       bool(_endpoint_clear), _endpoint_hit))
                return False, res, 'table-collision'
        if preflight_only:
            preflight_res = pos_err_mm(
                K.fk_world(side, q, base_at_ik), T_world)
            if not math.isfinite(float(preflight_res)):
                return False, 9e9, 'preflight-ik-invalid'
            return True, float(preflight_res), how
        q_commanded = np.asarray(q, dtype=float).copy()
        ok_move, joint_err = node.arm[side].move(
            q_commanded.copy(), timeout=timeout,
            tol_deg=(tol_deg if tol_deg is not None
                     else TOL_SCHED[min(it, len(TOL_SCHED) - 1)]))
        # Capture the servo object's own final sample separately from the
        # node-level joint cache. This detects callback/cache divergence and
        # prevents a late cache update from being mistaken for servo success.
        try:
            q_servo_final = np.asarray(node.arm[side]._read(), dtype=float)
        except Exception:
            q_servo_final = np.full_like(q_commanded, np.nan)
        q_node_final = q_meas(node, side)
        servo_error_check = (q_commanded - q_servo_final
                             if q_servo_final.shape == q_commanded.shape
                             else np.full_like(q_commanded, np.nan))
        node_error_check = (q_commanded - q_node_final
                            if q_node_final is not None
                            and q_node_final.shape == q_commanded.shape
                            else np.full_like(q_commanded, np.nan))
        # Servo completion only means the joints entered tolerance; the arm
        # can still settle and transmit reaction torque into the mobile base.
        # Let Gazebo advance before measuring the world-frame palm so the next
        # IK pass starts from a settled, fresh base-truth snapshot.
        settle_sim(node, 0.20)
        T_after_move = wrist(node, side)
        res = pos_err_mm(T_after_move, T_world)
        joint_err_deg = (
            float(np.max(np.abs(np.degrees(joint_err))))
            if np.all(np.isfinite(joint_err)) else 9e9)
        # Treat command snapshots as evidence only while fresh. A cached zero
        # from an earlier drive/stop cycle must never masquerade as a live stop.
        _trace_now = time.monotonic()
        def _command_evidence(snapshot):
            if snapshot is None:
                return None, None, False
            _age = max(0.0, _trace_now - float(snapshot[1]))
            _fresh = _age <= 0.5
            return (list(snapshot[0]) if _fresh else None,
                    round(_age, 3), _fresh)
        _drink_vel, _drink_age, _drink_fresh = _command_evidence(
            getattr(node, '_drink_cmd_snapshot', None))
        _robot_vel, _robot_age, _robot_fresh = _command_evidence(
            getattr(node, '_robot_cmd_snapshot', None))
        pass_trace.append({
            'pass': it + 1, 'ik': how, 'res_mm': round(res, 1),
            'ik_fk_mm': round(ik_fk_mm, 2),
            'ik_fk_deg': round(ik_fk_deg, 3),
            'base_at_ik': [round(v, 5) for v in base_at_ik],
            'joint_state_age_at_ik_s': (
                round(max(0.0, ik_sample_t - js_sample_t_at_ik), 4)
                if js_sample_t_at_ik else None),
            'truth_base_pose_age_at_ik_s': (
                round(max(0.0, ik_sample_t - truth_pose_t_at_ik), 4)
                if truth_pose_t_at_ik else None),
            'truth_base_pose_at_ik': (
                [round(v, 5) for v in truth_pose_at_ik]
                if truth_pose_at_ik is not None else None),
            'truth_base_twist_at_ik': (
                [round(float(v), 5) for v in truth_twist_at_ik]
                if truth_twist_at_ik is not None else None),
            'truth_base_twist_age_at_ik_s': (
                round(max(0.0, ik_sample_t - truth_twist_t_at_ik), 4)
                if truth_twist_t_at_ik > 0.0 else None),
            'drink_cmd_at_ik': (
                [round(float(v), 5) for v in drink_cmd_at_ik[0]]
                if drink_cmd_at_ik is not None
                and ik_sample_t - float(drink_cmd_at_ik[1]) <= 0.5
                else None),
            'drink_cmd_age_at_ik_s': (
                round(max(0.0, ik_sample_t - float(drink_cmd_at_ik[1])), 4)
                if drink_cmd_at_ik is not None else None),
            'robot_cmd_at_ik': (
                [round(float(v), 5) for v in robot_cmd_at_ik[0]]
                if robot_cmd_at_ik is not None
                and ik_sample_t - float(robot_cmd_at_ik[1]) <= 0.5
                else None),
            'robot_cmd_age_at_ik_s': (
                round(max(0.0, ik_sample_t - float(robot_cmd_at_ik[1])), 4)
                if robot_cmd_at_ik is not None else None),
            'base_truth_xy_mm_at_ik': (
                round(base_truth_xy_mm_at_ik, 2)
                if base_truth_xy_mm_at_ik is not None else None),
            'base_truth_yaw_deg_at_ik': (
                round(base_truth_yaw_deg_at_ik, 4)
                if base_truth_yaw_deg_at_ik is not None else None),
            'joint_state_age_s': (
                round(max(0.0, time.monotonic() -
                          float(getattr(node, 'js_sample_t', 0.0))), 3)
                if getattr(node, 'js_sample_t', 0.0) else None),
            'truth_base_pose_age_s': (
                round(max(0.0, time.monotonic() -
                          float(getattr(node, 'truth_base_pose_snapshot',
                                        (None, 0.0))[1])), 3)
                if getattr(node, 'truth_base_pose_snapshot', None)
                else None),
            'odom_twist': ([round(float(v), 5) for v in node.odom_twist]
                           if getattr(node, 'odom_twist', None) is not None
                           else None),
            'truth_base_twist': (
                [round(float(v), 5) for v in node.truth_base_twist]
                if getattr(node, 'truth_base_twist', None) is not None
                else None),
            'drink_cmd_vel': _drink_vel,
            'drink_cmd_age_s': _drink_age,
            'drink_cmd_fresh': _drink_fresh,
            'robot_cmd_vel': _robot_vel,
            'robot_cmd_age_s': _robot_age,
            'robot_cmd_fresh': _robot_fresh,
            'wheel_joint_velocity': {
                name: round(float(node.js_velocity[name]), 5)
                for name in ('left_wheel_joint', 'right_wheel_joint')
                if name in getattr(node, 'js_velocity', {})},
            'servo_ok': bool(ok_move),
'servo_names': list(getattr(node.arm[side], 'names', [])),
'servo_target_rad': [round(float(v), 6) for v in q_commanded],
'servo_final_rad': ([round(float(v), 6) for v in q_servo_final]
                    if np.all(np.isfinite(q_servo_final)) else None),
'node_final_rad': ([round(float(v), 6) for v in q_node_final]
                   if q_node_final is not None
                   and np.all(np.isfinite(q_node_final)) else None),
'servo_error_rad': ([round(float(v), 6) for v in servo_error_check]
                    if np.all(np.isfinite(servo_error_check)) else None),
'node_error_rad': ([round(float(v), 6) for v in node_error_check]
                   if np.all(np.isfinite(node_error_check)) else None),
'reported_error_rad': ([round(float(v), 6) for v in joint_err]
                       if np.all(np.isfinite(joint_err)) else None),
            'joint_err_deg': round(joint_err_deg, 2)})
        if not ok_move:
            # A failed servo pass returns before the ordinary convergence
            # summary below. Preserve the exact target/measurement and the
            # already-collected per-pass chassis/command evidence here.
            q_failed = q_meas(node, side)
            q_servo_failed = np.asarray(node.arm[side]._read(), dtype=float)
            node.get_logger().warn(
                'REACH %s 伺服未确认到位，停止补位: 残差 %.1fmm '
                '最大关节误差 %.2f° target_xyz=%s actual_xyz=%s '
                'base_at_ik=%s q_target=%s q_measured=%s '
                'servo_measured=%s servo_error_deg=%s node_error_deg=%s '
                'reported_error_deg=%s joint_delta_deg=%s trace=%s'
                % (side, res, joint_err_deg,
                   np.round(np.asarray(T_world)[:3, 3], 4).tolist(),
                   (np.round(T_after_move[:3, 3], 4).tolist()
                    if T_after_move is not None else None),
                   [round(float(v), 5) for v in base_at_ik],
                   np.round(q_commanded, 6).tolist(),
                   np.round(q_failed, 6).tolist()
                   if q_failed is not None else None,
                   (np.round(q_servo_failed, 6).tolist()
                    if np.all(np.isfinite(q_servo_failed)) else None),
                   (np.round(np.degrees(q_commanded - q_servo_failed), 3).tolist()
                    if q_servo_failed.shape == q_commanded.shape
                    and np.all(np.isfinite(q_servo_failed)) else None),
                   (np.round(np.degrees(q_commanded - q_failed), 3).tolist()
                    if q_failed is not None
                    and np.all(np.isfinite(q_failed)) else None),
                   (np.round(np.degrees(joint_err), 3).tolist()
                    if np.all(np.isfinite(joint_err)) else None),
                   np.round(np.degrees(joint_err), 3).tolist()
                   if np.all(np.isfinite(joint_err)) else None,
                   pass_trace))
            return False, res, how
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
    if res >= tol_mm:
        T_final = wrist(node, side)
        q_final = q_meas(node, side)
        truth_diag = (fk_truth_diagnostics(node, side, q_final)
                      if q_final is not None else {'reason': 'joint_state_missing'})
        node.get_logger().warn(
            'REACH %s 收敛不足: passes=%d/%d tol=%.1fmm '
            'first_ik=%s last_ik=%s trace=%s '
            'target_xyz=%s actual_xyz=%s pose_err=%.2fdeg '
            'q_final=%s q_target=%s fk_truth=%s'
            % (side, len(pass_trace), passes, tol_mm,
               first_how, how, pass_trace,
               np.round(np.asarray(T_world)[:3, 3], 4).tolist(),
               (np.round(T_final[:3, 3], 4).tolist()
                if T_final is not None else None),
               (axis_angle_deg(T_final, T_world)
                if T_final is not None else 9e9),
               (np.round(q_final, 3).tolist()
                if q_final is not None else None),
               (np.round(q_target_last, 3).tolist()
                if q_target_last is not None else None),
               truth_diag))
    return res < tol_mm, res, how


def reach(node, side, T_world, tol_mm=3.0, passes=len(TOL_SCHED),
          timeout=10.0, elbow='human', tol_deg=None, check_table=False,
          preflight_only=False, require_base_still=False):
    """Public reach wrapper for the isolated PREPARE hold-neutralization experiment."""
    suspend_prepare_hold = bool(
        require_base_still
        and os.environ.get('VLA_PREPARE_BASE_HOLD_ZERO_DURING_REACH', '0') == '1'
        and getattr(node, '_prepare_base_hold_target', None) is not None)
    if suspend_prepare_hold:
        node._prepare_base_reach_active = True
        node.get_logger().info(
            'PREPARE_BASE_HOLD_MODE zero-during-reach: '
            'send zero chassis command while arm reach is active')
    try:
        return _reach_impl(
            node, side, T_world, tol_mm=tol_mm, passes=passes,
            timeout=timeout, elbow=elbow, tol_deg=tol_deg,
            check_table=check_table, preflight_only=preflight_only,
            require_base_still=require_base_still)
    finally:
        if suspend_prepare_hold:
            node._prepare_base_reach_active = False


 how


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


def _present_hold_linear_limit():
    """Runtime-tunable PRESENT-only linear velocity cap; bounded to 0.04–0.12m/s."""
    try:
        requested = float(os.environ.get('VLA_PRESENT_BASE_HOLD_LINEAR_LIMIT', '0.08'))
    except (TypeError, ValueError, OverflowError):
        requested = 0.08
    if not math.isfinite(requested):
        requested = 0.08
    return max(0.04, min(0.12, requested))


def _present_hold_linear_kp():
    """Runtime-tunable PRESENT longitudinal position gain, bounded to 0.5–3.0."""
    try:
        requested = float(os.environ.get('VLA_PRESENT_BASE_HOLD_LINEAR_KP', '1.5'))
    except (TypeError, ValueError, OverflowError):
        requested = 1.5
    if not math.isfinite(requested):
        requested = 1.5
    return max(0.5, min(3.0, requested))


def _present_hold_linear_kd():
    """Runtime-tunable PRESENT longitudinal velocity damping, bounded to 0–1.5."""
    try:
        requested = float(os.environ.get('VLA_PRESENT_BASE_HOLD_LINEAR_KD', '0.8'))
    except (TypeError, ValueError, OverflowError):
        requested = 0.8
    if not math.isfinite(requested):
        requested = 0.8
    return max(0.0, min(1.5, requested))


def _arm_hold_yaw_limit():
    """Runtime-tunable yaw cap for bounded arm-reaction pose hold.

    The setting is intentionally bounded: values below 0.02 rad/s or above
    0.08 rad/s are clamped. Invalid values fall back to the conservative 0.04 rad/s default.
    """
    try:
        requested = float(os.environ.get('VLA_ARM_HOLD_YAW_LIMIT', '0.04'))
    except (TypeError, ValueError, OverflowError):
        requested = 0.04
    if not math.isfinite(requested):
        requested = 0.04
    return max(0.02, min(0.08, requested))


def _cup_lift_reach_tol_mm():
    """Tunable per-reach palm tolerance for cup lift; never looser than 1.5mm."""
    try:
        requested = float(os.environ.get('VLA_CUP_LIFT_REACH_TOL_MM', '1.5'))
    except (TypeError, ValueError, OverflowError):
        requested = 1.5
    if not math.isfinite(requested):
        requested = 1.5
    return max(0.5, min(1.5, requested))


def _kettle_pour_reach_tol_deg():
    """Tunable per-joint servo tolerance for kettle pour; never looser than 0.08deg."""
    try:
        requested = float(os.environ.get('VLA_KETTLE_POUR_REACH_TOL_DEG', '0.08'))
    except (TypeError, ValueError, OverflowError):
        requested = 0.08
    if not math.isfinite(requested):
        requested = 0.08
    return max(0.04, min(0.08, requested))


def base_pose_hold_velocity(target, current, max_linear=0.04,
                           max_angular=0.12, current_twist=(0.0, 0.0, 0.0),
                           translate_during_yaw=False):
    """Bounded unicycle pose stabilizer for short arm-reaction disturbances.

    target/current are (x, y, yaw) in world coordinates. Reverse motion is
    allowed so the controller can return to the captured pose without making
    a full turn. This is active only during bounded arm-reaction windows (PREPARE pour and PRESENT follow_hand).
    """
    values = tuple(float(v) for v in (*target, *current, *current_twist,
                                      max_linear, max_angular))
    if not all(math.isfinite(v) for v in values):
        return 0.0, 0.0
    tx, ty, tyaw, x, y, yaw, _tvx, _tvy, yaw_rate, vlim, wlim = values
    if vlim <= 0.0 or wlim <= 0.0:
        return 0.0, 0.0
    dx, dy = tx - x, ty - y
    rho = math.hypot(dx, dy)
    theta = math.atan2(math.sin(tyaw - yaw), math.cos(tyaw - yaw))
    # Arm reaction normally displaces the free-rolling base only a few mm.
    # In this near-pose regime, steering toward the XY bearing can command a
    # reverse-turn that fights the captured yaw (and was observed to increase
    # yaw error during PREPARE). Keep heading correction independent; during
    # PRESENT follow_hand only, permit bounded longitudinal correction at the
    # same time so yaw recovery does not leave translation drift unopposed.
    # The PREPARE trace crossed 30mm during arm reaction; dropping back to
    # the reverse-bearing controller there reintroduced yaw-opposing turns.
    # Keep the staged yaw/longitudinal controller through a 60mm recovery band.
    if rho <= 0.06:
        if abs(theta) > math.radians(0.5):
            # Gazebo chassis yaw has measurable inertia: pose-only P control
            # reversed after crossing the target but still overshot. Use the
            # independently sampled truth yaw rate as derivative damping.
            hold_wlim = min(wlim, _arm_hold_yaw_limit())
            wz = max(-hold_wlim, min(
                hold_wlim, 1.2 * theta - 1.5 * yaw_rate))
            if translate_during_yaw:
                longitudinal = dx * math.cos(yaw) + dy * math.sin(yaw)
                longitudinal_speed = _tvx * math.cos(yaw) + _tvy * math.sin(yaw)
                vx = max(-vlim, min(
                    vlim, _present_hold_linear_kp() * longitudinal
                    - _present_hold_linear_kd() * longitudinal_speed))
                return vx, wz
            return 0.0, wz
        longitudinal = dx * math.cos(yaw) + dy * math.sin(yaw)
        if translate_during_yaw:
            longitudinal_speed = _tvx * math.cos(yaw) + _tvy * math.sin(yaw)
            vx = max(-vlim, min(
                vlim, _present_hold_linear_kp() * longitudinal
                - _present_hold_linear_kd() * longitudinal_speed))
        else:
            vx = max(-vlim, min(vlim, 0.8 * longitudinal))
        return vx, 0.0
    if rho <= 0.0015:
        wz = max(-wlim, min(wlim, 1.2 * theta))
        return 0.0, (0.0 if abs(theta) <= math.radians(0.15) else wz)
    bearing = math.atan2(dy, dx)
    alpha = math.atan2(math.sin(bearing - yaw), math.cos(bearing - yaw))
    direction = 1.0
    if abs(alpha) > math.pi / 2.0:
        direction = -1.0
        alpha = math.atan2(math.sin(alpha - math.pi),
                           math.cos(alpha - math.pi))
    vx = direction * min(vlim, 0.8 * rho) * max(0.0, math.cos(alpha))
    wz = max(-wlim, min(wlim, 1.2 * alpha + 0.6 * theta))
    if abs(alpha) <= math.radians(0.5):
        wz = max(-wlim, min(wlim, 1.2 * theta))
    return vx, wz


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


def base_pose_converged(position_err_mm, yaw_err_rad, tol_mm=8.0,
                       yaw_tol_rad=math.radians(2.0)):
    """底盘摆位必须同时满足有限的位置误差与航向误差，NaN/Inf 一律失败。"""
    try:
        pos = float(position_err_mm)
        yaw = float(yaw_err_rad)
        pos_tol = float(tol_mm)
        yaw_tol = float(yaw_tol_rad)
    except (TypeError, ValueError):
        return False
    return (math.isfinite(pos) and math.isfinite(yaw)
            and math.isfinite(pos_tol) and math.isfinite(yaw_tol)
            and pos >= 0.0 and yaw >= 0.0
            and pos < pos_tol and yaw < yaw_tol)


def base_motion_snapshot_ok(wheel_velocity, joint_age_s, truth_twist,
                             twist_age_s, wheel_tol=0.05,
                             linear_tol=0.01, angular_tol=0.015,
                             max_age_s=0.25):
    """Require fresh wheel and Gazebo truth velocity evidence before arm IK."""
    try:
        wheels = {
            name: float(wheel_velocity[name])
            for name in ('left_wheel_joint', 'right_wheel_joint')}
        twist = tuple(float(v) for v in truth_twist)
        ages = (float(joint_age_s), float(twist_age_s))
        limits = (float(wheel_tol), float(linear_tol),
                  float(angular_tol), float(max_age_s))
    except (KeyError, TypeError, ValueError, OverflowError):
        return False
    values = list(wheels.values()) + list(twist) + list(ages) + list(limits)
    if (len(twist) != 3 or not all(math.isfinite(v) for v in values)
            or any(v < 0.0 for v in ages + limits)):
        return False
    return (max(abs(v) for v in wheels.values()) <= limits[0]
            and math.hypot(twist[0], twist[1]) <= limits[1]
            and abs(twist[2]) <= limits[2]
            and max(ages) <= limits[3])


def wait_base_stopped(node, timeout=4.0, stable_s=0.25):
    """Command zero and require fresh wheel+Gazebo velocities to stay quiet."""
    node.drive(0.0, 0.0)
    start_sim = sim_now(node)
    start_wall = time.monotonic()
    stable_since = None
    last = None
    while (sim_now(node) - start_sim < float(timeout)
           and time.monotonic() - start_wall < float(timeout) * 6.0 + 1.0):
        spin = getattr(node, 'spin_sim', None)
        if spin is None:
            node.spin_for(0.05)
        else:
            spin(0.05)
        now = time.monotonic()
        js_t = float(getattr(node, 'js_sample_t', 0.0) or 0.0)
        twist_t = float(getattr(node, 'truth_base_twist_t', 0.0) or 0.0)
        twist = getattr(node, 'truth_base_twist', None)
        wheels = getattr(node, 'js_velocity', {})
        last = {
            'wheel_rad_s': {
                name: wheels.get(name)
                for name in ('left_wheel_joint', 'right_wheel_joint')},
            'truth_twist_m_s_rad_s': twist,
            'joint_age_s': (max(0.0, now - js_t) if js_t else 9e9),
            'truth_twist_age_s': (
                max(0.0, now - twist_t) if twist_t else 9e9),
        }
        quiet = base_motion_snapshot_ok(
            wheels, last['joint_age_s'], twist,
            last['truth_twist_age_s'])
        if quiet:
            if stable_since is None:
                stable_since = sim_now(node)
            if sim_now(node) - stable_since >= float(stable_s):
                node.get_logger().info(
                    'BASE_STOP 已稳定: wheel=%s truth_twist=%s'
                    % (last['wheel_rad_s'], last['truth_twist_m_s_rad_s']))
                return True
        else:
            stable_since = None
    node.get_logger().error(
        'BASE_STOP 未在 %.1fs 内稳定，拒绝继续桌边 IK/动作: %s'
        % (float(timeout), last))
    return False


def place_robot(node, T_goal, tol_mm=8.0, tries=3, settle=0.25):
    """把底盘 teleport 到 T_goal, 用真值核对, 不够就重放。

    `CarryBus.place` 是"下一帧 SetWorldPoseCmd", 但 gz 的 teleport **不清零
    速度**。因此摆位前先发送零速度并等待轮速与 Gazebo 真值 twist 连续稳定；
    teleport 后再次确认底盘静止，再检查位置/yaw。缺少新鲜速度证据或超时均
    fail-closed，避免残余轮速在机械臂 IK 期间继续改变底座。

    返回最终位置残差 mm。
    """
    # 先清除 follower/上一轮 drive 残留的轮速。不能只看 cmd_vel=0：
    # E2E #68 第二轮实测 cmd_vel=0 时左轮仍为 -0.986rad/s，底盘 yaw
    # 在 5 轮 IK 中漂移约 15.6°，将 5.4mm 的末轮残差推过 4mm 门槛。
    if not wait_base_stopped(node):
        return 9e9
    # P98 原有位置+yaw 收敛判据继续保留。
    res = 9e9
    yaw_err = 9e9
    ack_ok = False
    for _ in range(max(1, tries) * 2):
        ack_ok = bool(node.carry.place('robot', T_goal))
        if not ack_ok:
            node.get_logger().error(
                'PLACE_ROBOT 无 ACK，停止重试并拒绝继续桌边动作')
            break
        settle_sim(node, settle)
        # Teleport 本身不清除 wheel velocity；确认实际底盘已停稳后才采信 pose。
        if not wait_base_stopped(node):
            ack_ok = False
            break
        T_now = object_pose(node, 'robot')
        if T_now is None:
            node.get_logger().error('PLACE_ROBOT 收到 ACK 但缺少 robot 真值位姿')
            ack_ok = False
            break
        res = pos_err_mm(T_now, T_goal)
        yaw_err = abs(yaw_err_of(T_now, T_goal))
        if base_pose_converged(res, yaw_err, tol_mm):
            break
    node.sync_base_from_truth(quiet=True)
    if not ack_ok or not base_pose_converged(res, yaw_err, tol_mm):
        node.get_logger().error(
            'PLACE_ROBOT 未收敛: ack=%s position=%.1fmm yaw=%.2f° '
            '(limits %.1fmm/2.00°)'
            % (ack_ok, res, math.degrees(yaw_err), tol_mm))
        # 保留历史返回“残差mm”的接口；失败时保证严格大于调用方 tol_mm。
        if not math.isfinite(float(res)):
            return 9e9
        return max(float(res), float(tol_mm) + 1.0)
    return float(res)


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
    ok_open, _ = open_hand(node, side)
    if not ok_open:
        node.get_logger().error(
            'grasp %s (%s): 张手伺服失败，禁止靠近和附着' % (model, side))
        return False, 9e9
    Tk = object_pose(node, model)
    if Tk is None:
        return False, 9e9
    T_grasp = palm_fn(Tk)
    T_pre = T_grasp @ K.trans((0.0, 0.0, approach))
    # Both approach and final grasp are performed near tabletop objects. The
    # old path omitted the table gate here, so an IK-valid arm posture could
    # drive into the table edge and time out far from the target. Check the
    # complete interpolated arm path before issuing any joint command.
    ok_pre, res_pre, how_pre = reach(
        node, side, T_pre, tol_mm=8.0, check_table=True)
    node.get_logger().info(
        'GRASP_APPROACH model=%s side=%s ok=%s residual_mm=%.1f ik=%s '
        'target_xyz=%s' %
        (model, side, ok_pre, float(res_pre), how_pre,
         np.round(T_pre[:3, 3], 4).tolist()))
    if not ok_pre:
        node.get_logger().error(
            'grasp %s (%s): 接近位未到达 %.1fmm，禁止继续伸入' %
            (model, side, res_pre))
        return False, res_pre
    # A grasp may induce small chassis motion through arm reaction torque.
    # Keep the 5mm gate unchanged, but allow extra truth-refreshed correction
    # passes after each post-servo settling interval. The final grasp approach
    # is independently collision-checked from the measured pre-grasp posture.
    ok, res, how = reach(
        node, side, T_grasp, tol_mm=GRASP_TOL_MM, passes=8,
        check_table=True)
    # P98 补: reach 严重未到位还照旧 attach, 会把物体钉到 0.3m 外(实测把杯
    # 钉飞、场景全污)。阈值取 3x 容差: 边缘差几毫米照常抓(IK 已收敛, 手指
    # 闭合余量足够), 300mm 级的灾难失败才拒绝, 让上游走 FAIL 路径。
    if not ok or res > GRASP_TOL_MM * 3.0:
        if verbose:
            node.get_logger().warn(
                'grasp %s (%s): reach 未到位 %.1fmm (IK=%s), 跳过 attach'
                % (model, side, res, how))
        return False, res
    ok_close, _ = close_hand(node, side, grip)
    if not ok_close:
        node.get_logger().error(
            'grasp %s (%s): 闭手未确认到位，禁止 attach' % (model, side))
        return False, res
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
               elbow='human', budget=60.0, reach_tol_deg=None,
               reach_passes=None, check_table=False,
               reach_require_base_still=False):
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
        palm_target = dT @ Tp
        ok_reach, reach_res, how = reach(
            node, side, palm_target,
            tol_mm=(max(tol_mm, 2.0) if reach_tol_mm is None
                    else reach_tol_mm), elbow=elbow,
            tol_deg=reach_tol_deg,
            passes=(len(TOL_SCHED) if reach_passes is None
                    else int(reach_passes)), check_table=check_table,
            require_base_still=reach_require_base_still)
        if not ok_reach:
            # reach() 失败时，前一轮的物体残差可能仍是大位移旧值；伺服
            # 已经移动到目标附近后，不能把这个过期值作为最终失败残差上报。
            # 重新读取物体/掌心真值，并记录目标、IK 分支和关节状态，便于
            # 区分位置不动点、姿态不收敛、IK 分支与伺服误差。
            Tk_after = object_pose(node, model)
            Tp_after = wrist(node, side)
            if Tk_after is not None:
                res_after = pos_err_mm(Tk_after, T_obj_goal)
                ang_after = axis_angle_deg(Tk_after, T_obj_goal)
                if ang_after > 180.0:
                    ang_after = 360.0 - ang_after
                object_xyz = np.round(Tk_after[:3, 3], 4).tolist()
            else:
                res_after, ang_after, object_xyz = res, 9e9, None
            if Tp_after is not None:
                palm_res_after = pos_err_mm(Tp_after, palm_target)
                palm_xyz = np.round(Tp_after[:3, 3], 4).tolist()
            else:
                palm_res_after, palm_xyz = 9e9, None
            q_after = q_meas(node, side)
            q_text = (np.round(q_after, 3).tolist()
                      if q_after is not None else None)
            node.get_logger().warn(
                'PLACE %s: 补位未确认 reach=%s(掌心 %.1fmm); '
                '掌心目标=%s 实测=%s 残差=%.1fmm; '
                '物体目标=%s 实测=%s 残差=%.1fmm/%.2f°; q=%s; '
                '此前物体残差 %.1fmm；保持当前抓持'
                % (model, how, reach_res,
                   np.round(palm_target[:3, 3], 4).tolist(), palm_xyz,
                   palm_res_after,
                   np.round(np.asarray(T_obj_goal)[:3, 3], 4).tolist(),
                   object_xyz, res_after, ang_after, q_text, res))
            # reach 的掌心容差是内部迭代停止条件，不替代物体最终验收。
            # 若伺服动作后物体位置/姿态已达标，只允许把本轮作为第 1 次
            # 稳定观测；必须再 settle 并独立复测第 2 次，不能单次放行。
            if (res_after < tol_mm and ang_after < tol_deg
                    and _ + 1 < iters):
                good = 1
                node.get_logger().info(
                    'PLACE %s: reach辅助容差未达，但物体位姿已达最终门槛 '
                    '(%.1fmm/%.2f°)；进入第二次停稳确认，不再补位'
                    % (model, res_after, ang_after))
                continue
            return False, res_after
    return (res < tol_mm and ang < tol_deg), res


def put_down(node, side, model, T_obj_goal, retreat=0.10,
             tol_mm=PLACE_TOL_MM, info=None):
    """松手放回；未落稳或张手后复核失败时保持附着，绝不提前 release。"""
    # Placement reaches often approach monotonically while the mobile base
    # settles under arm reaction. Give the same strict 3mm reach gate up to
    # eight IK/servo refinements rather than failing after the default five.
    # This adds convergence opportunities; it does not relax pose or safety gates.
    ok1, res = place_held(
        node, side, model, T_obj_goal, tol_mm, reach_passes=8)
    if not ok1:
        node.get_logger().error(
            'PUT_DOWN %s 落位闭环失败 %.1fmm，保持抓持，不松手' % (model, res))
        return False, res
    ok_open, _ = open_hand(node, side)
    if not ok_open:
        node.get_logger().error('PUT_DOWN %s 张手伺服失败，保持抓持' % model)
        return False, res
    node.spin_for(0.2)
    # 张手后再对一次位：此时手指已离开物体，收敛到的是"无接触"状态
    ok2, res2 = place_held(
        node, side, model, T_obj_goal, tol_mm, reach_passes=8)
    if not ok2:
        node.get_logger().error(
            'PUT_DOWN %s 张手后复核失败 %.1fmm，保持附着' % (model, res2))
        return False, res2
    T_back = wrist(node, side) @ K.trans((0.0, 0.0, retreat))
    release_ok = node.carry.release(model)
    if not release_ok:
        node.get_logger().error(
            'PUT_DOWN %s release ACK 未确认；不移动手臂，保持当前位置供安全恢复'
            % model)
        if info is not None:
            info.update(release_confirmed=False, object_state='still_attached',
                        release_blocked=True,
                        release_block_reason='release_ack_missing')
        return False, res2
    # Release ACK is the ownership boundary. From this point onward the
    # object is no longer held even if the arm cannot withdraw successfully.
    if info is not None:
        info.update(release_confirmed=True, object_state='released')
    node.spin_for(0.6)
    if info is not None:
        T_after_release = object_pose(node, model)
        if T_after_release is not None:
            info['drop_mm'] = round(
                (float(np.asarray(T_obj_goal)[2, 3])
                 - float(T_after_release[2, 3])) * 1e3, 1)
    ok_back, res_back, how_back = reach(
        node, side, T_back, tol_mm=8.0, check_table=True)
    withdraw_stage = None
    # The cup's two-hand put-down already uses this exact, collision-checked
    # 120mm world-Z waypoint when the free support arm cannot reach around the
    # table lip. Reuse that proven staged retreat for a released kettle: first
    # lift the empty hand above the lip, then retry the original retreat target.
    # Every segment still passes the full table/leg path gate; a blocked lift
    # or blocked second segment remains fail-closed.
    if not ok_back and how_back == 'table-collision':
        T_stage_now = wrist(node, side)
        if T_stage_now is not None:
            T_stage = T_stage_now.copy()
            T_stage[2, 3] += 0.12
            ok_stage, res_stage, how_stage = reach(
                node, side, T_stage, tol_mm=8.0, check_table=True)
            withdraw_stage = {
                'attempted': True, 'ok': bool(ok_stage),
                'residual_mm': round(float(res_stage), 1),
                'how': how_stage, 'target_z': round(float(T_stage[2, 3]), 4)}
            if ok_stage:
                ok_back, res_back, how_back = reach(
                    node, side, T_back, tol_mm=8.0, check_table=True)
                withdraw_stage.update(
                    retry_ok=bool(ok_back),
                    retry_residual_mm=round(float(res_back), 1),
                    retry_how=how_back)
        else:
            withdraw_stage = {'attempted': False,
                              'reason': 'current_wrist_pose_missing'}
    if info is not None:
        info['withdraw_ok'] = bool(ok_back)
        info['withdraw_res_mm'] = round(float(res_back), 1)
        if withdraw_stage is not None:
            info['withdraw_stage'] = withdraw_stage
    if ok_back and withdraw_stage and withdraw_stage.get('ok'):
        node.get_logger().info(
            'PUT_DOWN %s 经桌边上方安全航点撤手到位 '
            '(上抬残差 %.1fmm, 最终残差 %.1fmm)'
            % (model, withdraw_stage['residual_mm'], res_back))
    if not ok_back:
        node.get_logger().warn(
            'PUT_DOWN %s 已确认释放并落桌，但撤手未到位 %.1fmm；'
            '物体状态=released，禁止按仍抓持处理'
            % (model, res_back))
        if info is not None:
            info.update(withdraw_failed=True,
                        failure_stage='arm_withdraw_after_release')
    elif info is not None:
        info.update(withdraw_failed=False,
                    failure_stage=None)
    return bool(ok_back), res2


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



def putback_route_candidates(T_goal):
    """Deterministic, bounded waypoint parameter sweep for PUT_BACK preflight.

    Offsets are world-frame XY offsets from the final support palm pose. The
    final support pose itself is never changed. Only the collision-checked
    approach waypoint varies between trials.
    """
    offsets = (
        (0.0, 0.0), (0.0, -0.04), (0.0, 0.04),
        (-0.04, 0.0), (0.04, 0.0),
        (-0.04, -0.04), (-0.04, 0.04),
        (0.04, -0.04), (0.04, 0.04),
    )
    candidates = []
    # Prioritize vertical clearance for the observed forearm/table sweep collision.
    for height in (0.20, 0.16, 0.12):
        for dx, dy in offsets:
            waypoint = np.asarray(T_goal, float).copy()
            waypoint[:3, 3] += np.array([dx, dy, height], dtype=float)
            candidates.append({
                'height_m': height, 'dx_m': dx, 'dy_m': dy,
                'waypoint': waypoint,
            })
    return candidates


def preflight_table_route(node, side, targets, elbow='human',
                          max_ik_error_mm=4.0):
    """No-motion sequential IK/FK and full swept-path preflight.

    Each segment is solved from the preceding segment's predicted joint
    endpoint, under one frozen base snapshot. No servo command is sent.
    Missing state, invalid IK, human joint violations, or any collision rejects
    the entire candidate route.
    """
    q_start = q_meas(node, side)
    if q_start is None:
        return False, {'reason': 'joint_state_missing'}
    base = tuple(_base_pose(node))
    q_seed = np.asarray(q_start, float).copy()
    segment_results = []
    for index, target in enumerate(targets):
        mode = elbow if index == 0 else None
        q_goal, _ik_error, how = K.ik_world(
            side, np.asarray(target, float), base, q_seed, elbow=mode)
        fk = K.fk_world(side, q_goal, base)
        residual = pos_err_mm(fk, target)
        violations = K.violation_list(side, q_goal)
        if not math.isfinite(residual) or residual > float(max_ik_error_mm):
            return False, {
                'reason': 'ik_fk_error', 'segment': index,
                'residual_mm': round(float(residual), 3),
                'ik': how, 'segments': segment_results}
        if violations:
            return False, {
                'reason': 'human_joint_limit', 'segment': index,
                'violations': list(violations), 'ik': how,
                'segments': segment_results}
        count = max(1, int(math.ceil(
            float(np.max(np.abs(q_goal - q_seed))) / 0.02)))
        path = [q_seed + (q_goal - q_seed) * (step / float(count))
                for step in range(1, count + 1)]
        clear, hit = arm_path_table_clear(
            node, side, path, base=base, ignore_initial=True,
            q_start=q_seed)
        # A numerically exact primary IK may still choose a forearm posture
        # whose swept path intersects the table. Search the redundant human-
        # valid seed set before rejecting the route; every alternative must
        # independently pass IK/FK, joint-limit and identical swept collision
        # checks. This is preflight only: no servo command is issued here.
        if not clear:
            _route_seed_candidates = []
            _route_seen = set()
            for seed_index, seed in enumerate(
                    K.HUMAN_SEEDS + K.HUMAN_SEEDS_EXTENDED):
                seed_q = K.human_seed_joints(seed)
                alt_q, alt_error, alt_how = K.ik_world(
                    side, np.asarray(target, float), base, seed_q,
                    elbow=None, refine=False)
                if (not np.all(np.isfinite(alt_q))
                        or not math.isfinite(float(alt_error))
                        or float(alt_error) > 4e-3
                        or K.violation_list(side, alt_q)):
                    continue
                alt_fk = K.fk_world(side, alt_q, base)
                alt_residual = pos_err_mm(alt_fk, target)
                if (not math.isfinite(float(alt_residual))
                        or alt_residual > float(max_ik_error_mm)):
                    continue
                key = tuple(np.round(alt_q, 3))
                if key in _route_seen:
                    continue
                _route_seen.add(key)
                alt_count = max(1, int(math.ceil(
                    float(np.max(np.abs(alt_q - q_seed))) / 0.02)))
                alt_path = [
                    q_seed + (alt_q - q_seed) * (sample / float(alt_count))
                    for sample in range(1, alt_count + 1)]
                alt_clear, _alt_hit = arm_path_table_clear(
                    node, side, alt_path, base=base,
                    ignore_initial=True, q_start=q_seed)
                if not alt_clear:
                    continue
                score = (K.human_penalty(side, alt_q)
                         + 0.12 * float(np.max(np.abs(alt_q - q_seed)))
                         + 2.0 * float(alt_error))
                _route_seed_candidates.append(
                    (score, seed_index, alt_q, alt_residual, alt_how,
                     alt_count))
            if _route_seed_candidates:
                (_, seed_index, q_goal, residual, alt_how,
                 count) = min(_route_seed_candidates, key=lambda item: item[0])
                how = '%s/table-clear-seed-%d' % (alt_how, seed_index)
                path = [
                    q_seed + (q_goal - q_seed) * (sample / float(count))
                    for sample in range(1, count + 1)]
                clear, hit = arm_path_table_clear(
                    node, side, path, base=base, ignore_initial=True,
                    q_start=q_seed)
        segment_results.append({
            'segment': index, 'ik': how,
            'ik_fk_mm': round(float(residual), 3),
            'samples': count, 'collision_clear': bool(clear),
            'collision': hit,
        })
        if not clear:
            return False, {
                'reason': 'table_collision', 'segment': index,
                'collision': hit, 'segments': segment_results}
        q_seed = np.asarray(q_goal, float)
    return True, {
        'reason': 'clear', 'base': [round(float(v), 6) for v in base],
        'segments': segment_results,
    }


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
                        tol_mm=tol_mm, info=info)
    off_l = np.linalg.inv(T_obj) @ Tl
    off_r = np.linalg.inv(T_obj) @ Tr
    # P104: preserve the exact rigid support geometry before descending. If a
    # later right-arm path is blocked by the table, this distinguishes an
    # intrinsically obstructed support offset from an IK branch/path issue.
    support_geometry = {
        'cup_start_xyz': np.round(T_obj[:3, 3], 5).tolist(),
        'cup_goal_xyz': np.round(np.asarray(T_obj_goal, float)[:3, 3], 5).tolist(),
        'base': [round(float(v), 5) for v in node.base],
        'left_cup_to_palm': np.round(off_l, 5).tolist(),
        'right_cup_to_palm': np.round(off_r, 5).tolist(),
        'left_goal_palm_xyz': np.round(
            (np.asarray(T_obj_goal, float) @ off_l)[:3, 3], 5).tolist(),
        'right_goal_palm_xyz': np.round(
            (np.asarray(T_obj_goal, float) @ off_r)[:3, 3], 5).tolist(),
    }
    if info is not None:
        info['support_geometry'] = support_geometry
    node.get_logger().info(
        'PUT_DOWN_TWO_SUPPORT_GEOM %s' % support_geometry)
    for i in range(1, steps + 1):
        Ti = _blend_pose(T_obj, T_obj_goal, i / float(steps))
        ok_l, res_l, _ = reach(
            node, 'L', Ti @ off_l, tol_mm=4.0, elbow=elbow,
            check_table=True)
        if not ok_l:
            node.get_logger().error(
                'PUT_DOWN_TWO 第%d/%d步左手未到位 %.1fmm，停止下降并保持抓持'
                % (i, steps, res_l))
            if info is not None:
                info.update(release_blocked=True,
                            release_block_reason='left_arm_path_failed',
                            failed_step=i, left_res_mm=round(res_l, 1))
            return False, res_l
        T_right_goal = Ti @ off_r
        ok_r, res_r, how_r = reach(
            node, 'R', T_right_goal, tol_mm=6.0, elbow=elbow,
            check_table=True)
        if not ok_r and how_r == 'table-collision':
            # Fast convergent search: test bounded XY/Z waypoint parameters
            # without motion, then execute only a route whose every segment
            # passed the same IK/FK, human-joint, and full swept-path checks.
            # If execution exposes a state-dependent collision, keep the cup
            # attached and advance to the next parameter candidate.
            _route_ok = False
            _route_trials = 0
            _route_reasons = {}
            # The same support pose can have distinct collision-free IK
            # branches. A prior full E2E reached this target through the
            # elbow-out branch; this run's human branch had zero clear
            # endpoint seeds. Search both branches, but preflight and execute
            # each route with the exact same branch and unchanged safety gates.
            # The elbow-out branch has historical full-E2E success; try it before the
            # slower human-seed search so CI reaches the known viable branch.
            _route_elbows = (['out', elbow] if elbow == 'human'
                             else [elbow])
            for _route_elbow in _route_elbows:
                for _candidate in putback_route_candidates(T_right_goal):
                    _route_trials += 1
                    _route_waypoint = _candidate['waypoint']
                    _preflight_ok, _preflight = preflight_table_route(
                        node, 'R', [_route_waypoint, T_right_goal],
                        elbow=_route_elbow)
                    _reason = ('preflight_clear' if _preflight_ok else
                               str(_preflight.get('reason', 'preflight_rejected')))
                    _route_reasons[_reason] = _route_reasons.get(_reason, 0) + 1
                    node.get_logger().info(
                        'PUTBACK_ROUTE_TRIAL step=%d trial=%d elbow=%s params=%s '
                        'preflight=%s detail=%s'
                        % (i, _route_trials, _route_elbow,
                           {k: _candidate[k] for k in ('height_m', 'dx_m', 'dy_m')},
                           _preflight_ok, _preflight))
                    if not _preflight_ok:
                        continue
                    _ok_stage, _stage_mm, _stage_how = reach(
                        node, 'R', _route_waypoint, tol_mm=8.0,
                        elbow=_route_elbow, check_table=True)
                    if not _ok_stage:
                        _reason = 'stage_' + str(_stage_how)
                        _route_reasons[_reason] = _route_reasons.get(_reason, 0) + 1
                        if _stage_how == 'table-collision':
                            continue
                        how_r, res_r = _stage_how, _stage_mm
                        break
                    ok_r, res_r, how_r = reach(
                        node, 'R', T_right_goal, tol_mm=6.0,
                        elbow=_route_elbow, check_table=True)
                    if ok_r:
                        _route_ok = True
                        node.get_logger().info(
                            'PUTBACK_ROUTE_ACCEPTED step=%d trial=%d elbow=%s '
                            'params=%s residual=%.1fmm'
                            % (i, _route_trials, _route_elbow,
                               {k: _candidate[k] for k in
                                ('height_m', 'dx_m', 'dy_m')}, res_r))
                        break
                    _reason = ('final_' + str(how_r))
                    _route_reasons[_reason] = _route_reasons.get(_reason, 0) + 1
                    if how_r != 'table-collision':
                        break
                if _route_ok or how_r != 'table-collision':
                    break
            node.get_logger().info(
                'PUTBACK_ROUTE_SUMMARY step=%d trials=%d accepted=%s reasons=%s'
                % (i, _route_trials, _route_ok, _route_reasons))
            if not _route_ok:
                node.get_logger().warn(
                    'PUTBACK_ROUTE_EXHAUSTED step=%d trials=%d; '
                    '保持双手持杯并阻止 release' % (i, _route_trials))
        if not ok_r:
            node.get_logger().error(
                'PUT_DOWN_TWO 第%d/%d步右手未到位 %.1fmm，停止下降并保持抓持'
                % (i, steps, res_r))
            if info is not None:
                info.update(release_blocked=True,
                            release_block_reason='right_arm_path_failed',
                            failed_step=i, right_res_mm=round(res_r, 1),
                            right_path_how=how_r)
            return False, res_r
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
    if not ok1 or not resting:
        node.get_logger().error(
            'PUT_DOWN_TWO 杯子未可靠落稳(ok=%s, 残差 %.1fmm > %.1f)，'
            '保持两手抓持，不张手、不 release'
            % (ok1, res, tol_mm))
        if info is not None:
            info.update(rest_mm=round(res, 1), release_blocked=True,
                        release_block_reason='place_not_confirmed')
        return False, res
    if info is not None:
        # P103: 落稳那一刻(手还捧着杯子、还没张指)的关节姿态 —— 验收要量的是
        # **放回这个动作**的位形, 不是随后 `hands_off` 抬手的位形
        info['arm_rest'] = dict((s, arm_state(node, s)) for s in ('L', 'R'))
        info.update(rest_mm=round(res, 1),
                    cup_z_rest=round(float(np.asarray(T_obj_goal)[2, 3]), 4),
                    hand_gap_rest=dict((k, round(v * 1e3, 1))
                                       for k, v in d_rest.items()),
                    hand_axis_rest=dict((k, round(v * 1e3, 1))
                                        for k, v in d_axis.items()))
    # 同时张开(指头离开杯壁), 再收一次位, 然后才松钉。
    # 张手任何一侧失败都不能解除附着。
    open_l, _err_l = open_hand(node, 'L')
    open_r, _err_r = open_hand(node, 'R')
    if not (open_l and open_r):
        node.get_logger().error(
            'PUT_DOWN_TWO 张手伺服失败（L=%s R=%s），保持杯子附着'
            % (open_l, open_r))
        if info is not None:
            info['release_blocked'] = True
            info['release_block_reason'] = 'hand_open_failed'
        return False, res
    settle_sim(node, 0.25)
    # 张手之后**只压实、不重挑分支**(elbow=None)。这一步的目标就是"再对一次位",
    # 手已经贴在杯壁上; 要是走 `elbow='human'` 就会在酒杯旁边重新做一次分支决策,
    # 实测有一轮把左臂挑成 `q=[−1.53,−1.30,+1.90,−2.42,0.42,0.71,+1.57]`
    # (up_roll 顶 R4、肘折 2.42 越 R7)—— 姿态难看之外, 这只"重新拧过"的手
    # 随后撤手/倒车时把杯子蹭走 20mm。压实就该沿用当前分支, 做局部跟踪。
    ok2, res2 = place_held(node, 'L', model, T_obj_goal, tol_mm, elbow=None)
    if not ok2:
        node.get_logger().error(
            'PUT_DOWN_TWO 张手后复核失败 %.1fmm，保持杯子附着，不 release'
            % res2)
        if info is not None:
            info['release_blocked'] = True
            info['release_block_reason'] = 'post_open_pose_not_confirmed'
        return False, res2
    release_ok = node.carry.release(model)
    if not release_ok:
        node.get_logger().error(
            'PUT_DOWN_TWO release ACK 未确认；停止张手后撤离动作，不移动底盘')
        if info is not None:
            info['release_blocked'] = True
            info['release_block_reason'] = 'release_ack_missing'
        return False, res2
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
                node.get_logger().error(
                    'HANDS_OFF[%s] 缺少掌心真值，停止撤手' % s)
                return False, 9e9
            T = np.asarray(Tp, float).copy()
            T[:3, 3] = T[:3, 3] + lifts[i][k]
            _ok, e, _how = reach(node, s, T, tol_mm=tol_mm, elbow=elbow,
                                 check_table=True)
            worst = max(worst, float(e))
            if not _ok:
                node.get_logger().error(
                    'HANDS_OFF[%s] 第%d段未完成 %.1fmm，停止后续撤手' %
                    (s, k + 1, e))
                if info is not None:
                    info.update(ok=False, failed_side=s, failed_stage=k + 1,
                                residual_mm=round(float(e), 1))
                return False, worst
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


def hang_natural(node, side, fold=None, info=None, stages=(1, 2, 3),
                 check_table=True):
    """分阶段执行人体式收臂；默认每段先做桌面/桌腿碰撞检查。

    stages=(1,) 可在放杯后立即启动安全预回收（折肘，肘尖固定）；
    stages=(2,3) 在底盘达到臂长净空后完成肩部回收和伸肘悬垂。
    返回 (结束时实测 q, 过程信息 dict)。碰撞检查不通过时不下发该段关节命令。
    """
    q0 = q_meas(node, side)
    out = {'ok': False, 'stages_done': [], 'table_clear': True}
    if q0 is None:
        out.update(reason='joint_state_missing')
        if info is not None:
            info.update(out)
        return None, out
    fold_v = K.HANG_FOLD if fold is None else fold
    el_start = K.elbow_pos(side, q0)
    palm0 = K.fk_world(side, q0, node.base)[:3, 3]
    zmin = 9e9
    xmax = -9e9
    wrist_step = 0.0
    q_prev = q0.copy()
    elbow_drift = 0.0

    for stage in stages:
        if stage not in (1, 2, 3):
            raise ValueError('hang_natural stage must be 1, 2 or 3')
        q_cur = q_meas(node, side)
        if q_cur is None:
            out.update(reason='joint_state_lost', ok=False)
            if info is not None:
                info.update(out)
            return None, out
        paths = K.hang_path_stages(q_cur, fold=fold_v)
        path = paths[stage - 1]
        if not path:
            out['stages_done'].append(stage)
            continue
        if check_table:
            clear, hit = arm_path_table_clear(
                node, side, path, ignore_initial=True)
            if not clear:
                out.update(ok=False, table_clear=False,
                           table_block=hit, blocked_stage=stage)
                node.get_logger().warn(
                    'HANG[%s] 第%d段因桌面/桌腿碰撞风险被拦截: %s'
                    % (side, stage, hit))
                if info is not None:
                    info.update(out)
                return q_cur, out
        el_before = K.elbow_pos(side, q_cur)
        for path_i, q in enumerate(path):
            ok_move, err_move = node.arm[side].move(
                q, timeout=6.0, tol_deg=3.0)
            if not ok_move:
                out.update(ok=False, reason='joint_servo_timeout',
                           blocked_stage=stage, blocked_waypoint=path_i,
                           max_joint_error_deg=round(
                               float(np.max(np.abs(np.degrees(err_move)))), 2))
                node.get_logger().error(
                    'HANG[%s] 第%d段路点%d伺服超时；保持当前姿态，不跳到下一段'
                    % (side, stage, path_i))
                if info is not None:
                    info.update(out)
                return q_meas(node, side), out
            wrist_step = max(
                wrist_step,
                float(np.max(np.abs(q[[4, 5, 6]] - q_prev[[4, 5, 6]]))))
            q_prev = q
            palm = K.fk_world(side, q, node.base)[:3, 3]
            zmin = min(zmin, float(palm[2]))
            xmax = max(xmax, float(palm[0] - node.base[0]))
        q_after = q_meas(node, side)
        if q_after is None:
            out.update(ok=False, reason='joint_state_lost_after_move')
            if info is not None:
                info.update(out)
            return None, out
        if stage == 1:
            el_after = K.elbow_pos(side, q_after)
            elbow_drift = float(np.linalg.norm(el_after - el_before)) * 1e3
        out['stages_done'].append(stage)

    if 3 in stages:
        # 路点最终点本身即 Q_HANG；但伺服完成后的实测姿态可能有残差。
        # 最后的压实动作仍需从最新实测关节状态重新经过桌面/桌腿门控，
        # 不能因为前一条规划路径已检查就跳过最后一次纠偏运动的安全检查。
        if check_table:
            clear, hit = arm_path_table_clear(
                node, side, [K.Q_HANG], ignore_initial=True)
            if not clear:
                out.update(ok=False, table_clear=False,
                           table_block=hit, blocked_stage=3,
                           reason='hang_settle_table_collision')
                node.get_logger().warn(
                    'HANG[%s] 最终悬垂压实因桌面/桌腿碰撞风险被拦截: %s'
                    % (side, hit))
                if info is not None:
                    info.update(out)
                return q_meas(node, side), out
        # 最终压实目标并收拢手指。
        ok_arm, err_arm = node.arm[side].move(K.Q_HANG, timeout=8.0)
        if not ok_arm:
            out.update(ok=False, reason='hang_settle_timeout',
                       max_joint_error_deg=round(
                           float(np.max(np.abs(np.degrees(err_arm)))), 2))
            if info is not None:
                info.update(out)
            return q_meas(node, side), out
        ok_hand, err_hand = node.hand[side].move(
            K.grip_values(side, K.GRIP_HANG), timeout=6.0)
        if not ok_hand:
            out.update(ok=False, reason='hang_hand_timeout')
            if info is not None:
                info.update(out)
            return q_meas(node, side), out
    q1 = q_meas(node, side)
    if q1 is None:
        out.update(ok=False, reason='joint_state_missing_after_recovery')
        if info is not None:
            info.update(out)
        return None, out
    out.update(
        ok=True,
        elbow_drift_mm=round(elbow_drift, 1),
        fold_deg=round(math.degrees(min(float(q0[3]), fold_v) - float(q0[3])), 1),
        wrist_deg=round(math.degrees(
            abs(float(q0[4])) + abs(float(q0[5])) + abs(float(q0[6]))), 1),
        wrist_step_deg=round(math.degrees(wrist_step), 1),
        hand_min_z=None if zmin == 9e9 else round(zmin, 3),
        hand_max_x=None if xmax == -9e9 else round(xmax, 3),
        q0=[round(float(v), 2) for v in q0],
        palm0_m=[round(float(palm0[0] - node.base[0]), 3),
                 round(float(palm0[1] - node.base[1]), 3),
                 round(float(palm0[2]), 3)],
        gap_m=round(table_gap_m(node), 3),
        reach_m=round(K.ARM_REACH, 3),
    )
    node.get_logger().info(
        'HANG[%s] stages=%s 肘点漂移 %.1fmm 腕单步 %.1f° '
        '桌边净距 %.2fm/%s'
        % (side, out['stages_done'], out['elbow_drift_mm'],
           out['wrist_step_deg'], out['gap_m'],
           'guarded' if check_table else 'unguarded'))
    if info is not None:
        info.update(out)
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
                'LIFT[%s] 预备位 IK 未收敛 %.1fmm (%s)，停止抬臂' %
                (s, err * 1e3, how))
            ok_all = False
            errs[s] = 9e9
            continue
        path_ok = True
        for path_i, qq in enumerate(K.raise_path(q0, q)):
            _move_ok, _move_err = node.arm[s].move(
                qq, timeout=6.0, tol_deg=3.0)
            if not _move_ok:
                node.get_logger().error(
                    'LIFT[%s] 第%d个预备路点伺服超时，停止后续抬臂' %
                    (s, path_i))
                path_ok = False
                break
        if path_ok:
            path_ok, _move_err = node.arm[s].move(q, timeout=6.0)
        if not path_ok:
            ok_all = False
            errs[s] = 9e9
            continue
        # 路点走完只保证"关节进了 3° 容差", 7 轴串下来指尖还有 20mm 级残差
        # (实测 17~25mm)。预备位是后面抓杯的起点, 差 20mm 就要靠抓取那一步
        # 自己补 —— 用与全局一致的闭环再压一次。
        _ok, e, _how2 = reach(node, s, T, tol_mm=5.0, passes=3)
        if e > 4.0 and not _ok:
            _ok2, e2, _how3 = reach(
                node, s, T, tol_mm=3.0, passes=2, tol_deg=0.08)
            if e2 < e:
                e = e2
            _ok = _ok2
        elif e > 4.0:
            # 还有 5~8mm 的不动点(见 reach 的 tol_deg 说明): 把伺服判据逼紧
            # 再补一轮。预备位是抓杯的起点, 差 7mm 就得让抓取那一步自己补。
            _ok2, e2, _how3 = reach(node, s, T, tol_mm=3.0, passes=2,
                                     tol_deg=0.08)
            if e2 < e:
                e = e2
        errs[s] = round(float(e), 1)
        ok_all = ok_all and ok and _ok
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
    ok_reach, reach_res, _how = reach(
        node, side, palm_fn(T_obj), tol_mm=tol_mm, elbow=elbow)
    if not ok_reach:
        node.get_logger().warn(
            'HAND_ONTO[%s] 未贴合到位 %.1fmm，不冻结双手相对偏移' %
            (side, reach_res))
        return None
    settle_sim(node, 0.30)                      # 等停稳再冻结, 否则量到滑行中的值
    T_obj = object_pose(node, model)
    Tp = wrist(node, side)
    if T_obj is None or Tp is None:
        return None
    return np.linalg.inv(T_obj) @ Tp


def follow_hand(node, side, model, off, tol_mm=5.0, elbow='human',
                check_table=False):
    """第二只手按冻结偏移跟上物体当前位置。"""
    T_obj = object_pose(node, model)
    if T_obj is None or off is None:
        return False, 9e9
    Tp_goal = np.asarray(T_obj, float) @ off
    ok, res, _how = reach(node, side, Tp_goal, tol_mm=tol_mm,
                          elbow=elbow, check_table=check_table)
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
    if not ok_c or not human_kinematics_ok(out['arm']['grasp_cup'], required_sides=('L',)):
        node.get_logger().error(
            'POUR 抓杯阶段未达成或手臂违反人体关节规则，停止后续倒水')
        return out
    Tcup_pour = P.cup_for_pour()
    # P103: 举杯要走"肘朝外"的解 —— 与捧杯/递杯同一套(P101 的经验)。默认的
    # KDL 种子解在这里会给出肘内收/前臂反向拧的分支, 画面上就是"肘拐进怀里"。
    # 这一步既要满足杯体严格 <3mm 的位置闭环，也要满足 P103 人体姿态规则。
    # 最近实测杯体残差 3.4mm，略高于严格 3.0mm 门槛；每轮 reach 原先
    # 使用 3.0mm 掌心容差，可能过早停止补偿。将单轮掌心容差收紧至可调
    # 0.5–1.5mm，最多 8 轮；杯体 <3.0mm、姿态与 IK/碰撞门槛完全不变。
    ok_l, res_l = place_held(
        node, 'L', 'cup', Tcup_pour, tol_mm=3.0,
        reach_passes=8, reach_tol_mm=_cup_lift_reach_tol_mm(),
        reach_tol_deg=0.08)
    out['res']['lift_cup'] = res_l
    out['arm']['lift_cup'] = log_arm(node, '举杯', ('L',))
    lift_kin_ok = human_kinematics_ok(
        out['arm']['lift_cup'],
        min_out_line_mm={'L': 100.0},
        max_abs_droop_mm={'L': 120.0},
        max_abs_twist={'L': 2.2}, required_sides=('L',))
    if not ok_l:
        node.get_logger().error(
            'POUR 举杯位置闭环失败: 杯体残差 %.1fmm，严格要求 <3.0mm；'
            '保持抓持并停止后续倒水' % float(res_l))
        return out
    if not lift_kin_ok:
        node.get_logger().error(
            'POUR 举杯 P103 姿态判据失败: metrics=%s；保持抓持并停止后续倒水'
            % out['arm']['lift_cup'].get('L'))
        return out
    ok_k, res_k = grasp(node, 'R', 'kettle', P.kettle_grasp_palm, K.GRIP_KETTLE)
    out['res']['grasp_kettle'] = res_k
    out['arm']['grasp_kettle'] = log_arm(node, '抓壶', ('R',))
    if not ok_k or not human_kinematics_ok(out['arm']['grasp_kettle'], required_sides=('R',)):
        node.get_logger().error(
            'POUR 抓壶阶段未达成或手臂违反人体关节规则，停止后续倒水')
        return out
    Tk_pour, tip_tgt = P.kettle_for_pour(object_pose(node, 'cup'))
    # The default 0.25deg servo deadband left a repeatable 5–8mm palm
    # residual in the kettle-pour pose. Use the same tight refinement strategy
    # already exercised for cup lift; only the internal servo stop criterion
    # changes. Object acceptance remains strictly <4mm and <2deg.
    ok_p, res_p = place_held(
        node, 'R', 'kettle', Tk_pour, tol_mm=4.0, reach_passes=8,
        reach_tol_deg=_kettle_pour_reach_tol_deg(),
        reach_require_base_still=(
            os.environ.get('VLA_PREPARE_KETTLE_BASE_STILL', '0') == '1'))
    out['res']['kettle_pour'] = res_p
    out['arm']['pour'] = log_arm(node, '倒水', ('L', 'R'))
    pour_kin_ok = human_kinematics_ok(
        out['arm']['pour'], min_out_line_mm={'R': 60.0},
        required_sides=('L', 'R'))
    if not ok_p:
        node.get_logger().error(
            'POUR 倒水位置闭环失败: 水壶目标残差 %.1fmm（严格要求 <4.0mm）；'
            '保持抓持并禁止启动水柱；arm=%s'
            % (float(res_p), out['arm']['pour']))
        return out
    if not pour_kin_ok:
        node.get_logger().error(
            'POUR 倒水 P103 姿态判据失败: metrics=%s；'
            '保持抓持并禁止启动水柱' % out['arm']['pour'])
        return out
    # 水柱先伸出并确认目标指令已被插件接收，再启动水量积分。
    # ramp ACK 丢失时用一次性 joint ACK 重试；两条都失败就不启动水流，
    # 避免“杯内水位上涨但水柱没有伸出”或倒水半途继续执行。
    liq = getattr(node, 'liquid', None)
    if ramp:
        jet_ok = node.carry.joint_ramp(
            'kettle', 'jet_joint', jet_len, dur=0.5)
        if not jet_ok:
            jet_ok = node.carry.joint('kettle', 'jet_joint', jet_len)
        if not jet_ok:
            node.get_logger().error(
                'POUR 水柱伸出指令无 ACK；不启动液体积分，保持道具抓持')
            out['reason'] = 'jet_extend_unconfirmed'
            return out
        if liq is None:
            water_ok = node.carry.joint_ramp(
                'cup', 'water_joint', L.CUP_WATER_MAX * fill, dur=1.2)
            if not water_ok:
                water_ok = node.carry.joint(
                    'cup', 'water_joint', L.CUP_WATER_MAX * fill)
            if not water_ok:
                node.carry.joint('kettle', 'jet_joint', 0.0)
                node.get_logger().error(
                    'POUR 杯内水位驱动无 ACK；已请求收回水柱，不报告倒水成功')
                out['reason'] = 'water_level_unconfirmed'
                return out
    else:
        jet_ok = node.carry.joint('kettle', 'jet_joint', jet_len)
        if not jet_ok:
            node.get_logger().error(
                'POUR 水柱目标未确认，不启动倒水')
            out['reason'] = 'jet_extend_unconfirmed'
            return out
        if liq is None:
            water_ok = node.carry.joint(
                'cup', 'water_joint', L.CUP_WATER_MAX * fill)
            if not water_ok:
                node.carry.joint('kettle', 'jet_joint', 0.0)
                node.get_logger().error(
                    'POUR 杯内水位目标未确认，已请求收回水柱')
                out['reason'] = 'water_level_unconfirmed'
                return out
    if liq is not None:
        liq.pour_start(fill)
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
    stand_mm = place_robot(node, T_stand)
    if stand_mm >= 8.0:
        out.update(ok=False, reason='stand_pose_not_confirmed',
                   stand_mm=round(float(stand_mm), 1))
        node.get_logger().error(
            'POUR 入场无法确认远端待命位（%.1fmm），不执行 home/抬臂' % stand_mm)
        return out
    home_ok = node.home()            # 在远处复位(臂 Q_REST、十指张开、水流归零)
    if not home_ok:
        out.update(ok=False, reason='home_failed')
        node.get_logger().error('POUR 入场 home 复位失败，停止抬臂')
        return out
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
    snap_ok = bool(snap < 8.0)
    out.update(gap_after=round(table_gap_m(node), 3),
               drive_mm=round(res_d, 1), snap_mm=round(snap, 1),
               snap_ok=snap_ok, home_ok=bool(home_ok),
               ok=bool(ok_l and ok_d and snap_ok and home_ok))
    if not out['ok']:
        out['reason'] = ('lift_ready_failed' if not ok_l else
                         'drive_to_service_failed' if not ok_d else
                         'service_pose_snap_failed')
        node.get_logger().error('POUR 入场未完成: %s' % out['reason'])
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
        node.get_logger().error(
            'PREPARE 入场未通过（抬臂/驶入未确认），中止桌边操作，避免带着异常姿态伸手')
        return out
    Tk_home = object_pose(node, 'kettle')
    if Tk_home is None:
        out['reason'] = 'kettle_pose_missing'
        node.get_logger().error('PREPARE 缺少水壶真值位姿，停止倒水')
        return out
    # The arm-only pour sequence can push the free-rolling chassis even when
    # both velocity command channels are zero. Hold the measured service pose
    # only across this arm-motion window; release it before base repositioning.
    _base_hold_snapshot = getattr(node, 'truth_base_pose_snapshot', None)
    _base_hold_now = time.monotonic()
    if (_base_hold_snapshot is not None
            and _base_hold_now - _base_hold_snapshot[1] <= 0.25):
        node._prepare_base_hold_target = tuple(_base_hold_snapshot[0])
        node.get_logger().info(
            'PREPARE_BASE_HOLD start target=%s' %
            (list(node._prepare_base_hold_target),))
    else:
        node._prepare_base_hold_target = None
        node.get_logger().warn(
            'PREPARE_BASE_HOLD unavailable: chassis truth stale')
    try:
        pour = pour_into_cup(node, fill=fill)
    finally:
        node._prepare_base_hold_target = None
        node.drive(0.0, 0.0)
    out.update(pour)
    if not pour['ok']:
        out['ok'] = False
        return out
    # 倒水子流程成功不代表 PREPARE 已完成；水壶放回、右臂归位、底盘复位、
    # 捧杯/副手贴合和转身仍可能失败。最终聚合之前总状态必须保持 False。
    out['ok'] = False
    # 收水: 先确认水柱缩回命令，再结束液体积分；无 ACK 时不移动水壶。
    jet_retract_ok = node.carry.joint_ramp(
        'kettle', 'jet_joint', 0.0, dur=0.5)
    if not jet_retract_ok:
        jet_retract_ok = node.carry.joint('kettle', 'jet_joint', 0.0)
    if not jet_retract_ok:
        out['reason'] = 'jet_retract_unconfirmed'
        node.get_logger().error(
            'PREPARE 水柱无法确认缩回，保持水壶抓持，不执行放壶/转身')
        return out
    if getattr(node, 'liquid', None) is not None:
        node.liquid.pour_stop()      # P98: 水柱已回收, 再停止液体积分
    kettle_putdown = {}
    ok_d, res_d = put_down(
        node, 'R', 'kettle', Tk_home, info=kettle_putdown)
    out['res']['kettle_back'] = res_d
    out['kettle_putdown'] = kettle_putdown
    if not ok_d:
        if kettle_putdown.get('release_confirmed'):
            out['reason'] = 'kettle_retract_failed_after_release'
            node.get_logger().error(
                'PREPARE 水壶已确认释放并落桌，但右臂撤离失败；'
                '停止后续动作，物体状态=released')
        else:
            out['reason'] = 'kettle_release_or_placement_unconfirmed'
            node.get_logger().error(
                'PREPARE 水壶未确认释放/落稳，保持抓持状态并中止后续动作')
        return out
    ok_r_home, err_r_home = node.arm['R'].move(K.Q_REST, timeout=10.0)
    if not ok_r_home:
        out['reason'] = 'right_arm_home_failed'
        node.get_logger().error(
            'PREPARE 右臂回 Q_REST 超时(最大误差 %.1f°)，保持杯子抓持并中止'
            % math.degrees(float(np.nanmax(np.abs(err_r_home)))))
        return out
    node.spin_for(0.4)
    serve_mm = place_robot(node, T_serve)
    if serve_mm >= 8.0:
        out['reason'] = 'service_pose_reset_failed'
        node.get_logger().error(
            'PREPARE 服务位底盘摆正失败 %.1fmm，保持杯子抓持并中止'
            % serve_mm)
        return out
    bx, by, byaw = node.base
    Tcup_carry = P.cup_in_base(bx, by, byaw, L.CUP_CARRY_R, L.CUP_CARRY_Z)
    # P101: 捧杯/贴手都走"肘朝外"解 —— 抱着杯子时肘得像人一样架在体侧,
    # 而不是垂在下面(原来 KDL 用 home 当种子, 两侧肘都朝下)。
    # Cup-carry reaches over the table edge; use the same interpolated
    # capsule/OBB gate as put-down withdrawals so physical contact cannot
    # shove the mobile base while the arm is still converging.
    ok_c, res_c = place_held(
        node, 'L', 'cup', Tcup_carry, tol_mm=5.0, check_table=True)
    out['res']['cup_carry'] = res_c
    if not ok_c:
        node.get_logger().error(
            'PREPARE 左手双手捧杯位形未到位（%.1fmm），保持抓持并中止转身' % res_c)
        return out
    off_r = hand_onto(node, 'R', 'cup', lambda T: P.cup_cradle_palm(T, 'R'))
    if off_r is None:
        node.get_logger().error(
            'PREPARE 右手未能贴合水杯，保持左手抓持并中止递杯准备')
        return out
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
    face_mm = place_robot(node, T_face)
    face_ok = bool(face_mm < 8.0)
    out.update({'ok': bool(ok_d and ok_c and off_r is not None and ok_t
                            and face_ok),
                'turn_ok': ok_t, 'turn_err': err_t,
                'service_reset_mm': round(serve_mm, 1),
                'face_reset_mm': round(face_mm, 1),
                'face_reset_ok': face_ok,
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
    # Predict where both palms will actually be after the scripted close
    # motion. The shoulder-root nominal point is not the midpoint of the
    # articulated child palms and can be off by >40mm in a rotated scene.
    Th = person_hand_midpoint_world(node, L.PERSON_HAND_CLOSE)
    if Th is None:
        target = cup_present_target(node)
        node.get_logger().warn(
            'PRESENT_TARGET source=nominal_fallback reason=hand_midpoint_unavailable '
            'base=%s target_xyz=%s' %
            (np.round(node.base, 5).tolist(),
             np.round(target[:3, 3], 5).tolist()))
        return target
    dx, dy = Th[0, 3] - bx, Th[1, 3] - by
    d_hb = math.hypot(dx, dy)
    if d_hb < 0.30:            # 数据不可信(人贴太近), 退回名义位形
        target = cup_present_target(node)
        node.get_logger().warn(
            'PRESENT_TARGET source=nominal_fallback reason=hand_distance_below_300mm '
            'd_hand_base=%.5f base=%s hand_xyz=%s target_xyz=%s' %
            (d_hb, np.round(node.base, 5).tolist(),
             np.round(Th[:3, 3], 5).tolist(),
             np.round(target[:3, 3], 5).tolist()))
        return target
    # P94: 上限从 CUP_PRESENT_R(0.62) 放宽到 CUP_PRESENT_R_MAX(0.70) ——
    # 0.62 是"人站准"的解; 人往外站 0.15m 时车→人手 0.77m, 卡在 0.62
    # 会把杯子停在离手 150mm 处。0.70 是 diag_p94_reach.py 扫出来的
    # 安全上限(再远就顶臂长)。不够的部分由 `_approach_if_far` 补。
    r = max(0.50, min(L.CUP_PRESENT_R_MAX, d_hb))
    yaw = math.atan2(dy, dx)
    target = P.cup_in_base(bx, by, yaw, r, L.CUP_PRESENT_Z)
    node.get_logger().info(
        'PRESENT_TARGET source=predicted_closed_midpoint base=%s hand_xyz=%s '
        'd_hand_base=%.5f yaw_deg=%.3f radius=%.5f target_xyz=%s '
        'target_minus_hand_mm=%s' %
        (np.round(node.base, 5).tolist(),
         np.round(Th[:3, 3], 5).tolist(), d_hb, math.degrees(yaw), r,
         np.round(target[:3, 3], 5).tolist(),
         np.round((target[:3, 3] - Th[:3, 3]) * 1000.0, 2).tolist()))
    return target


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


def person_hand_gap_expected_mm(openness):
    """由双臂外展关节目标幅度估算双手心目标间距（端点标定：闭合184/张开345mm）。"""
    close = float(L.PERSON_HAND_CLOSE)
    open_ = float(L.PERSON_HAND_OPEN)
    span = open_ - close
    if abs(span) < 1e-9:
        return 184.0
    ratio = max(0.0, min(1.0, (float(openness) - close) / span))
    return 184.0 + ratio * (345.0 - 184.0)


def person_hand_gap_ok(gap_mm, openness, tolerance_mm=45.0):
    """使用 Gazebo 真值手心间距验证人物双手确已到达张开/合拢目标。"""
    if gap_mm is None:
        return False
    try:
        gap = float(gap_mm)
        target = person_hand_gap_expected_mm(openness)
        tolerance = float(tolerance_mm)
    except (TypeError, ValueError):
        return False
    return (math.isfinite(gap) and math.isfinite(target)
            and math.isfinite(tolerance) and gap >= 0.0
            and tolerance >= 0.0 and abs(gap - target) <= tolerance)


def person_hands(node, openness, dur=0.6, dt=0.05, tag=''):
    """人的双手斜坡开合，并通过 Gazebo 真值手心间距验收到位。"""
    cur = float(node.bus.get('person_arm_l_joint'))
    n = max(1, int(dur / max(dt, 1e-3)))
    for i in range(1, n + 1):
        v = cur + (float(openness) - cur) * float(i) / n
        node.bus.set(['person_arm_l_joint', 'person_arm_r_joint'], [v, -v])
        node.spin_for(dt)
    node.spin_for(0.25)          # 让 JPC 把最后一段走完再读真值
    gap = hand_gap_mm(node)
    target = person_hand_gap_expected_mm(openness)
    ok = person_hand_gap_ok(gap, openness)
    node.get_logger().info(
        '双手%s: 命令幅度 %.3f rad, 真值手心距 %s mm / 目标 %.1f±45mm %s'
        % (tag, openness,
           '缺失' if gap is None else '%.1f' % gap,
           target, '到位' if ok else '未到位'))
    return bool(ok)


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


def person_hand_midpoint_predict(T_left_palm, T_right_palm,
                                 q_left, q_right, openness,
                                 left_offset=None, right_offset=None):
    """Predict the two palm world poses after both articulated hand joints move.

    The person hand links rotate about their shoulder origins around local +Z.
    Inputs are current palm world transforms and measured joint positions;
    target right joint is the mirrored negative openness command.
    """
    try:
        Tl = np.asarray(T_left_palm, dtype=float)
        Tr = np.asarray(T_right_palm, dtype=float)
        ql, qr, target = float(q_left), float(q_right), float(openness)
        ol = (L.PERSON_HAND_IN_ARML if left_offset is None
              else tuple(float(v) for v in left_offset))
        or_ = (L.PERSON_HAND_IN_ARMR if right_offset is None
               else tuple(float(v) for v in right_offset))
        if (Tl.shape != (4, 4) or Tr.shape != (4, 4)
                or not np.all(np.isfinite(Tl)) or not np.all(np.isfinite(Tr))
                or not all(math.isfinite(v) for v in (ql, qr, target))
                or len(ol) != 3 or len(or_) != 3
                or not all(math.isfinite(v) for v in ol + or_)):
            return None
        # Recover each child-link frame from its measured palm frame, then
        # apply only the commanded joint delta about that link's local origin.
        Tll = Tl @ np.linalg.inv(K.trans(ol))
        Trl = Tr @ np.linalg.inv(K.trans(or_))
        Tlp = Tll @ K.frame(K.rot_z(target - ql)) @ K.trans(ol)
        Trp = Trl @ K.frame(K.rot_z(-target - qr)) @ K.trans(or_)
        midpoint = 0.5 * (Tlp[:3, 3] + Trp[:3, 3])
        if not np.all(np.isfinite(midpoint)):
            return None
        return K.trans(midpoint)
    except (ValueError, TypeError, np.linalg.LinAlgError, OverflowError):
        return None


def person_hand_joint_angles_from_truth(T_arms_world, T_left_link_world,
                                           T_right_link_world):
    """Recover revolute joint angles from Gazebo parent/child link poses."""
    try:
        parent = np.asarray(T_arms_world, dtype=float)
        left = np.asarray(T_left_link_world, dtype=float)
        right = np.asarray(T_right_link_world, dtype=float)
        if any(T.shape != (4, 4) or not np.all(np.isfinite(T))
               for T in (parent, left, right)):
            return None
        Rp = parent[:3, :3]
        rel_l = Rp.T @ left[:3, :3]
        rel_r = Rp.T @ right[:3, :3]
        # SDF declares both child joints revolute about parent-local +Z.
        for rel in (rel_l, rel_r):
            if (abs(float(rel[2, 2]) - 1.0) > 0.02
                    or abs(float(rel[0, 2])) > 0.02
                    or abs(float(rel[1, 2])) > 0.02):
                return None
        ql = math.atan2(float(rel_l[1, 0]), float(rel_l[0, 0]))
        qr = math.atan2(float(rel_r[1, 0]), float(rel_r[0, 0]))
        if (not math.isfinite(ql) or not math.isfinite(qr)
                or abs(ql) > 0.65 or abs(qr) > 0.65):
            return None
        return ql, qr
    except (TypeError, ValueError, np.linalg.LinAlgError, OverflowError):
        return None


def person_hand_midpoint_if_at_target(T_left_palm, T_right_palm,
                                         openness, tolerance_mm=15.0):
    """Use measured palm truth directly when the measured gap matches target.

    This avoids reconstructing a joint angle for a pose that is already at the
    requested opening. It is especially important when Gazebo link truth is
    available but /drink/joint_state omits the person's articulated joint names.
    A different measured opening is never treated as the requested pose.
    """
    try:
        Tl = np.asarray(T_left_palm, dtype=float)
        Tr = np.asarray(T_right_palm, dtype=float)
        target = person_hand_gap_expected_mm(openness)
        tolerance = float(tolerance_mm)
        if (Tl.shape != (4, 4) or Tr.shape != (4, 4)
                or not np.all(np.isfinite(Tl))
                or not np.all(np.isfinite(Tr))
                or not math.isfinite(target)
                or not math.isfinite(tolerance) or tolerance < 0.0):
            return None
        gap_mm = float(np.linalg.norm(Tl[:3, 3] - Tr[:3, 3])) * 1000.0
        if not math.isfinite(gap_mm) or abs(gap_mm - target) > tolerance:
            return None
        midpoint = 0.5 * (Tl[:3, 3] + Tr[:3, 3])
        if not np.all(np.isfinite(midpoint)):
            return None
        return K.trans(midpoint)
    except (TypeError, ValueError, OverflowError):
        return None

def person_hand_midpoint_from_confirmed_command(T_left_palm, T_right_palm,
                                                q_left, q_right, openness,
                                                tolerance_mm=15.0):
    """Predict a new opening from the commanded pose only after truth confirms it.

    The person joint names are not always present in JointState. The symmetric
    command is accepted as the current angle only when both command values are
    finite, mirrored, within the configured range, and the measured palm gap
    independently matches the gap expected at that command.
    """
    try:
        ql, qr = float(q_left), float(q_right)
        close, open_ = float(L.PERSON_HAND_CLOSE), float(L.PERSON_HAND_OPEN)
        if (not math.isfinite(ql) or not math.isfinite(qr)
                or ql < close - 0.02 or ql > open_ + 0.02
                or abs(qr + ql) > 0.02):
            return None
        Tl = np.asarray(T_left_palm, dtype=float)
        Tr = np.asarray(T_right_palm, dtype=float)
        if (Tl.shape != (4, 4) or Tr.shape != (4, 4)
                or not np.all(np.isfinite(Tl))
                or not np.all(np.isfinite(Tr))):
            return None
        gap_mm = float(np.linalg.norm(Tl[:3, 3] - Tr[:3, 3])) * 1000.0
        expected = person_hand_gap_expected_mm(ql)
        if (not math.isfinite(gap_mm) or not math.isfinite(expected)
                or abs(gap_mm - expected) > float(tolerance_mm)):
            return None
        return person_hand_midpoint_predict(Tl, Tr, ql, qr, openness)
    except (TypeError, ValueError, OverflowError):
        return None

def person_hand_midpoint_world(node, openness=None):
    """Predict closed-hand midpoint using measured Gazebo link-pose angles."""
    if openness is None:
        openness = L.PERSON_HAND_CLOSE
    T_person = object_pose(node, 'person')
    Tl, Tr = person_hand_poses(node, T_person)
    if Tl is None or Tr is None:
        return None
    # If Gazebo truth already shows the requested opening, its measured palm
    # midpoint is authoritative; do not require a separate joint-state channel
    # or reconstruct an unnecessary angle from parent/child orientations.
    measured = person_hand_midpoint_if_at_target(Tl, Tr, openness)
    if measured is not None:
        return measured
    # When the requested pose differs (PRESENT asks for closed palms after
    # the person has opened them), use the mirrored command as the current
    # angle only after Gazebo palm-gap truth validates that commanded pose.
    bus = getattr(node, 'bus', None)
    if bus is not None:
        commanded = person_hand_midpoint_from_confirmed_command(
            Tl, Tr, bus.get('person_arm_l_joint'),
            bus.get('person_arm_r_joint'), openness)
        if commanded is not None:
            return commanded
    T_arms = person_hand_world(node, T_person, link='person_arms',
                               local_off=(0.0, 0.0, 0.0))
    T_left_link = person_hand_world(node, T_person, link='person_arm_l',
                                    local_off=(0.0, 0.0, 0.0))
    T_right_link = person_hand_world(node, T_person, link='person_arm_r',
                                     local_off=(0.0, 0.0, 0.0))
    if T_arms is None or T_left_link is None or T_right_link is None:
        return None
    angles = person_hand_joint_angles_from_truth(
        T_arms, T_left_link, T_right_link)
    if angles is None:
        return None
    return person_hand_midpoint_predict(Tl, Tr, angles[0], angles[1],
                                        openness)


def person_hand_midpoint_offset(T_arms_world, T_left_world, T_right_world):
    """Return the measured two-palm midpoint in the person_arms local frame.

    The carry host is the shoulder/root link, while the palms belong to two
    independently articulated child links. A fixed nominal offset is therefore
    not guaranteed to coincide with their realized midpoint after articulation.
    """
    try:
        T_arms = np.asarray(T_arms_world, dtype=float)
        T_left = np.asarray(T_left_world, dtype=float)
        T_right = np.asarray(T_right_world, dtype=float)
        if (T_arms.shape != (4, 4) or T_left.shape != (4, 4)
                or T_right.shape != (4, 4)
                or not np.all(np.isfinite(T_arms))
                or not np.all(np.isfinite(T_left))
                or not np.all(np.isfinite(T_right))):
            return None
        midpoint = 0.5 * (T_left[:3, 3] + T_right[:3, 3])
        local = np.linalg.inv(T_arms) @ np.append(midpoint, 1.0)
        if not np.all(np.isfinite(local)) or abs(float(local[3])) < 1e-9:
            return None
        offset = local[:3] / local[3]
        return tuple(float(v) for v in offset)
    except (ValueError, TypeError, np.linalg.LinAlgError, OverflowError):
        return None


def hand_gap_mm(node, T_person=None):
    """两手心的间距(mm)，真值测量。合拢 ≈184mm，张开 ≈345mm。"""
    Tl, Tr = person_hand_poses(node, T_person)
    if Tl is None or Tr is None:
        return None
    return float(np.linalg.norm(Tl[:3, 3] - Tr[:3, 3])) * 1000.0


def person_handover_geometry_ok(distances_mm, target_mm=92.0,
                               tolerance_mm=25.0):
    """Fail-closed check that both palms are within the accepted cup-axis band."""
    try:
        values = tuple(float(v) for v in distances_mm)
        target = float(target_mm)
        tolerance = float(tolerance_mm)
    except (TypeError, ValueError, OverflowError):
        return False
    if (len(values) != 2 or not math.isfinite(target)
            or not math.isfinite(tolerance) or tolerance < 0.0):
        return False
    return all(math.isfinite(v) and v >= 0.0
               and abs(v - target) <= tolerance for v in values)


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
