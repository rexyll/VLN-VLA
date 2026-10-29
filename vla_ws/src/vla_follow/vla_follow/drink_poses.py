#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""drink_poses.py —— 喝水任务的**全部关键位形**唯一计算处 (P84)

设计要点:
  1. 所有位形都以 4x4 齐次矩阵表达(世界系), 由 scene_layout 的坐标 + 抓取
     几何推出, drink_task 与 tools/check_ik.py 用的是同一批函数 —— 离线校验
     的结果就是线上会执行的东西。
  2. 抓取一律走 arm_kin.palm_target(): "物体位姿 + 手指朝向/掌心法向 +
     物体在掌内的位置" → 手掌目标位姿。SDF 改了手指尺寸只需改
     arm_kin.OBJ_IN_PALM_*, 这里不用动。
  3. 倒水位形由"杯口目标点 + 倾角"反解壶的位置: 壶嘴尖必须落在杯口正上方
     jet_gap 处, 否则水柱接不上。
"""
import math

import numpy as np

from vla_follow import arm_kin as K
from vla_follow import scene_layout as L

# ---- 抓取几何(物体坐标系下的手指朝向 / 掌心法向) ----
# 水壶: 竖直把手条, 手从机器人一侧(-x)水平伸过去, 五指沿把手上下一字排开
KETTLE_APPROACH = (0.0, -1.0, 0.0)     # 手指指向 = 壶体坐标 -y
KETTLE_CURL = (1.0, 0.0, 0.0)          # 掌心朝向 = 壶体坐标 +x
# 水杯: 从 +y 侧横抓(左手) / 从 -y 侧横抓(右手)
CUP_APPROACH = (1.0, 0.0, 0.0)
CUP_CURL_LEFT = (0.0, -1.0, 0.0)
CUP_CURL_RIGHT = (0.0, 1.0, 0.0)

# 倒水/搬运参数(可被 drink_config.json 覆盖)
KETTLE_TILT = math.radians(68.0)      # 倒水倾角
JET_GAP = 0.075                       # 壶嘴尖到杯口的竖直距离
CUP_LIFT_Z = 0.86                     # 倒水时杯心高度
CUP_LIFT_DX = 0.0                     # 倒水时杯子相对桌面位置的 x 偏移
CUP_LIFT_DY = 0.0


def frame(R, t):
    return K.frame(np.asarray(R, float), np.asarray(t, float))


def kettle_R(tilt=0.0):
    """壶体世界姿态: 先绕 z 转 yaw(壶嘴朝 +y), 再绕世界 x 轴倒 tilt(顶往 +y 倾)"""
    return K.rot_x(-tilt) @ K.rot_z(L.KETTLE_YAW)


# ---------------------------------------------------------------- 桌面物件
def kettle_on_table(tilt=0.0):
    return frame(kettle_R(tilt), (L.KETTLE_XY[0], L.KETTLE_XY[1], L.KETTLE_CZ))


def cup_on_table(z=None, dx=0.0, dy=0.0):
    zz = L.CUP_CZ if z is None else z
    return frame(K.rot_z(L.CUP_YAW),
                 (L.CUP_XY[0] + dx, L.CUP_XY[1] + dy, zz))


def cup_in_base(bx, by, byaw, r, z):
    """杯子在机器人 base 系 (r, 0, z) 处, 杯柄朝 base +x(即朝人)"""
    R = K.rot_z(byaw)
    t = (bx + r * math.cos(byaw), by + r * math.sin(byaw), z)
    return frame(R, t)


# ---------------------------------------------------------------- 抓取位形
def kettle_grasp_palm(T_kettle):
    """右手抓壶: 目标点为把手上的一点(把手中心)"""
    gp = T_kettle[:3, 3] + T_kettle[:3, :3] @ np.array(L.KETTLE_GRIP_LOCAL)
    return K.palm_target(T_kettle[:3, :3], gp,
                         KETTLE_APPROACH, KETTLE_CURL,
                         K.OBJ_IN_PALM_BAR)


def cup_grasp_palm(T_cup, side='L'):
    curl = CUP_CURL_LEFT if side == 'L' else CUP_CURL_RIGHT
    return K.palm_target(T_cup[:3, :3], T_cup[:3, 3],
                         CUP_APPROACH, curl, K.OBJ_IN_PALM_CUP)


# 双手捧杯: 单手握的偏置(横向 0.036)是给"一只手把杯子攥住"用的, 两只手同时
# 摆上去会**叠在一起**(掌心中心只差 0.072m, 而一只手掌就有 0.08m 宽)。捧杯
# 只要求"两只手分居杯子两侧、深度一致", 所以横向扩到 PALM_HALF_GAP=0.105 的
# 一半多一点 —— 掌心中心离杯轴 0.075m, 两手间距 0.15m, 正好把 0.08m 的杯子
# 夹在中间而不互相穿透。
OBJ_IN_PALM_CUP2 = (0.0, 0.075, K.OBJ_IN_PALM_CUP[2])


def cup_cradle_palm(T_cup, side='L'):
    """双手捧杯时一侧掌心的目标位姿(两手都从机器人一侧伸来, 分居杯子两侧)。"""
    curl = CUP_CURL_LEFT if side == 'L' else CUP_CURL_RIGHT
    return K.palm_target(T_cup[:3, :3], T_cup[:3, 3],
                         CUP_APPROACH, curl, OBJ_IN_PALM_CUP2)


# ---------------------------------------------------------------- 倒水
def cup_for_pour():
    """倒水时杯子被左手举到的位置(竖直, 杯柄仍朝 +x)"""
    return cup_on_table(z=CUP_LIFT_Z, dx=CUP_LIFT_DX, dy=CUP_LIFT_DY)


def kettle_for_pour(T_cup, tilt=KETTLE_TILT, gap=JET_GAP):
    """由杯口位置反解壶体位姿: 壶嘴尖落在杯口正上方 gap 处"""
    mouth = T_cup[:3, 3] + T_cup[:3, :3] @ np.array([0.0, 0.0, L.CUP_H / 2.0])
    tip_tgt = mouth + np.array([0.0, 0.0, gap])
    R = kettle_R(tilt)
    origin = tip_tgt - R @ np.array(L.KETTLE_SPOUT_TIP_LOCAL)
    return frame(R, origin), tuple(tip_tgt)


def water_level_frac(t):
    """倒水进度 t∈[0,1] → 杯内水位关节目标(prismatic, 0..CUP_WATER_MAX)"""
    t = max(0.0, min(1.0, t))
    return L.CUP_WATER_MAX * t


# ---------------------------------------------------------------- 递/接
def present_cup(bx, by, byaw, r=None, z=None):
    rr = L.CUP_PRESENT_R if r is None else r
    zz = L.CUP_PRESENT_Z if z is None else z
    return cup_in_base(bx, by, byaw, rr, zz)


def carry_cup(bx, by, byaw):
    return cup_in_base(bx, by, byaw, L.CUP_CARRY_R, L.CUP_CARRY_Z)


def person_hand(bx, by, byaw):
    """人的捧杯点(世界位姿)。人站在 (bx,by), 面朝 byaw, 双手在体前 0.30m"""
    t = (bx + L.PERSON_HAND_FWD * math.cos(byaw),
         by + L.PERSON_HAND_FWD * math.sin(byaw),
         L.PERSON_HAND_Z)
    return frame(K.rot_z(byaw), t)


def person_spot_pose():
    """人接水时应该站的位置与朝向(机器人面向人)"""
    px, py = L.PERSON_SPOT
    return (px, py, L.PERSON_SPOT_YAW + math.pi)   # 人面朝机器人


# ---------------------------------------------------------------- 自检
def cases():
    """返回 [(side, 标签, 世界位形 4x4, 底盘位姿(x,y,yaw)), ...]

    drink_task 执行序列与 tools/check_ik.py 的离线体检共用这一份清单,
    保证"校验过的位形"就是"会执行的位形"。
    """
    bx, by, _byaw = L.SERVE_POSE
    base_serve = (bx, by, 0.0)
    base_hand = (bx, by, L.HANDOVER_YAW)
    out = []
    # 1) 桌面抓壶 / 放壶 / 抓杯
    out.append(('R', '抓壶(桌面)', kettle_grasp_palm(kettle_on_table()),
                base_serve))
    out.append(('L', '抓杯(桌面)', cup_grasp_palm(cup_on_table(), 'L'),
                base_serve))
    # 2) 倒水(左手举杯, 右手持壶)
    Tcp = cup_for_pour()
    Tkp, _tip = kettle_for_pour(Tcp)
    out.append(('L', '举杯(倒水)', cup_grasp_palm(Tcp, 'L'), base_serve))
    out.append(('R', '持壶(倒水)', kettle_grasp_palm(Tkp), base_serve))
    out.append(('R', '放壶(桌面)', kettle_grasp_palm(kettle_on_table()),
                base_serve))
    # 3) 双手捧杯(胸前) / 递出 —— 用**捧杯**偏置, 单手握的偏置会让两只手叠住
    for tag, Tcup in (('双手捧杯(胸前)', carry_cup(bx, by, 0.0)),
                      ('双手递杯(前伸)', present_cup(bx, by, 0.0))):
        out.append(('L', tag, cup_cradle_palm(Tcup, 'L'), base_serve))
        out.append(('R', tag, cup_cradle_palm(Tcup, 'R'), base_serve))
    # 4) 转身面向人后的递杯 / 接回
    Tph = present_cup(bx, by, L.HANDOVER_YAW)
    out.append(('L', '递杯给人(L)', cup_cradle_palm(Tph, 'L'), base_hand))
    out.append(('R', '递杯给人(R)', cup_cradle_palm(Tph, 'R'), base_hand))
    return out


def report():
    """把关键位形过一遍逆解, 返回 [(side, tag, 位置误差, 姿态误差, 余量, 解)]"""
    rows = []
    seed = {'L': K.Q_REST.copy(), 'R': K.Q_REST.copy()}
    for side, tag, T, base in cases():
        q, _e, how = K.ik_world(side, T, base, seed[side])
        err = K.pose_err(K.fk_world(side, q, base), T)
        ep = float(np.linalg.norm(err[:3]))
        er = float(np.linalg.norm(err[3:]))
        marg = float(np.min(np.minimum(q - K.LIM_LO, K.LIM_HI - q)))
        if ep < 0.01:
            seed[side] = q
        rows.append((side, tag, ep, er, marg, how, q))
    return rows
