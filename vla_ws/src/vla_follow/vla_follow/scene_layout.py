#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""scene_layout.py —— "喝水任务"场景坐标的唯一真源 (P84)

本模块是**桌子 / 水壶 / 水杯 / 机器人服务位 / 人的接水位** 的唯一定义处。
SDF(由 tools/gen_drink_scene.py 生成)、bridge 配置、drink_task 运行时
全部从这里取值 —— 改布局只改这一个文件, 不要手改 work_area.sdf 的生成块。

坐标约定(与 ROS/gz 一致):
    x 前 / y 左 / z 上; 机器人本体坐标系 base_link 原点在两驱动轮轴中点
    正下方的地面上(z=0), 车头朝 +x。

几何自洽关系(改数前务必保持):
    |PERSON_SPOT - SERVE_POSE| == CUP_PRESENT_R + PERSON_HAND_FWD
    即"机器人把杯子前伸 0.62m"正好落在"人前伸 0.30m 的手上"。
"""
import math

# ---------------------------------------------------------------- 桌子
TABLE_XY = (2.60, 0.35)       # 桌面中心(世界系)
TABLE_YAW = 0.0
TABLE_W = 0.94                # x 向长
TABLE_D = 0.66                # y 向宽
TABLE_TOP_Z = 0.72            # 桌面上表面高度
TABLE_PLATE_T = 0.05          # 台面厚度
TABLE_LEG = 0.05              # 桌腿截面
# 障碍表用的外接半径(follower / person_walk / gen_map 三处共用)
TABLE_R = 0.62

# ---------------------------------------------------------------- 桌面物件
# 水壶: 壶身中心(世界), 壶嘴朝 +y(即 yaw=+90°), 把手朝 -y(靠机器人一侧)
KETTLE_XY = (2.30, 0.20)
KETTLE_YAW = math.pi / 2.0
KETTLE_BODY_R = 0.068         # 壶身半径
KETTLE_BODY_H = 0.160         # 壶身高
KETTLE_BASE_Z = TABLE_TOP_Z   # 壶底 z
# +0.003 出生余隙: 若壶底/杯底与桌面**恰好贴合**, 求交判据会判成穿透,
# 接触冲量把物件弹飞(实测飞出去 1.7~2.5m)。抬高 3mm 让它自由落体坐稳。
SPAWN_CLEARANCE = 0.003
KETTLE_CZ = KETTLE_BASE_Z + KETTLE_BODY_H / 2.0 + SPAWN_CLEARANCE
# 把手(竖直条)在壶身坐标系的位置: x=-0.085 处, z 相对壶身中心 +0.04
KETTLE_GRIP_LOCAL = (-0.085, 0.0, 0.04)
KETTLE_GRIP_BAR_R = 0.012
# 壶嘴: 从壶身上部 +x 伸出, 局部坐标(壶嘴中心线)
KETTLE_SPOUT_LOCAL = (0.062, 0.0, 0.055)   # 起点
KETTLE_SPOUT_LEN = 0.070
KETTLE_SPOUT_TIP_LOCAL = (0.062 + KETTLE_SPOUT_LEN, 0.0, 0.055 + 0.022)

# 水杯: 杯身中心(世界), 杯柄朝 +x(背对机器人, 不挡左手)
CUP_XY = (2.30, 0.44)
CUP_YAW = 0.0
CUP_R = 0.040
CUP_H = 0.100
CUP_BASE_Z = TABLE_TOP_Z
CUP_CZ = CUP_BASE_Z + CUP_H / 2.0 + SPAWN_CLEARANCE      # 0.773
CUP_WATER_MAX = 0.024                      # 水位最大升起量(prismatic 行程)
# 杯柄在杯身坐标系的朝向(+x)
CUP_HANDLE_DIR = (1.0, 0.0, 0.0)

# ---------------------------------------------------------------- 机器人
# 服务位: 站在桌子正面, 车头朝 +x
SERVE_POSE = (1.75, 0.35, 0.0)
# 底盘中心到桌面中心距离 = 0.85; 底盘前缘(0.17)到桌沿(2.13)间隙 0.21m
SHOULDER_Z = 0.95             # 肩关节高度
SHOULDER_Y = 0.19             # 肩关节左右偏移
SHOULDER_X = 0.02

# 递杯: 杯子在 base_link 下的位置
CUP_PRESENT_R = 0.62          # 前伸距离(名义值, 人站在 PERSON_SPOT 时正好)
CUP_PRESENT_Z = 1.00          # 递出高度
CUP_CARRY_R = 0.50            # 收在胸前的距离
CUP_CARRY_Z = 1.00
PALM_HALF_GAP = 0.105         # 两手掌到杯心的横向间距
# P94: 前伸上限(人站偏时用)。0.62 是"人站准"的解, 不是臂的极限 ——
# `tests/diag/diag_p94_reach.py` 扫出来: 递杯方向偏 ±30°、前伸 0.70m 时
# 两手位置残差 0.00mm、限位余量还有 39°; 到 0.75m 就顶到臂长(残差 9.8mm、
# 余量掉到 8.5°)。所以安全上限取 0.70, 留 50mm 不吃满。
CUP_PRESENT_R_MAX = 0.70
# P94: 底盘补位。臂伸满仍差超过 35mm 时, 机器人朝人挪一段再递 —— 否则
# `hand_to` 交接时杯子会从"离手 70mm 处"瞬移过去。
PRESENT_APPROACH_GAP = 0.035   # 差多少才值得挪(小于它臂的余量能吸收)
PRESENT_APPROACH_MAX = 0.40    # 一次最多挪多少 —— 真正的上限由
#                               # `table_clearance()` 现算: 贴到桌面
#                               # PRESENT_TABLE_PAD 之内就不再往前
PRESENT_APPROACH_CLEAR = 0.69  # 补位目标: 把车→人手压到这个值以内
PRESENT_TABLE_PAD = 0.05       # 补位时与桌面矩形保持的间隙。别再往下调:
#                               # pad 0.03 允许挪 0.38m, 但机器人贴到桌沿
#                               # 3cm 之后, 递杯/放回的臂部动作会扫到桌面,
#                               # 实测把杯子撞飞 4.13m。
CHASSIS_R = 0.17               # 底盘外接圆半径(服务位前缘到桌沿 0.21m 即此值)

# ---- P102: 去倒水时的"入场距离" ----
# 用户 2026-10-28: "机器人去倒水时, 在离桌子的距离**略大于**机械臂+灵巧手
# 整体长度的位置, 先把机械臂往上抬、以肘为支点弯曲, 把前臂和双手抬到与水壶/
# 水杯同等高度, 然后再靠近桌子倒水。"
# 臂+手全长 = arm_kin.ARM_REACH = 0.715m(肩→指尖), 这里取车壳到桌沿 0.80m
# —— 多留 85mm, 保证"整条臂放下来"的圆弧也够不到桌子。
POUR_STANDOFF_GAP = 0.80
POUR_STANDOFF_X = (TABLE_XY[0] - TABLE_W / 2.0) - CHASSIS_R - POUR_STANDOFF_GAP

# ---------------------------------------------------------------- 人
PERSON_HAND_FWD = 0.30        # 人前伸取杯时, 手到人体中心的水平距离
PERSON_HAND_Z = 1.00          # 手的高度(世界)
# `person_arms` 这个 link 的**原点在肩关节**, 不是两手之间 —— 世界位姿
# 探出来是 (0.10, 0, 1.02)(= skirt 原点 0.28 + link pose 里的 0.74)。
# 所以凡是要"钉到人的手上"的地方(hand_to / rela_to), 都必须再叠一个
# link 系内的偏置: 手点相对 link 原点 = (0.30-0.10, 0, 1.00-1.02)。
# P89 [3] 的 1359.6mm 就是漏了这一步 + 漏乘模型位姿两件事叠加出来的。
PERSON_ARM_ROOT = (0.10, 0.0, 1.02)                  # person_arms 原点(世界)
PERSON_HAND_IN_ARM = (PERSON_HAND_FWD - PERSON_ARM_ROOT[0], 0.0,
                      PERSON_HAND_Z - PERSON_ARM_ROOT[2])   # (0.20, 0, -0.02)

# ---- P96: 人的两只手各自的外展关节 ----
# 手的 link 原点在肩点(person_arms 系 ±0.155), 手心在它前方 (0.20,∓0.070,-0.02),
# 绕肩点的竖直轴外展 θ 时手心在 person_arms 系的 y 为
#     sgn*(0.155 + 0.20*sinθ - 0.070*cosθ)
# 见下: 0.035rad 时 |y|=0.092 = 杯半径 0.040 + 手掌半径 0.052, 恰好贴住杯壁;
# 0.42rad 时 |y|=0.173, 两手内侧面相距 0.242m, 杯子(直径 0.080)两侧各留 81mm
# —— 这个开口是**让杯子能进得来**, 也是"人伸手去接"这件事能被看见的幅度。
# 两条关节轴都写 (0,0,1), 由上层对右侧发**反号**值(§6.4 #60)。
PERSON_HAND_CLOSE = 0.035     # 合拢(捧住杯子)  两手心间距 184mm
PERSON_HAND_OPEN = 0.42       # 张开(迎杯/交还) 两手心间距 345mm
PERSON_ARM_LINKS = ('person_arm_l', 'person_arm_r')
# 手掌球半径 / 杯半径: 判据要按"贴住 + 不穿模"算, 见 test_p96.py
PERSON_HAND_R = 0.052
CUP_GRIP_R = CUP_R + PERSON_HAND_R        # 0.092: 手心到杯轴正好相切
# 肩点 / 手点在"人体系"的左右偏移。这两个数是 SDF 里两条手臂 link 的**摆放
# 依据**(tools/gen_drink_scene.py 从这里取, 不是各写一份), 也是上层算"手心
# 世界位姿"要用到的 link 系内偏置 —— 两边共用一个数, 才不会各改各的。
PERSON_SHOULDER_Y = 0.155     # 肩点(手臂 link 原点)左右偏移
PERSON_HAND_Y = 0.085         # 手点左右偏移(向中线收拢, 合拢位形下)
_dhy = PERSON_SHOULDER_Y - PERSON_HAND_Y      # 0.070
_dz = PERSON_HAND_Z - PERSON_ARM_ROOT[2]      # -0.02
_dfx = PERSON_HAND_FWD - PERSON_ARM_ROOT[0]   # 0.20
# 手心在各自手臂 link 系里的位置(= 手臂 link 原点在肩点, 手心在它前内侧)
PERSON_HAND_IN_ARML = (_dfx, -_dhy, _dz)
PERSON_HAND_IN_ARMR = (_dfx, +_dhy, _dz)
# 接水位: 由服务位出发, 距离 = CUP_PRESENT_R + PERSON_HAND_FWD, 方位 63°
PERSON_SPOT_R = CUP_PRESENT_R + PERSON_HAND_FWD      # 0.92
PERSON_SPOT_YAW = math.radians(62.9)
PERSON_SPOT = (SERVE_POSE[0] + PERSON_SPOT_R * math.cos(PERSON_SPOT_YAW),
               SERVE_POSE[1] + PERSON_SPOT_R * math.sin(PERSON_SPOT_YAW))
HANDOVER_YAW = PERSON_SPOT_YAW        # 机器人面向人的朝向

# ---- P93: 喝完水之后各自的"撤离位"(必须都在桌子 / 场地之外) ----
# 机器人: 从服务位沿 -x(背离桌面)退 1.15m。服务位车头朝 +x 正对桌子,
# 所以撤离时**倒车**即可, 不用先掉头; 退到 (0.60,0.35) 后底盘前缘距
# 桌沿 1.5m, 交还给 follower 时它一接手就是干净位形(不在桌子的 1.13m
# 膨胀圈里, 见 §6.4 #47), 否则 follower 的绕障会先跟桌子较劲。
SERVE_RETREAT_XY = (SERVE_POSE[0] - 1.15, SERVE_POSE[1])
SERVE_RETREAT_YAW = 0.0               # 保持朝桌, 倒车出去
# 人: 沿"桌心 → 接水位"方向再往外 1.75m。方向 (-0.43,0.82)/0.926,
# 终点 (1.36,2.72) —— 距桌心 2.6m(出了桌子 0.62+0.2 的避障圈)、
# 距机器人撤离位 2.5m。2.5m 是刻意留的: 太近(贴着桌)人走两步就出画,
# 太远(场地对角)follower 要 Nav2 搜很久才追上, 端到端等不起。
_dx, _dy = (PERSON_SPOT[0] - TABLE_XY[0], PERSON_SPOT[1] - TABLE_XY[1])
_dn = math.hypot(_dx, _dy)
PERSON_LEAVE_SPOT = (PERSON_SPOT[0] + 1.75 * _dx / _dn,
                     PERSON_SPOT[1] + 1.75 * _dy / _dn)
# 交还跟随后的验收带宽: 沿用 P84 的跟随目标距离(1.0m ± 0.35)
FOLLOW_BAND_M = 1.00
FOLLOW_BAND_TOL_M = 0.35


def table_clearance(x, y, pad=0.0):
    """底盘中心 (x, y) 到**桌面矩形**的间隙(m), 已含底盘外接圆半径。

    P94: 判"还能不能再往前挪"时, 用桌子的外接圆(0.574)会过于保守 —— 机器
    人在服务位沿 62.9° 挪 0.2m 后离桌心只有 0.79m, 外接圆判据说撞了
    (0.574+0.17=0.744), 可它其实在桌子的**侧面**, 离最近的桌沿还有 0.29m。
    递杯补位就靠这个差值拿到可挪空间(实测: 外接圆只允许挪 0.147m, 矩形
    判据允许 0.37m, 而人要站偏 0.15m 时需要挪 0.194m)。
    """
    dx = max(abs(x - TABLE_XY[0]) - TABLE_W / 2.0, 0.0)
    dy = max(abs(y - TABLE_XY[1]) - TABLE_D / 2.0, 0.0)
    return math.hypot(dx, dy) - (CHASSIS_R + pad)


def rpy_mat(yaw, pitch=0.0, roll=0.0):
    """SDF 顺序 rpy = 绕固定轴 X→Y→Z 的内旋: R = Rz(yaw)Ry(pitch)Rx(roll)"""
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def person_hand_pose(px, py, pz, yaw):
    """人的"捧杯点"世界位姿(人坐标系: +x 朝前)。

    人模型是单刚体, 双手平举在胸前 —— 手点固定在 (0.30, 0, 1.00) 相对
    人体中心(世界 z 由 pz 给出, 人模型原点在地面)。
    """
    hx = px + PERSON_HAND_FWD * math.cos(yaw)
    hy = py + PERSON_HAND_FWD * math.sin(yaw)
    return (hx, hy, PERSON_HAND_Z), yaw


def world_to_base(x, y, z, bx, by, byaw):
    """世界点 → 机器人 base_link 坐标"""
    dx, dy = x - bx, y - by
    c, s = math.cos(-byaw), math.sin(-byaw)
    return (dx * c - dy * s, dx * s + dy * c, z)


def base_to_world(x, y, z, bx, by, byaw):
    """机器人 base_link 坐标 → 世界点"""
    c, s = math.cos(byaw), math.sin(byaw)
    return (bx + x * c - y * s, by + x * s + y * c, z)


# 观察相机(验证/演示用)
# P84: 观察相机必须同时框住"机器人(原点) + 桌子(2.6,0.35) + 人的接水位",
# 否则双手机械臂到底有没有装对根本看不出来(第一版只拍了桌子, 机器人整台
# 都在画面外)。
# P87: 原来的 (0,-2.6,1.3) 看 (0,0,0.7) —— 那是**世界原点**, 而作业区已经
# 移到 x≈2.3 的桌边, 于是画面里只有人和机器人, 壶/杯/水柱全在画框外
# (p87_pouring.png 里只能看到桌子的一角)。改成架在桌子 -y 侧、贴着杯子上方
# 1.1m 处:
#   * 机位在桌外(y=-0.30 < 桌面下沿 0.02), 视线从桌沿上方越过, 不遮;
#   * 画幅 1.7m 宽, 左边带机器人(1.75)、右边带桌子(2.6), 正中是壶/杯/水柱;
#   * 杯柄朝 +x(CUP_HANDLE_DIR), 从 -y 侧看是"柄在右手边", P88/P89 验
#     "柄朝人"时直接看这张图;
#   * 太远就没用了: 2.3m 处杯径只有 14px(第一版), 1.14m 处 30px。
# P88 再改一次: 上面那个机位在桌子 -y 侧, 机器人**转身面向人之后**杯子就跑到
# 右臂后面去了(实测 p88_present.png 里杯子被手挡住)。改成站在**人的斜后方**:
#   * 方位 36°(机器人转身后朝 63°), 于是转身前后都看的是机器人正脸;
#   * 不在"相机→机器人"的连线上, 人不会挡住机器人;
#   * 视线越过桌沿(过 y=0.68 时 z=1.16 > 台面 0.72), 桌面不遮挡;
#   * 递杯(0.62m)与人身前 0.30m 的手都在画幅内, P89 的交接能直接看。
OBS_CAM_POS = (3.05, 1.30, 1.45)
OBS_CAM_LOOK = (2.15, 0.48, 1.02)
# 水平视场角(rad)。1.30 时画幅 1.7m, 杯径只有 30px; 收到 1.15 后杯径 ~30px@1.29m,
# 同时还能框住桌面(0.72)、壶顶(0.92)与捧杯高度(1.00)。
OBS_CAM_FOV = 1.15
