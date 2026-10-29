#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""follower.py —— 语音指令驱动的跟随控制器 (方案 v4)

控制方案:
1. 位姿: Gazebo 真值(dynamic_pose/info, gz-transport 直连)
2. 朝向: "先对正、再移动" 两段式
   - 目标偏离 front 相机垂直中线超过 aim_band_deg(默认 15°) → ALIGN:
     原地纯旋转, 比例控制 + 软死区, 角速度上限 recenter_rot_speed;
     目标在左/右/后相机时同样先转回前向视野。
   - 目标进入 ±15° → TRACK: 距离 PID 决定前进/后退/保持,
     同时小幅修正航向把目标压在 ±15° 内。
3. 速度: 距离环 PID + 人速前馈, 人停→机器人停; 近人限速、过近后退
4. 绕障(真值地图): 仅在对正之后生效, 取障碍侧方切点绕行
5. 目标丢失: 按 2Hz 快照历史外推去向 → 前往最后出现点 → 原地扫描
注: v/PID 全部参数来自 vla_demo/vision_config.json。
"""
import json
import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from nav2_msgs.action import NavigateToPose

import gz.transport13 as gz_transport
from gz.msgs10.pose_v_pb2 import Pose_V as GzPose_V

# ---- 跟踪配置(P15, 与视觉节点共用同一配置文件) ----
CONFIG_PATH = os.path.join(
    os.environ.get('VLA_HOME', '/home/earl/workspace/VLA'),
    'vla_demo', 'vision_config.json')

DEFAULT_CFG = {
    # P79: 跟随距离区间由 1.0±0.1 放宽到 **0.5~1.5m**(tol=0.5)。
    # 放宽后有两处必须同步改, 否则"宽区间"会退化成"不停后退":
    #   ① `guard_d`/`close_back`/ALIGN 过近门限原来都按 ±0.1 的窄带写死,
    #      带宽一放大, 0.55~0.85m 这段**合法**区域仍在触发后退;
    #   ② PID 的 err_d 绝对值在带边缘翻倍(0.4 vs 0.2), 同一 kp 会输出
    #      饱和的 ±0.45 → dist 一进 0.55 就一脚油门倒车。所以配
    #      `inband_fb` 软死区(带内按 0.4 倍衰减, 出带连续恢复到原增益)。
    'track_dist_m': 1.0, 'track_dist_tol_m': 0.50,
    # P79: 接近减速区 —— 人距 < slow_zone_m 起按幂曲线收油门, 到
    # slow_floor_m(区间下限)收到 0。指数 0.6 <1 让"2.0→1.5m"这段降得
    # 平缓(还跟得上人), "1.0→0.5m"这段急降(不会撞上)。
    # 之前的版本只有 v_lat=(dist-guard_d)/0.8 一条隐式限速, 2.0m 处
    # 完全不起作用(dist=2.0 时 v_lat≈1.8), 等于"看见人还在全速冲"。
    'slow_zone_m': 2.00,            # 开始减速的人距
    'slow_floor_m': 0.50,           # 减速到 0 的人距(区间下限)
    'slow_exp': 0.45,               # 减速曲线指数(1=线性; 0.6 实测把 1.3~1.5m
                                    #   的速度压到 0.49~0.56, 人稍快就跟不上,
                                    #   距离被拉到 1.9m → 丢目标。0.45 让 1.5m 处
                                    #   仍是 0.72*0.83=0.60, 只在 <1.0m 段急降)
    'inband_fb': 0.40,              # 带内反馈增益衰减系数(1=不衰减)
    'recenter_rot_speed': 1.5,      # 对正最大角速度 rad/s
    'approach_speed': 0.62,         # P40: 0.52→0.62, 给人突然加速留追赶余量
    # P40: 1.62 → 1.0。P36 那个 1.62 是错的 —— 日志里 93 组
    # `指令v / 实际v` 配对的比值众数恰好是 1.6, 过原点拟合斜率 1.066,
    # 说明差速插件本来就 1:1 跟踪速度指令; 当初用 /robot/odom 标出来的
    # "×0.62" 是 odom 计量造成的假象。多放大 62% 的结果是机器人冲过头、
    # 再被安全层拉回, 距离在 0.41~2.78m 之间大幅摆动。
    'cmd_gain': 1.00,
    # P40: 距离环前馈使能 —— 带内靠前馈与人同速, 误差只修正残差
    'vel_ff_gain': 0.90,            # 人速前馈系数(0=关)
    'vel_ff_max': 0.72,             # 前馈限幅 m/s
    'use_waypoint': False,          # P39: 航点绕障在临界角上会左右翻转死锁
    'use_tangent': False,
    # P46: 锁存式绕障。P39/P43b 把两种绕障都关了, 结果目标走到障碍后
    # 就完全失去横向机动 —— 车头一朝向目标就被安全层急停, 原地打转。
    'use_avoid': True,
    'avoid_deg': 72.0,      # 绕行时车头相对"目标方向"的固定偏转角
    'avoid_min_v': 0.14,    # 最小推进速度(不允许为 0, 否则又变原地转)
    'avoid_trig_s': 0.45,   # 前方持续受阻多久才判定需要绕行
    'avoid_max_s': 9.0,     # 单次绕行最长时间(超时换边重试)
    'avoid_exit_v': 0.90,   # 前方余量超过 slow_d*该系数 视为畅通           # P43b: 切线绕障(实测会原地打转, 默认关)
    # P77: 0.32 → 0.357。用 e1/e2/t18_p76 三份评测的 recs 直接量
    # `vis_d - 真值d` 的中位偏差 = **-0.356 / -0.358 / -0.357**(三份几乎一致),
    # 且按距离分档(0.6~0.9 / 0.9~1.1 / 1.1~1.4)偏差恒为 -0.345~-0.361、
    # 而比值从 0.56 变到 0.70 ⇒ 是**固定偏移**不是比例误差。
    # 原来补 0.32 差了 3.7cm ⇒ 控制器以为已经到 1.00 时真值其实是 1.037,
    # 这就是"稳态落在 1.04m、in_band 只有 35%"的全部原因。
    'vis_bias_m': 0.357,            # P77: 0.32→0.357(见上), 感知距离≈真值
    'pid_kp_v': 1.90, 'pid_ki_v': 0.06, 'pid_kd_v': 0.04,   # P43
    'pid_kp_w': 2.2, 'pid_ki_w': 0.40, 'pid_kd_w': 0.30,
    # P20: 对正(align)参数
    'aim_band_deg': 15.0,           # 目标允许偏离 front 中线的角度带
    # P77 §17-4: 对正进/退的**时间滞回**。只看角度阈值时 brg 边界噪声会让
    # ALIGN↔TRACK 每 0.5~1s 翻一次(受控场景 mode 切换 56 次/75s), 每次切换
    # w 都要 0↔±0.35, 这是换向率的主要来源。
    'align_enter_s': 0.25,          # 越界需持续此时长才进对正
    'align_exit_s': 0.35,           # 回带需持续此时长才退出对正(比进入更黏)
    # P77 §17-2: 近距离方位可信度。dist 小到人体占满 front 画幅时, 检测框只剩
    # 残缺一块、主相机在 front/left/right 之间乱切, brg 会在一帧内跳 ±130°
    # (P76 实测 -42°/+133°/+25°/-136°)。这在物理上不可能: 人站在 0.6m 处
    # 横向偏 35° 意味着 0.1s 内横移 0.36m(3.6 m/s)。所以近距离两个动作:
    #   ① 滤波系数调小(0.55 → 0.35), ② 对每帧方位变化量按物理上限夹住。
    # 远距离行为完全不变(_close=False 时走原路径)。
    'brg_close_m': 0.90,            # 低于此距离: 关掉"跨相机切换"豁免, 大跳先冻结
    'brg_af_close': 0.45,           # 近距离 EWMA 系数(默认 0.55)
    'recenter_kp': 2.0,             # 对正比例增益 (rad/s per rad)
    'turn_min_w': 0.35,             # 对正最小角速度(克服静摩擦)
    # P77 §17-3/§17-4: 转向(w)仲裁 + 双向滞回。
    # P76 受控场景实测 w 换向 58 次 / 75s ≈ 0.77 次/s。排查发现这些换向大多
    # 不是"目标真的换了边", 而是 SCAN 与 ALIGN **各算各的 w、交替接管**后
    # 互相推翻: 两边虽然各自有 `_scan_dir` / `_align_dir` 锁存, 但谁也管不了
    # 对方, 于是在 SCAN↔ALIGN 边界上每 1~3s 翻一次, 机器人原地打摆。
    # 现在所有会决定转向的模块都必须走 `_arb_w()` 这一个出口。
    'w_hold_s': 0.45,               # 同 owner 内反向请求需持续的时长
    'w_owner_hold_s': 0.50,         # 同级/降级交接 owner 前双方各自的最短保持
    'w_flip_urgent': 0.90,          # 幅值超过此值允许立即换向(真急转)
    'w_arb_on': 1,                  # 0 = 关掉仲裁/滞回(只留直通), 用于 A/B
    'hold_kp': 2.2,                 # TRACK 中保持航向的增益
    # P25/P26: 全向安全层(避障最高优先级)
    #   车体 0.30x0.20 → 外接半径 0.18。停车距离必须同时覆盖
    #   ① 车体半径 ② 测距滞后(v*age) ③ 制动距离。
    #   旧值 0.42 只够 ① —— 实测 0.5m/s + 5Hz 相机 + 0.45s 陈旧,
    #   等于"看到 0.42m 时实际已在 0.20m", 车体(0.18)已经贴上去了。
    'lidar_stop_m': 0.66,           # 到障碍表面 <= 此值: 立即停车
    'lidar_slow_m': 1.60,           # 到障碍表面 <  此值: 按距离限速
    'lidar_side_m': 0.60,           # 侧向安全距离(防侧擦)
    'lidar_max_age_s': 0.22,        # 测距数据最大陈旧度(相机 10Hz → 约 3 帧)
    'lidar_tau_s': 0.60,            # 限速时假设的制动反应时间
    'escape_s': 0.90,               # 卡死后脱困动作持续时长
    'esc_hold_s': 0.45,             # P27 绕障转向方向最短保持时间
    # P77 §17-3: 急停的释放滞回。进: d_head <= stop_d; 出: d_head > k*stop_d。
    # 只有一个阈值时 d_head 在 0.66 上下浮动会让急停每帧吸/放一次, 而急停
    # 会把 w 强行掰到 1.15*_esc_dir(与对正反号) —— 实测 1.4s 一个乒乓,
    # 是"障碍旁来回摆"的直接来源。
    'estop_release_k': 1.25,        # 释放需前方净空 > 1.25*stop_d (0.825m)
    'estop_max_s': 2.50,            # 最长吸住时间, 兜底防"永远等不到净空"
    # P77 §17-4: 人已经看得很清楚(≤此距离)但前进被挡时**驻留对正**,
    # 不做"哪边空往哪绕"的脱困转向 —— 那会把车头从人身上掰开, 下一帧对正
    # 又掰回来, 形成 1.4s 周期的原地打摆(实测后 40s 净位移 0.1~0.3m、
    # 偏航 churn 1500~1800°)。
    'hold_blocked_m': 2.50,         # 人距 ≤ 2.5m 且前方受阻 → 驻留
    # P77 §17-4 第三版: 驻留必须"先等一会儿"。瞬时受阻(障碍正好扫过)该
    # 绕就绕; 只有**持续**受阻才说明"路就在人那边、但被挡死了", 这时绕
    # 也绕不过去, 原地对正才是正解。
    # 实测依据: 一受阻就驻留(t22), 回放评测 in_band 37.4→34.5、far 33.2→35.6
    # —— 把本来 1~2s 就能绕过去的情形也冻住了; 反过来"驻留 hold_max_s 后
    # 强制定向绕行"(t23) 更糟: min_d 0.649→0.451、居中度 99.3→96.5、丢人 1 次
    # (车头掰开 78° 后照样往前顶)。所以这里只加"持续性"门槛, 不加绕行窗口。
    'hold_after_s': 0.80,           # 连续受阻超过此时长才转入驻留
    # P78: 驻留/急停刚解除的那 0.3s 里, 目标线速度从 0 线性升到全速 ——
    # 否则 v_t 在一帧内从 0 跳到 approach_speed, 期间机器人正好停在人
    # 0.8~1.2m 处, 一个梯形爬升的冲量就够把 min_d 压到 0.59(低于
    # lidar_stop_m 0.66)。居中度多掉的那 1.6pt 也全集中在这 0.3~0.5s。
    # P79: 让距起点跟着**区间下限**走, 不再用固定 0.15。原来是
    # `dist < target_d-0.15=0.85` 就后退 —— 在 0.5~1.5 的宽区间下,
    # 0.55~0.85m 全是合法区域, 仍按 0.85 后退就是用户点名要消灭的
    # "频繁后退"。改成 `close_back_m = target_d - slow_floor_m = 0.50`,
    # 也就是只有真的跌破区间下限才让距。
    'close_back_m': 0.50,           # 让距生效区间起点(dist < target_d-此值)
    'close_back_v': 0.10,           # 让距后退速度基数(刚越线时)
    'close_back_k': 0.80,           # 越线每多 1m 追加的速度(上限 0.28)
    'close_back_on': 1,             # 0 = 关掉让距(A/B)。单次差异在噪声内
                                    #     (同配置重跑 in_band ±6.3pt), 按
                                    #     "不让人贴到 0.6m"的机理默认开。
    # P80: 盲走禁令 —— 视觉连续这么久无检出, 就禁止前进(允许后退/转/停)。
    # 归因(t32 76.0~76.4s): 人在 0.62→0.41m 时检测丢失, ALIGN 的
    # "边转边走"仍按**陈旧**的 last_known dist 判"还没到目标距离", 继续以
    # 0.22m/s 前推 1.0s, 一路顶到 0.41m。人裙摆 0.26 + 车体 0.18 = 0.44m
    # 即物理接触, 而深度相机此时报"前 99"(0.4m 内是近距盲区) —— 没有任何
    # 传感器能兜底。这是"撞上人"的唯一真因, 其它限速/让距都只是减伤。
    'blind_v_s': 0.30,              # 0=关闭(A/B 对照)
    # P80: 主动让距。检测器在真值 0.70m 就看不见人(人体占满画幅),
    # 所以"停在 0.7~0.85m 等人自己走完"是行不通的 —— 会直接走进接触距离。
    # 让距速度 = gain*人的逼近速度 + k*(目标让距-当前距离), 人停就不退。
    'back_hold_m': 0.85,
    'back_hold_gain': 1.00,
    'back_hold_k': 0.70,
    'back_hold_vmax': 0.45,
    'back_hold_vmin': 0.06,
    'back_hold_on': 1,
    'resume_soft_s': 0.00,          # 前进软启动(0=关; t25 实测会拖慢恢复,
                                   #  反而让人走进 0.4m 内丢目标, 默认关)
    'hold_max_s': 2.00,             # 驻留上限(仅 detour_on=1 时生效)
    'detour_s': 1.30,               # 定向绕行窗口时长(A/B 用, 默认关)
    'detour_dir_hyst': 0.35,        # 绕行方向换边所需的对侧额外余量(m)
    'detour_w': 1.05,               # 绕行角速度
    'detour_on': 0,                 # 0 = 只驻留; 1 = 驻留超时后强制定向绕行
    'blk_s': 2.20,                  # P27 原地受阻多久后改后退脱困
    # P32: 丢失处理(严格视觉模式) —— 视觉丢失后不再用 Gazebo 真值跟踪
    'allow_gt_track': False,        # 是否允许真值兜底(仅供对照, 默认关)
    'lost_grace_s': 1.10,           # P38: 0.80→1.10, 远处偶发漏帧不算丢失
    'scan_dur_s': 12.0,             # 到达最后位置后原地扫描时长
    'goto_timeout_s': 12.0,         # P45: 20→12, 20s 静默死等纯浪费
    'goto_reach_m': 0.75,           # P45: 到点阈值, 原硬编码 0.5 比安全层
                                    # stop_d(0.66) 还小, 障碍旁数学上不可达
    'goto_stall_s': 3.0,            # P45: 3s 内靠近不足 8cm 即判定"过不去"
    'scan_dur_s2': 7.0,             # P45: 扫描一圈约 5.2s, 12s 是 2.3 圈
    'scan_rounds': 2,               # 最多"推测→前往→扫描"轮次
    # P81: 遮挡期"绕视"(PEEK) —— 人不是跑掉了, 而是被障碍挡住了。
    # 这时按"最后位置前往"是绕不出去的(人会沿着障碍继续走, 机器人的
    # 视线始终被同一根柱子切断), 实测绕柱场景 65s 内丢 10 次、最长 3.1s、
    # 累计 15.7s, 且 100% 的漏检都发生在视线被挡的瞬间。PEEK 让机器人
    # 沿视线法向朝远离障碍的一侧平移, 主动把视线绕出来。
    'peek_on': 1,                   # 0 = 关闭(退回原来的 GOTO)
    'peek_dur_s': 6.0,              # 绕视最长时长, 超时交给 GOTO
    'peek_pass_m': 0.60,            # 绕视方向上要求的净空(低于此值换方向)
    'scan_seek_s': 3.0,             # P81: 原地扫描超过这么久仍无检出 → 横移绕视
    'peek_v': 0.30,                 # 绕视平移速度 m/s
    'peek_kp': 2.2,                 # 绕视转向比例
}


def load_track_cfg():
    cfg = dict(DEFAULT_CFG)
    try:
        with open(CONFIG_PATH, encoding='utf-8') as f:
            data = json.load(f)
        for k in cfg:
            v = data.get(k)
            if v is not None:
                cfg[k] = float(v)
    except Exception:
        pass
    return cfg


# ---- 真值地图: 与 world 严格一致 ----
# P22 修正: 原表有 7 项, 其中 4 项(3.5,2.0)/(-3.0,3.2)/(4.2,-3.0)/(-1.5,-4.5)
# 在 work_area.sdf 里根本没有任何几何体, 半径也统一多写了 0.07。这些"幽灵
# 障碍"让绕障反复误触发: 机器人对着空气打方向(实测 w 冲到 2.2, 指令 v 冲到
# 0.6), 目标方位被甩到 ±22°, 看上去就像"原地打转"。下面是 SDF 里真实存在
# 的 3 根立柱, 半径按 <cylinder><radius> 原值填写。
WALL_SOFT = 5.15        # 接近该值开始向场内修正(提前+加强)
WALL_HARD = 5.75        # 不得越过
# P24 修正: 原表只剩 3 项(误判为"幽灵障碍"), 实际 SDF 的 obstacles 模型下
# 有 ob1..ob7 共 7 个几何体。漏项会让绕障直接撞柱子: 实测机器人顶着 ob3
# (0.5,-3.2) 绿柱, 指令 v=+0.12 而实际 v=+0.01, 原地卡死。
# 半径取水平外接圆(方块按半对角), 保证斜向也不会擦碰。
OBSTACLES = [           # (x, y, 外接半径) —— 与 work_area.sdf ob1..ob7 一一对应
    (3.5, 2.0, 0.57),     # ob1 0.8x0.8 方块
    (-3.0, 3.2, 0.67),    # ob2 1.2x0.6 方块(yaw 0.5)
    (0.5, -3.2, 0.40),    # ob3 r0.40 圆柱
    (-4.2, -2.5, 0.50),   # ob4 r0.50 圆柱
    (4.2, -3.0, 0.42),    # ob5 0.6x0.6 方块
    (-1.5, -4.5, 0.56),   # ob6 1.0x0.5 方块(yaw -0.4)
    (2.2, 4.6, 0.35),     # ob7 r0.35 圆柱
]
ROBOT_R = 0.18          # 机器人外接半径
INFLATE = ROBOT_R + 0.38  # 绕障膨胀余量
# P45: 真实几何表(与 work_area.sdf 的 ob1..ob7 完全一致)。
# OBSTACLES 存的是**外接圆**半径, 对外接圆做膨胀会把方块的四个角
# 吃掉一大块"实际可通行"的空间 —— ob6 是 1.0x0.5, 外接半径 0.56,
# 判据 0.56+0.56+0.15=1.27m, 而机器人到它的真实表面还有 0.73m。
# 真值兜底必须用 OBB 真实距离, 否则"明明能过却停车"。
# (kind, x, y, a, b, yaw)  box: a/b = 半长/半宽; cyl: a = 半径
GEO = [
    ('box', 3.5, 2.0, 0.40, 0.40, 0.0),      # ob1 0.8x0.8
    ('box', -3.0, 3.2, 0.60, 0.30, 0.5),     # ob2 1.2x0.6 yaw0.5
    ('cyl', 0.5, -3.2, 0.40, 0.0, 0.0),      # ob3 r0.40
    ('cyl', -4.2, -2.5, 0.50, 0.0, 0.0),     # ob4 r0.50
    ('box', 4.2, -3.0, 0.30, 0.30, 0.0),     # ob5 0.6x0.6
    ('box', -1.5, -4.5, 0.50, 0.25, -0.4),   # ob6 1.0x0.5 yaw-0.4
    ('cyl', 2.2, 4.6, 0.35, 0.0, 0.0),       # ob7 r0.35
]


def geo_gap(x, y, g):
    """点到障碍**表面**的距离(欧氏, 负=在内部)。box 按 OBB 解算。"""
    k, ox, oy, a, b, ya = g
    dx, dy = x - ox, y - oy
    if ya:
        c, sn = math.cos(-ya), math.sin(-ya)
        dx, dy = dx * c - dy * sn, dx * sn + dy * c
    if k == 'cyl':
        return math.hypot(dx, dy) - a
    qx, qy = abs(dx) - a, abs(dy) - b
    return math.hypot(max(qx, 0.0), max(qy, 0.0))


def _los_blocked(x0, y0, x1, y1):
    """P81: 视线 (x0,y0)→(x1,y1) 是否被已知障碍挡住。

    用途: 把"人跑出视野"和"人被障碍挡住"分开。前者按最后位置前往(GOTO)
    是对的; 后者在原地怎么转都等不到人 —— 实测绕柱场景里机器人 65s 内
    被同一根柱子挡住 10 次, 每次都按"丢失"处理, 于是反复 GOTO、距离被顶
    到 1.5m 还回不来。挡住时该做的是沿视线法向平移绕出遮挡区(PEEK)。

    返回: 挡住视线的那个障碍 (k,ox,oy,a,b,ya); 通视返回 None。
    """
    best, bd = None, 1e9
    for g in GEO:
        k, ox, oy, a, b, ya = g
        if ya:
            c, sn = math.cos(-ya), math.sin(-ya)
            ax, ay = (x0 - ox) * c - (y0 - oy) * sn, \
                (x0 - ox) * sn + (y0 - oy) * c
            bx, by = (x1 - ox) * c - (y1 - oy) * sn, \
                (x1 - ox) * sn + (y1 - oy) * c
        else:
            ax, ay = x0 - ox, y0 - oy
            bx, by = x1 - ox, y1 - oy
        dx, dy = bx - ax, by - ay
        hit = False
        if k == 'cyl':
            tt = -(ax * dx + ay * dy) / max(dx * dx + dy * dy, 1e-9)
            tt = max(0.0, min(1.0, tt))
            hit = math.hypot(ax + tt * dx, ay + tt * dy) < a
        else:
            t0, t1 = 0.0, 1.0
            for p, d, h in ((ax, dx, a), (ay, dy, b)):
                if abs(d) < 1e-9:
                    if p < -h or p > h:
                        hit = False
                        break
                else:
                    ta, tb = (-h - p) / d, (h - p) / d
                    if ta > tb:
                        ta, tb = tb, ta
                    t0, t1 = max(t0, ta), min(t1, tb)
                    if t0 > t1:
                        hit = False
                        break
            else:
                hit = True
        if hit:
            _d = math.hypot(ox - x0, oy - y0)
            if _d < bd:
                best, bd = g, _d
    return best


def path_free(x, y, ya, margin=0.20, s_max=1.60, step=0.05):
    """沿车头方向扫掠。返回 (可用直行距离, 走廊是否够宽, 首个拦截物)。

    P53/P57。为什么不再用 `min(d_fl,d_fr)*0.72`: 8 扇区每个方向只有一个
    "最近距离", 拿 45° 扇区值乘 0.72 当"正前方余量"是没有真值地图时代的
    代偿 —— 它假定障碍正好在 45°, 而扇区跨度约 20~70°, 实际角度未知。
    实测 eval14 的死锁现场: ob2 的角在车头左前 64°、车心到表面 0.395m,
    沿 39° 航向通过时法向间隙 0.355m(车体半径 0.18) —— 完全过得去;
    但 0.395*0.72 = 0.284 < stop_d 0.66 直接急停, 急停又强制反向转,
    与上层"转向目标"互掐, 造成极限环。

    两个返回值必须分开, 否则重复扣车体半径:
      d_surf —— 沿航向到"车心将触到表面"的距离, 就是**表面距离**,
                和 d_f 同量纲, 可以直接和 stop_d/slow_d 比较;
      tight  —— 走廊宽度够不够(沿途任一点中心间隙 < ROBOT_R+margin
                即 False)。这一项单独拦"贴着角切过去": 实测 evalN
                (PW_SEED=7) 机器人贴着 ob5(4.2,-3.0) 的角、以 0.25m
                中心间隙切入, 然后卡在角里 —— 清距 0.07m, 既转不动
                (车体外接圆扫进障碍)、也退不出(ob5 在侧后方, d_b 被
                判"后退受阻"), 冻死 45s, 人从 1.4m 走到 5.96m。
    """
    best = 99.0
    tight = True
    s_tight = s_max        # P73: 窄段起点(第一次 tight 失效处的弧长)
    s = step
    while s <= s_max + 1e-9:
        nx = x + math.cos(ya) * s
        ny = y + math.sin(ya) * s
        if abs(nx) > WALL_HARD or abs(ny) > WALL_HARD:
            return max(0.0, s - step), tight, 'wall', s_tight
        gmin, gobj = 99.0, None
        for g in GEO:
            gp = geo_gap(nx, ny, g)
            if gp < gmin:
                gmin, gobj = gp, g
        if gmin < best:
            best = gmin
        if gmin <= ROBOT_R + margin:
            if tight:
                s_tight = s
            tight = False
        if gmin <= 0.02:            # 车心即将触到表面 → 这就是可用距离
            return max(0.0, s - step), tight, gobj, s_tight
        s += step
    return s_max, tight, None, s_tight


def fit_slope(win):
    """最小二乘拟合 (t, v) 窗口的斜率 dv/dt, 样本不足返回 None。"""
    n = len(win)
    if n < 4:
        return None
    st = sum(w[0] for w in win)
    sv = sum(w[1] for w in win)
    stt = sum(w[0] * w[0] for w in win)
    stv = sum(w[0] * w[1] for w in win)
    den = n * stt - st * st
    if den <= 1e-9:
        return None
    return (n * stv - st * sv) / den


def fit_vel(win):
    """最小二乘拟合窗口内 (t, x, y) 的速度 → (vx, vy), 样本不足返回 None。

    P73。原来是相邻帧差分的 EWMA(a=0.35): 视觉方位量化约 1°, 在 1.5m
    处相当于 2.6cm, 10Hz 下就是 0.26m/s 的单轴噪声, EWMA 之后仍有
    ±0.12; 人走到 2.5m 时噪声涨到 ±0.2 —— 前馈于是变成"随机推/随机
    拖"。实测日志抓到目标正在远离(距离 1.44→1.56 单调增)而
    ff=-0.25(全速倒拖)的样本。窗口拟合噪声按 1/sqrt(n) 收, 且没有
    EWMA 的相位滞后。
    """
    n = len(win)
    if n < 4:
        return None
    st = sum(w[0] for w in win)
    stt = sum(w[0] * w[0] for w in win)
    den = n * stt - st * st
    if den <= 1e-9:
        return None
    sx = sum(w[1] for w in win)
    sy = sum(w[2] for w in win)
    stx = sum(w[0] * w[1] for w in win)
    sty = sum(w[0] * w[2] for w in win)
    return ((n * stx - st * sx) / den, (n * sty - st * sy) / den)


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def yaw_of(q):
    """四元数 → 航向角.

    P20 修正: 分母必须是 1-2*(y²+z²)。原代码写成 1-2*(y²+x²), 会把航向角
    严重压缩(实测真值 90° 读成 45°、180° 读成 0°、141° 读成 32°), 于是
    "目标方位" 与 "实际航向" 对不上, 控制器永远算不出正确的转角 ——
    表现为机器人原地打转、朝任何方向都追不上目标。
    """
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def seg_dist(cx, cy, ax, ay, bx, by):
    """点 (cx,cy) 到线段 (ax,ay)-(bx,by) 的距离"""
    abx, aby = bx - ax, by - ay
    denom = abx * abx + aby * aby
    if denom < 1e-9:
        return math.hypot(cx - ax, cy - ay)
    t = ((cx - ax) * abx + (cy - ay) * aby) / denom
    t = max(0.0, min(1.0, t))
    return math.hypot(cx - (ax + t * abx), cy - (ay + t * aby))


class Follower(Node):

    def __init__(self):
        super().__init__('follower')
        self.cfg = load_track_cfg()
        # P74: 统一时间源 —— 优先 /clock(仿真时间), 回退 wall clock。
        # 原来全程 wall clock, 与 gz 的仿真步进(RTF 波动)解耦,
        # 同一 PW_SEED 两次跑人的轨迹能差 11m, 评测无法 A/B。
        self._t0_wall = time.monotonic()
        self._t0_sim = None
        # P65: GOTO 段由 Nav2 NavigateToPose 接管(规划+绕障+恢复行为),
        # follower 只发目标/取消/模式开关; /robot/cmd_vel 由 cmd_mux 按模式择路
        self.pub = self.create_publisher(Twist, '/follower_cmd_vel', 10)
        self.pub_nav_active = self.create_publisher(Bool, '/nav2_active', 10)
        self._nav_client = None
        self._nav_goal_future = None
        self._nav_goal_handle = None
        self._nav_result_future = None
        self._nav_goal_xy = None
        self.create_subscription(String, '/robot/command', self._cmd_cb, 10)
        self.create_subscription(String, '/perception/proximity',
                                 self._prox_cb, 10)
        self.create_subscription(String, '/perception/person',
                                 self._vis_cb, 10)
        self.create_subscription(String, '/perception/person_history',
                                 self._hist_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self._gz_node = gz_transport.Node()
        self._gz_node.subscribe(GzPose_V,
                                '/world/follow_world/dynamic_pose/info',
                                self._pose_cb)
        self._lock = threading.Lock()
        self.state = 'IDLE'
        self.target_d = self.cfg['track_dist_m']
        self.rx, self.ry, self.ryaw = 0.0, 0.0, 0.0
        self.px, self.py = 1.0, 0.0
        self._r_prev = None
        self._p_prev = None
        self.robot_speed = 0.0
        self.person_speed = 0.0
        self.scan_count = 0
        self.scan = None          # P25: (ranges, angle_min, angle_inc, mono)
        # P26: 8 个 45° 方位的全向测距(4 台深度相机分扇区合成)
        self.prox = {'front': -1.0, 'fl': -1.0, 'left': -1.0, 'bl': -1.0,
                     'back': -1.0, 'br': -1.0, 'right': -1.0, 'fr': -1.0,
                     'age': 99.0}
        self.vis = None
        self._vis_mono = 0.0
        self._vis_msg_mono = 0.0      # P23: 视觉消息到达时刻(含未检出帧)
        self._vis_prev = None
        self.vis_pspd = 0.0
        # P40: 人速估计(视觉世界坐标差分) —— 距离环前馈用。
        # 纯误差反馈的距离环有个固有缺陷: 稳态误差 = 人速/kp。人走 0.30m/s、
        # kp=0.6 时稳态偏差就有 0.5m, 机器人永远"慢半拍" —— 实测 150s 里
        # 55.6% 的时间被拉开到 1.1m 以外。加前馈后误差只需修正残差。
        # P43: 切线绕障状态(方向锁存, 防左右翻转死锁)
        self._det_dir = 0.0
        self._det_t0 = 0.0
        self._pv_t = None
        self._pv_x = 0.0
        self._pv_y = 0.0
        self._pvx = 0.0
        self._pvy = 0.0
        # P73: 视觉世界坐标样本窗[(t, px, py)], 供 fit_vel 估人速
        self._pv_win = []
        self._pv_cam = ''
        # P73: 距离变化率观测窗 [(t, 视觉距离)], 与本机前向速度合成人速
        self._rr_win = []
        self._pvvr = 0.0        # 估计出来的人速(沿机头径向, 远离为正)
        self.robot_vfwd = 0.0   # 本机前向速度(有符号, 真值位姿差分)
        self._src_prev = ''
        # 快照式跟踪状态机(P12): TRACK | GOTO | SCAN
        self._track = 'TRACK'
        self._search_target = None
        self._search_t0 = 0.0
        self._scan_until = 0.0
        self._reacq_until = 0.0
        # P32: 严格视觉模式 / 丢失处理状态
        self._vis_ok = None           # 最后一帧"确实检出"的视觉结果
        self._last_vis_xy = None      # 最后一次视觉确认的目标世界坐标
        self._lost = False            # 是否处于目标丢失态
        self._lost_at = 0.0           # 丢失开始时刻
        self._scan_round = 0          # "推测→前往→扫描"已完成轮次
        # P81: 遮挡期绕视(PEEK)
        self._peek_t0 = 0.0           # 本轮绕视开始时刻
        self._peek_xy = None          # 绕视期间"人最后可见"的世界坐标
        self._peek_obs = None         # 挡住视线的那个障碍
        self._peek_vd = (0.0, 0.0)    # 进入绕视时人的世界速度向量(m/s)
        self._peek_from = 'TRACK'     # 绕视前所在状态(决定超时后回哪)
        self._peek_n = 0              # 累计绕视次数(diag)
        # P45: GOTO 进度监视 —— 目标点被障碍卡住时提前退出, 别死等超时
        self._goto_best = None        # 历史最近目标点距离
        self._goto_prog_t = 0.0       # 最后一次"确实靠近了"的时刻
        self._lost_announced = False  # 是否已提示"目标丢失"
        # P47: 视觉时空一致性滤波
        self._fdist = None            # 滤波后的目标距离
        self._fbrg = None             # 滤波后的目标方位(相对车头)
        self._fhold_until = 0.0       # 跳变抑制截止时刻
        self._fcam = ''               # 上一帧主相机
        # P77 §17-2: 近距离方位限幅用的时间戳 / 计数(进 diag)
        self._fbrg_t = 0.0            # 上一帧方位更新时刻
        self._brg_clip = 0            # 被限幅/夹住的累计次数
        self._brg_clip_rate = 0.0     # 最近 1s 发生率(0~1)
        self._brg_clip_win = []       # 限幅事件时间窗
        self._status_pub = self.create_publisher(
            String, '/follower/status', 10)
        self._pub_status_t = 0.0
        self.hist = []
        # PID 状态(P16)
        self._int_d = 0.0
        self._prev_err_d = 0.0
        self.v = 0.0
        self._pub_t = None        # P21: 上次发布时刻, 用于真实 dt 的加速度限幅
        self._last_w = 0.0
        self._last_note = ''          # P53: 最近一次避障决策说明
        # P73: 距离环控制链快照(仅用于日志诊断)。v_t 最终为 0 时,
        # 看 ff/pid/cap/dh 谁先归零, 就能定位是哪一层把它掐掉的。
        self._dbg = {'ff': 0.0, 'pid': 0.0, 'cap': 0.0,
                     'dh': -1.0, 'df': -1.0}
        # P76: 10Hz 结构化诊断 —— 只靠 0.5s 一条的文本日志无法归因"障碍旁徘徊"
        # (文本里 指令v 和 实际v 是不同量纲, 且 note 只在变化时才有)。
        # 这里把安全层内部量全部导出成 JSON, 离线统计"徘徊时段被哪条规则掐住"。
        self._diag_pub = self.create_publisher(String, '/follower/diag', 10)
        self._diag_t = 0.0
        self._last_dist = 0.0
        self._last_brg = 0.0
        # 对正状态(P20)
        self._aligning = False
        self._align_dir = 0.0     # ±180° 附近的方向锁存
        self._align_edge_t = 0.0  # P77: 越界/回带的起始时刻(时间滞回)
        # P77 §17-3/§17-4: 转向仲裁状态
        self._w_owner = ''        # 当前掌握转向的模块
        self._w_owner_t = 0.0     # 该模块开始掌握的時刻
        self._w_req = ''          # 正在申请的模块
        self._w_req_t = 0.0       # 申请起始时刻
        self._w_sign = 0.0        # 已生效的转向方向(±1)
        self._w_opp_t = 0.0       # 连续反向请求起始时刻
        self._w_flip = 0          # _arb_w 内部实际换向次数
        self._w_flip_win = []     # 换向时间窗(算次/秒)
        self._w_dir_last = 0.0    # 下发 w 的上一个符号(统计用)
        self._w_flip_rate = 0.0   # 最近 1s 实际换向率(进 diag)
        self._w_pub_win = []      # 下发 w 的换向时间窗(客观指标, 与仲裁器无关)
        self._w_pub_rate = 0.0
        self._blind_win = []     # P80: 盲走禁令触发时刻(最近 1s)    # 最近 1s 下发 w 的换向率(0 最好)
        # P77 §17-4: 受阻驻留 / 定向绕行窗口
        self._hold_t0 = 0.0       # 连续受阻起始时刻(算 hold_max_s)
        self._detour_dir = 0.0    # 绕行方向(±1), 0 = 未在绕行
        self._detour_until = 0.0  # 绕行窗口截止时刻
        self._detour_t0 = 0.0     # 窗口起始时刻
        self._slow_fac = 1.0      # P79: 接近减速系数(1=不减速)
        self._last_slow_log = 0.0
        self._last_back_t = 0.0   # P79: 上次下发后退指令的时刻(统计频繁后退)
        # P78: 受阻解除后的前进软启动
        self._was_stalled = False  # 上一帧是否处于"受阻"
        self._resume_t0 = 0.0      # 解除受阻的时刻
        self._stuck_t0 = 0.0
        self._stuck_pos = (0.0, 0.0)
        self._stuck_yaw = 0.0
        # P25: 脱困窗口(卡死后持续倒退+转向, 而不是只发一个 tick)
        self._escape_until = 0.0
        self._escape_v = 0.0
        self._escape_w = 0.0
        self._bypass = None       # (障碍索引, 绕行侧)
        self._bypass_t0 = 0.0     # 当前绕行侧开始时间
        self._bypass_arc = 0.0    # 轮廓跟随的弧进度(rad)
        self._bypass_last = None  # (i, side, arc, t) 短暂释放记忆
        self._last_pose_mono = 0.0
        self._last_pose_warn = 0.0
        self._last_log = 0.0
        self._last_diag = 0.0
        self._last_avoid_log = 0.0    # P26: 全向安全层单独的日志限流
        # P27: 急停绕行的方向记忆 —— 没有它 w 会每帧翻号(见 patch 说明)
        self._esc_dir = 0.0           # -1 右转 / +1 左转 / 0 未定
        self._esc_t0 = 0.0            # 当前方向锁定的起始时刻
        self._estop_latch = False     # P77: 急停吸住态(释放需 1.25*stop_d 净空)
        self._estop_hold_t = 0.0      # P77: 本次吸住起始时刻
        self._blk_t0 = 0.0            # 前方持续受阻的起始时刻(0=未受阻)
        # P46: 锁存式绕障状态
        self._avoiding = False        # 是否处于绕行态
        self._avoid_side = 1.0        # +1 左绕 / -1 右绕(进入时锁死, 不重算)
        self._avoid_d0 = 0.0          # 进入绕行时的目标距离(Bug2 的基准)
        self._avoid_t0 = 0.0          # 进入绕行时刻
        self._blk2_t0 = 0.0           # 绕障触发计时(与脱困用的 _blk_t0 分开)
        # P49: 绕行航向必须在**世界系**锁存。用"相对车头的固定偏角"会让
        # w 恒为 2.4*72°=3.0(钳 1.7), 车头转了偏角还是 72° → 匀速转圈。
        self._vfh_w = None            # P52: VFH 选定的世界系方向(0.4s 锁存)
        self._vfh_t = 0.0             # 上次重选时刻
        self._vfh_hint = ''           # 日志用
        self._avoid_hdg = 0.0         # 兼容保留
        self._avoid_hdg_t = 0.0
        # P30: 感知看门狗 —— 按**收到消息的时刻**判断, 不能只信 payload 里的
        # age(那是发布方算的, 发布方死了就永远冻在小值上)
        self._prox_rx = 0.0           # 最近一次收到 proximity 的时刻
        self._prox_dead_log = 0.0
        # P26: 雷达可用性自检 —— gpu_lidar 只支持 ogre2, 本项目为绕开 VMware
        # 白屏固定用 ogre1, 于是 /scan 会返回全 range_min。开局探一次并落日志,
        # 避免以后误以为"雷达在参与避障"。
        self._lidar_checked = False
        self._lidar_ok = False
        self.create_timer(0.05, self._tick)  # 20 Hz
        self.get_logger().info(
            'FOLLOWER 跟随控制器v4就绪(对正带±%.0f° / 距离PID / 真值地图绕障)'
            % self.cfg['aim_band_deg'])

    # ---------- 回调 ----------
    def _cmd_cb(self, msg):
        try:
            cmd = json.loads(msg.data)
        except Exception:
            self.get_logger().warn(f'FOLLOWER 无法解析指令: {msg.data}')
            return
        action = cmd.get('action')
        if action == 'FOLLOW':
            d = cmd.get('distance')
            # P79: 目标距离**钳位到允许区间**(track_dist_m ± tol = 0.5~1.5m)。
            # 起因: demo 启动时播放的语音是"请跟随我, 保持两米的距离",
            # voice_trigger 的默认 wav 就是 follow_2m.wav, 于是 target_d 被
            # 设成 2.0m —— 控制器按 2m 去追, 与"0.5~1.5m"的设定冲突, 而且
            # 2m 时 err_d=+0.8 会让 PID 长期饱和前进, 距离环永远 settle 不了。
            # 外部指令(语音/遥控/评测脚本)一律服从这个区间, 越界只告警不生效。
            _lo = float(self.cfg['track_dist_m']) - float(self.cfg['track_dist_tol_m'])
            _hi = float(self.cfg['track_dist_m']) + float(self.cfg['track_dist_tol_m'])
            if d is None:
                self.target_d = float(self.cfg['track_dist_m'])
            else:
                _d = float(d)
                if _d < _lo - 1e-6 or _d > _hi + 1e-6:
                    self.get_logger().warn(
                        f'FOLLOW 请求距离 {_d:.2f}m 超出允许区间 '
                        f'{_lo:.2f}~{_hi:.2f}m, 已钳位为 '
                        f'{max(_lo, min(_hi, _d)):.2f}m')
                    _d = max(_lo, min(_hi, _d))
                self.target_d = _d
            self.state = 'FOLLOW'
            self._int_d = 0.0
            self._prev_err_d = 0.0
            self.get_logger().info(
                f'FOLLOWER ★ 收到跟随指令(来自语音: "{cmd.get("text", "")}"), '
                f'目标距离 = {self.target_d:.2f}m — 开始跟随')
        elif action == 'STOP':
            self.state = 'IDLE'
            self.v = 0.0
            self._nav_cancel('收到停止指令')
            self._publish(0.0, 0.0)
            self.get_logger().info(
                f'FOLLOWER ★ 收到停止指令(来自语音: "{cmd.get("text", "")}"), '
                f'已停止跟随')
        else:
            self.get_logger().info(
                f'FOLLOWER 忽略指令: {json.dumps(cmd, ensure_ascii=False)}')

    # ---------- Nav2 搜索段接口 (P65) ----------
    def _nav_send(self, gx, gy):
        """向 Nav2 下发 NavigateToPose 目标(人最后出现位置)"""
        if self._nav_client is None:
            self._nav_client = ActionClient(self, NavigateToPose,
                                            'navigate_to_pose')
        if not self._nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('FOLLOWER Nav2 action server 未就绪, 就地扫描')
            self._enter_scan(now=self._now(), announce=False)
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(gx)
        goal.pose.pose.position.y = float(gy)
        goal.pose.pose.orientation.w = 1.0
        self._nav_goal_future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._nav_feedback)
        self._nav_goal_future.add_done_callback(self._nav_goal_cb)
        self._nav_goal_xy = (gx, gy)
        self._nav_result_future = None
        self.get_logger().info(
            f'FOLLOWER ★ Nav2 接管搜索: 前往({gx:+.2f},{gy:+.2f})')

    def _nav_goal_cb(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().warn('FOLLOWER Nav2 目标被拒绝, 就地扫描')
            self._nav_goal_handle = None
            self._enter_scan(now=self._now(), announce=False)
            return
        self._nav_goal_handle = gh
        self._nav_result_future = gh.get_result_async()
        self._nav_result_future.add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, fut):
        # P71: rclpy 的结果 future 解析出来的是 GetResult 响应,
        #      action 状态码在 resp.status, 动作返回值在 resp.result。
        #      原代码取 fut.result().result.status —— NavigateToPose_Result
        #      里没有 status 字段 → AttributeError → follower 进程直接死掉
        #      (Nav2 一有结果就崩, 表现为"每完成一次 GOTO 就跟丢了")。
        try:
            resp = fut.result()
            status = int(getattr(resp, 'status', -1))
            ret = getattr(resp, 'result', None)
            code = int(getattr(ret, 'error_code', 0) or 0)
            self.get_logger().info(
                'FOLLOWER Nav2 结果: status=%d error_code=%d' % (status, code))
        except Exception as exc:                     # noqa: BLE001
            self.get_logger().warn(
                'FOLLOWER Nav2 结果回调异常(已忽略): %r' % (exc,))
        self._nav_goal_handle = None
        self._nav_result_future = None
        if self._track == 'GOTO':
            # 到达(或 Nav2 放弃) → 扫描确认; 是否再推测由扫描逻辑决定
            self._enter_scan(now=self._now(), announce=False)

    def _nav_cancel(self, why):
        if self._nav_goal_handle is not None:
            try:
                self._nav_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.get_logger().info('FOLLOWER Nav2 目标已取消: %s' % why)
        self._nav_goal_handle = None
        self._nav_result_future = None
        self._nav_goal_xy = None
        self.pub_nav_active.publish(Bool(data=False))

    def _nav_feedback(self, fb):
        pass

    def _now(self):
        """P74: 当前时间(s)。有 /clock 就用仿真时间(归零到节点启动),
        否则回退 wall clock —— 两条分支都从 0 起, 切换时只跳几百 ms。"""
        try:
            ns = int(self.get_clock().now().nanoseconds)
        except Exception:
            ns = 0
        if ns > 0:
            if self._t0_sim is None:
                self._t0_sim = ns * 1e-9
            return ns * 1e-9 - self._t0_sim
        return time.monotonic() - self._t0_wall

    def _pose_cb(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        robot = person = None
        for p in msg.pose:
            if p.name == 'robot':
                robot = p
            elif p.name == 'person':
                person = p
        if robot is None or person is None:
            return
        with self._lock:
            self.rx = robot.position.x
            self.ry = robot.position.y
            self.ryaw = yaw_of(robot.orientation)
            self.px = person.position.x
            self.py = person.position.y
            if self._r_prev is not None and stamp > self._r_prev[0] + 1e-3:
                pt, prx, pry = self._r_prev
                dtp2 = stamp - pt
                inst = math.hypot(robot.position.x - prx,
                                  robot.position.y - pry) / dtp2
                self.robot_speed = 0.6 * self.robot_speed + 0.4 * min(inst, 2.0)
                # P73: 有符号前向速度 —— 人速前馈要用它把"距离变化率"
                # 折算成人的绝对速度(v_person = v_robot + d(dist)/dt)。
                _yf = yaw_of(robot.orientation)
                _vf = ((robot.position.x - prx) * math.cos(_yf)
                       + (robot.position.y - pry) * math.sin(_yf)) / dtp2
                self.robot_vfwd = 0.7 * self.robot_vfwd + \
                    0.3 * max(-1.5, min(1.5, _vf))
            if self._p_prev is not None and stamp > self._p_prev[0] + 1e-3:
                pt, ppx, ppy = self._p_prev
                inst = math.hypot(person.position.x - ppx,
                                  person.position.y - ppy) / (stamp - pt)
                self.person_speed = 0.6 * self.person_speed + 0.4 * min(inst, 2.0)
            self._r_prev = (stamp, robot.position.x, robot.position.y)
            self._p_prev = (stamp, person.position.x, person.position.y)
        self._last_pose_mono = self._now()

    def _scan_cb(self, msg):
        """P25: 保存 360° 雷达测距, 供全向安全层使用。

        原实现只做 scan_count += 1 —— 雷达帧数仅用于打日志, 也就是说这台
        机器人"看得见 360°"却完全没用它避障, 避障只剩两样东西:
          (a) 硬编码的真值障碍表(依赖人工维护, 只能预判正前方)
          (b) 5Hz 的深度相机, 且最小停车距离取 0.30m
        而车体半径 0.18m + 立柱半径 0.40m = 中心距 0.58m 时就已物理接触,
        所以"等 0.30m 才停"必然撞上去; 斜向接近时地图层的前向点积又判不到,
        于是表现为"有时能绕开、有时直接撞"。
        """
        with self._lock:
            self.scan_count += 1
            self.scan = (list(msg.ranges), float(msg.angle_min),
                         float(msg.angle_increment), self._now())

    def _sector_min(self, lo_deg, hi_deg):
        """雷达扇区 [lo_deg, hi_deg] 内的最小距离(米)。相对车头, 逆时针为正。

        无有效回波返回 -1。自动过滤 nan / inf / 贴脸噪声点(<=6cm)。
        """
        sc = self.scan
        if sc is None:
            return -1.0
        rng, amin, ainc, _ = sc
        n = len(rng)
        if n == 0 or ainc <= 1e-6:
            return -1.0
        i0 = int(round((math.radians(lo_deg) - amin) / ainc))
        i1 = int(round((math.radians(hi_deg) - amin) / ainc))
        best = -1.0
        for i in range(i0, i1 + 1):
            r = rng[i % n]
            if r is None or not (0.06 < r < 11.5):
                continue          # nan / inf / 无效点: nan 比较恒为 False
            if best < 0.0 or r < best:
                best = r
        return best

    def _prox_cb(self, msg):
        """8 方位全向测距: front/fl/left/bl/back/br/right/fr + age"""
        try:
            p = json.loads(msg.data)
        except Exception:
            return
        keys = ('front', 'fl', 'left', 'bl', 'back', 'br', 'right', 'fr')
        self.prox = {k: p.get(k, -1.0) for k in keys}
        self.prox['age'] = p.get('age', 99.0)
        self._prox_rx = self._now()      # P30 看门狗

    def _vis_cb(self, msg):
        """相机人检测: {detected, cam, dist, bearing, bearing_cam, u, area}"""
        try:
            v = json.loads(msg.data)
        except Exception:
            return
        self.vis = v
        self._vis_msg_mono = self._now()
        # P23: 只有"确实检出"才刷新检出生效时刻。
        # 原实现每帧无条件刷新(包括 detected=false), 于是
        # "now - _vis_mono > 2.0"(视觉丢失 2s) 永远不成立 ——
        # 目标丢失后的 GOTO→SCAN 搜索链路整段是死代码, 实测 24 次
        # "四相机视野内均无人" 却 0 次触发搜索。
        # P32: 同时缓存最后一帧"检出"结果, 单帧漏检时上层还能在
        # grace 窗口内继续用这一帧跟随, 而不是立刻掉进丢失分支。
        if v.get('detected'):
            self._vis_mono = self._vis_msg_mono
            self._vis_ok = v


    def _hist_cb(self, msg):
        """快照历史: {seq:[{t,cam,bearing,dist,wx,wy}...], lateral:bool}"""
        try:
            h = json.loads(msg.data)
        except Exception:
            return
        self.hist = h.get('seq', [])

    def _free_target(self, tx, ty, margin=0.22):
        """P45: 把目标点从障碍膨胀区里推出来。

        GOTO 的目标点来自"最后 10 张快照 + 1.5s 外推", 很容易落在障碍
        里 —— 机器人对着障碍推到超时也到不了。这里沿"障碍中心→目标点"
        方向把点推到表面外 0.22m(车体 0.18 + 余量), 最多迭代 6 轮处理
        多个障碍互相重叠的情况。
        """
        for _ in range(6):
            moved = False
            for g in GEO:
                gap = geo_gap(tx, ty, g)
                if gap >= ROBOT_R + margin:
                    continue
                vx, vy = tx - g[1], ty - g[2]
                nn = math.hypot(vx, vy)
                if nn < 1e-6:
                    vx, vy, nn = 1.0, 0.0, 1.0
                push = (ROBOT_R + margin) - gap + 0.02
                tx += vx / nn * push
                ty += vy / nn * push
                moved = True
            if not moved:
                break
        # 别推到墙外: 用 WALL_SOFT(5.15) 而不是 5.3, 给贴墙修正留空间
        tx = max(-WALL_SOFT, min(WALL_SOFT, tx))
        ty = max(-WALL_SOFT, min(WALL_SOFT, ty))
        return tx, ty

    def _person_vdir(self):
        """P81: 由快照历史估"人最后的世界速度向量"(m/s)。

        遮挡期间视觉不刷新, 但人在继续走 —— GOTO/绕视都必须知道人往哪走,
        否则只能朝"人最后出现过的位置"走, 而人早就不在那儿了(_infer_target
        也是这个思路, 只是它外推 1.5s 限幅 1.2m, 太粗, 不能每个 tick 用)。
        """
        h = self.hist or []
        if len(h) < 2:
            return (0.0, 0.0)
        n = min(4, len(h))
        w = h[-n:]
        dt = max(float(w[-1]['t']) - float(w[0]['t']), 0.5)
        vx = (float(w[-1]['wx']) - float(w[0]['wx'])) / dt
        vy = (float(w[-1]['wy']) - float(w[0]['wy'])) / dt
        sp = math.hypot(vx, vy)
        if sp > 0.75:
            vx, vy = vx / sp * 0.75, vy / sp * 0.75
        return (vx, vy)

    def _enter_scan(self, now, announce=False):
        """P32: 进入原地 360° 扫描态; announce=True 时明确提示"目标丢失" """
        _was = self._track          # P81: 是否"新"进入扫描
        self._track = 'SCAN'
        self.pub_nav_active.publish(Bool(data=False))   # P65: 停用 Nav2 驱动
        self._nav_cancel('转入扫描')
        # P45: 12s 是 2.3 圈(转速 1.2rad/s 时一圈 5.2s), 纯浪费
        self._scan_until = now + float(self.cfg.get(
            'scan_dur_s2', self.cfg['scan_dur_s']))
        # P81: 扫描起点只在**新进入**扫描时刷新。上一版无条件刷新, 而丢失
        # 判定块每 tick 都会调一次 _enter_scan(scan_round 用尽时), 于是
        # _scan_t0 每帧都被推到现在 —— "扫够 3s 就横移"的判据永远不成立,
        # 机器人被柱子挡住时原地空转 20s+ 也不会动一下。
        if _was != 'SCAN':
            self._scan_t0 = now
        if announce and not self._lost_announced:
            self._lost_announced = True
            where = ('(%+.1f,%+.1f)' % self._last_vis_xy
                     if self._last_vis_xy is not None else '(未知)')
            self.get_logger().error(
                'FOLLOWER ✗✗ 目标丢失: 已按最后 %d 张快照前往并搜索 %d 轮仍无'
                '检出, 最后确认位置%s → 原地扫描, 等待目标重新进入视野'
                % (min(len(self.hist), 10), self._scan_round, where))

    def _infer_target(self):
        """P32: 用最后 10 张快照推测人最后所处的位置。

        位置取窗口尾部 1/3 的均值(最近几帧最可信, 人可能已驻足),
        速度用窗口首末差(抑制单帧噪声), 外推 1.5s 且限幅 1.2m ——
        要求是"先移动到人最后所处的位置", 不宜激进外推。
        """
        CAM_CN = {'front': '前', 'left': '左', 'right': '右', 'back': '后'}
        if not self.hist:
            return None, '无快照历史'
        n = min(10, len(self.hist))
        win = self.hist[-n:]
        k = max(1, n // 3)
        tail = win[-k:]
        lx = sum(float(h['wx']) for h in tail) / len(tail)
        ly = sum(float(h['wy']) for h in tail) / len(tail)
        tx, ty = lx, ly
        spd = 0.0
        if n >= 2:
            dt = max(float(win[-1]['t']) - float(win[0]['t']), 1.0)
            vx = (float(win[-1]['wx']) - float(win[0]['wx'])) / dt
            vy = (float(win[-1]['wy']) - float(win[0]['wy'])) / dt
            spd = math.hypot(vx, vy)
            if spd > 0.12:                    # 站立不动时不外推
                ex, ey = vx * 1.5, vy * 1.5
                e = math.hypot(ex, ey)
                if e > 1.2:
                    ex, ey = ex * 1.2 / e, ey * 1.2 / e
                tx, ty = lx + ex, ly + ey
        tx = max(-5.3, min(5.3, tx))
        ty = max(-5.3, min(5.3, ty))
        # P45: 外推点常常落进障碍里 → 机器人永远到不了。先推出来。
        tx, ty = self._free_target(tx, ty)
        seq = [h.get('cam', '?') for h in win]
        seq_cn = ','.join(CAM_CN.get(c, c) for c in seq[-6:])
        lateral = len(set(seq)) > 1
        desc = ('方位序列[%s]%s, 末位置(%+.1f,%+.1f) 人速%.2f → 前往(%+.1f,%+.1f)'
                % (seq_cn, '横向移动' if lateral else '同相机',
                   lx, ly, spd, tx, ty))
        return (tx, ty), desc


    # ---------- 绕障航点 ----------
    def _waypoint(self, rx, ry, px, py):
        """返回 (tx, ty, note, 到航点距离)。

        连线穿障碍膨胀圆时, 取切点为初始航点; 机器人到达切点而连线仍被挡
        时, 航点沿圆弧向绕行侧推进(轮廓跟随), 直到连线畅通;
        短暂释放后重新命中同一障碍时保留侧别与弧进度(防边缘翻转);
        单侧绕行超过 10s 或走满一圈仍被挡 → 换边重来。"""
        best = None
        dist_t = math.hypot(px - rx, py - ry)
        for i, (ox, oy, r) in enumerate(OBSTACLES):
            R = r + INFLATE
            # P22 修正: 障碍必须真的挡在"机器人→目标"之间。
            # 只用 seg_dist() 是不够的 —— 它把投影参数 t 夹到 [0,1], 于是停在
            # 目标"身后"的障碍也会被算成挡路。实测: 人在 (-2.4,-4)、障碍5在
            # (-1.5,-4.5)(人在其前方 1m), 判定仍命中 → 绕障接管角速度通道,
            # 目标方位被反复甩到 +22°, 机器人绕着一个根本没挡路的东西打转。
            # 这里显式要求投影落在 (0.05, 0.97) 区间内, 即障碍严格在目标之前。
            t = 0.0
            if dist_t > 1e-6:
                t = ((ox - rx) * (px - rx) + (oy - ry) * (py - ry)) / dist_t ** 2
            if not (0.05 < t < 0.97):
                continue
            d = seg_dist(ox, oy, rx, ry, px, py)
            # 迟滞带: 新触发需深入 0.15m; 已在绕行则越出 0.05m 才释放
            if self._bypass is not None and self._bypass[0] == i:
                blocked = d < R + 0.05
            else:
                blocked = d < R - 0.15
            if blocked:
                pen = R - d
                if best is None or pen > best[0]:
                    best = (pen, i, ox, oy, R)
        now = self._now()
        if best is None:
            if self._bypass is not None:
                self._bypass_last = (self._bypass[0], self._bypass[1],
                                     self._bypass_arc, now)
            self._bypass = None
            self._bypass_t0 = 0.0
            self._bypass_arc = 0.0
            return px, py, '', 0.0
        _, i, ox, oy, R = best
        side = -1.0 if ((px - rx) * (oy - ry) - (py - ry) * (ox - rx)) > 0 else 1.0
        arc = 0.0
        if self._bypass is not None and self._bypass[0] == i:
            side = self._bypass[1]
            arc = self._bypass_arc
            if now - self._bypass_t0 > 10.0 or arc > 6.0:
                side = -side
                arc = 0.0
                self._bypass_t0 = now
                self.get_logger().info(
                    f'FOLLOWER 绕障{i}换边{"左" if side > 0 else "右"}')
        else:
            last = self._bypass_last
            if (last is not None and last[0] == i
                    and now - last[3] < 2.5):
                side = last[1]      # 短暂释放: 续用原侧与弧进度
                arc = last[2]
            self._bypass_t0 = now
        self._bypass = (i, side)
        d_ro = math.hypot(rx - ox, ry - oy)
        ac = math.atan2(ry - oy, rx - ox)      # 圆心->机器人 方向
        if d_ro <= R + 0.05:
            at = ac + side * math.pi / 2       # 已在圆边: 沿切向走出
        else:
            alpha = math.acos(min(1.0, R / d_ro))
            at = ac + side * alpha             # 切点方位(从圆心看)
        at += side * arc                       # 轮廓跟随: 弧推进
        wx = ox + R * math.cos(at)
        wy = oy + R * math.sin(at)
        wx = max(-5.2, min(5.2, wx))
        wy = max(-5.2, min(5.2, wy))
        dwp = math.hypot(wx - rx, wy - ry)
        if dwp < 0.40 and now - getattr(self, '_arc_adv_t', 0.0) > 1.5:
            # 已到当前航点且连线仍被挡 → 航点沿圆弧前推一步
            self._arc_adv_t = now
            arc = min(arc + 0.4, 6.2)
            self._bypass_arc = arc
            at2 = (ac + (math.pi / 2 if d_ro <= R + 0.05 else
                         math.acos(min(1.0, R / d_ro)))) + side * arc
            wx = ox + R * math.cos(at2)
            wy = oy + R * math.sin(at2)
            wx = max(-5.2, min(5.2, wx))
            wy = max(-5.2, min(5.2, wy))
            dwp = math.hypot(wx - rx, wy - ry)
        else:
            self._bypass_arc = arc
        return wx, wy, f'绕障{i}{"左" if side > 0 else "右"}', dwp

    # ---------- 主循环 ----------
    def _tick(self):
        now = self._now()
        # 雷达可用性自检(只做一次): 见 __init__ 里的说明
        if not self._lidar_checked and self.scan_count >= 5:
            self._lidar_checked = True
            r = self._sector_min(-25.0, 25.0)
            self._lidar_ok = r > 0.135
            self.get_logger().info(
                'FOLLOWER 雷达自检: %s (前向 %.2fm)'
                % ('可用, 已叠加到全向安全层' if self._lidar_ok
                   else '不可用(全为量程下限) → 全向避障改用 4 台深度相机', r))
        if self.state != 'FOLLOW':
            self._publish(0.0, 0.0)
            if now - self._last_log > 5.0:
                self._last_log = now
                self.get_logger().info('FOLLOWER 待命: 等待语音指令...')
            return
        with self._lock:
            rx, ry, ryaw = self.rx, self.ry, self.ryaw
            px, py = self.px, self.py
            pspd = self.person_speed
            rspd = self.robot_speed
            nscan = self.scan_count
        if now - self._last_pose_mono > 1.5:
            self._publish(0.0, 0.0)
            self.v = 0.0
            if now - self._last_pose_warn > 3.0:   # P21: 限流, 否则 20Hz 刷屏
                self._last_pose_warn = now
                self.get_logger().warn('FOLLOWER 丢失真值位姿, 已停车')
            return

        band = math.radians(self.cfg['aim_band_deg'])

        # --- 1. 目标估计 (P32: 严格视觉模式) ---
        # 只有"视觉确实检出"才允许驱动跟随。丢失后不再回落到 Gazebo 真值
        # 位姿 —— 那样机器人其实一直知道人在哪, 丢失处理等于没生效, 也无从
        # 验证"按最后 10 张抓拍推测位置"这条链路。allow_gt_track=True 仅供对照。
        vis = self._vis_ok
        lost_dt = (now - self._vis_mono) if self._vis_mono > 0 else 1e9
        vision_ok = (vis is not None and vis.get('dist', -1) > 0
                     and lost_dt <= self.cfg['lost_grace_s'])
        src = '真值'
        if vision_ok:
            vis_cam = str(vis.get('cam', 'front'))
            bearing = float(vis['bearing'])     # 相对机器人前向, + = 左
            # P35: 相机测的是人体表面, 加偏置换算成"人中心到机器人中心"
            vdist = float(vis['dist']) + float(self.cfg['vis_bias_m'])
            if self._track != 'TRACK':
                gap = (now - self._lost_at) if self._lost else 0.0
                self.get_logger().info(
                    'FOLLOWER ★ 重新捕获目标([%s]相机, 偏离中线%+.0f°, %.2fm)%s'
                    ' → 立即对正跟踪'
                    % (vis_cam,
                       math.degrees(float(vis.get('bearing_cam', 0.0))),
                       vdist,
                       (' 丢失%.1fs后恢复' % gap) if self._lost else ''))
                self._track = 'TRACK'
                self._lost = False
                self._lost_announced = False
                self._scan_round = 0
                self._reacq_until = now + 8.0
                self._nav_cancel('重新捕获目标')
            # P47: 时空一致性抑制。人贴着障碍时 bbox 只剩残缺的一小块,
            # 深度取窗还会混进障碍 —— 实测 vis_d 在 0.28~1.74 之间摆 6 倍,
            # 主相机在 left/right 之间来回切, brg 跟着跳到 ±50~120°。
            # 这种跳变一旦进控制环, 就会把机器人打进"对正→原地转"的死循环。
            # 判据: 单帧距离跳 >0.45m 或方位跳 >35°, 视为不可信, 沿用上一帧;
            #       但最多抑制 0.6s, 超时后强制接受(否则人真跑了会被冻结)。
            #       例外: 方位跳但距离基本不变 且 主相机换了 —— 这是真实的
            #       跨相机切换, 必须立刻接受, 否则切换机动性会变差。
            _prev_d = self._fdist
            _close = (_prev_d is not None
                      and _prev_d < float(self.cfg['brg_close_m']))
            if self._fdist is not None and self._fbrg is not None:
                _jd = abs(vdist - self._fdist)
                _jb = abs(wrap(bearing - self._fbrg))
                _cam_chg = (vis_cam != self._fcam)
                _jump = (_jd > 0.45 or _jb > math.radians(35.0))
                # P77 §17-2 修正: 近距离**不信任"真实跨相机切换"这个豁免**。
                # 第一版做法是给方位加 60°/s 的角速率上限, 结果把真实的跨相机
                # 切换也一起拖住了 —— 人真走到侧面时机器人朝错方向继续转 1~2s,
                # 评测直接退化(in_band 40.0→35.2、min d 0.634→0.473、
                # max_deg 21.6→38.0)。改成: 近距离的大跳一律走"冻结"这条路,
                # 但冻结最多 0.6s(_fhold_until), 持续的大跳会被强制接受 ——
                # 单帧尖峰被吃掉, 持续变化只延后 0.6s。
                _real_switch = (not _close
                                and _jb > math.radians(35.0)
                                and _jd < 0.28 and _cam_chg)
                if _jump and not _real_switch and now < self._fhold_until:
                    vdist, bearing = self._fdist, self._fbrg
                    if _close:
                        self._brg_clip_win.append(now)   # 近距离冻结计数
                else:
                    if _jump and not _real_switch:
                        self._fhold_until = now + 0.60
                    # P77 §17-2: 近距离把 EWMA 系数从 0.55 收到 0.35
                    a_f = (float(self.cfg['brg_af_close']) if _close
                           else 0.55)
                    self._fdist = (1 - a_f) * self._fdist + a_f * vdist
                    bearing = wrap((1 - a_f) * self._fbrg + a_f * bearing)
                    vdist = self._fdist
            # P77 §17-2 第一版的"角速率上限"已废弃(见上): 它会把真实的跨相机
            # 切换一起拖住。现在近距离只做两件事 —— ① 关掉 _real_switch 豁免,
            # 大跳走 0.6s 冻结; ② EWMA 系数收到 0.45, 让残余抖动更慢进控制环。
            self._fdist, self._fbrg = vdist, bearing
            self._fbrg_t = now
            self._brg_clip_win = [w for w in self._brg_clip_win if now - w <= 1.0]
            # 视觉 10Hz ⇒ 每秒最多 10 帧, 归一化成 0~1 的"冻结占比"
            self._brg_clip_rate = len(self._brg_clip_win) / 10.0
            self._fcam = vis_cam
            px = rx + math.cos(ryaw + bearing) * vdist
            py = ry + math.sin(ryaw + bearing) * vdist
            self._last_vis_xy = (px, py)      # 只由视觉刷新
            # P40: 由视觉世界坐标差分估计人速(前馈源)。
            # 注意两个坑: ① 跨相机切换时 bearing 会跳变, px/py 瞬间跳一下,
            # 差分出来是几十 m/s 的假峰, 必须按 |inst| 上限剔除;
            # ② 视觉 10Hz, 但 bearing 量化到 ~1°, 1m 处相当 1.7cm 抖动,
            # 所以用 EWMA(0.35) 压, 且 dt 太短/太长都不采样。
            # P73: 窗口最小二乘估人速(见 fit_vel)。跨相机切换会让 px/py
            # 瞬间跳一下, 必须整窗作废, 否则拟合出几十 m/s 的假速度;
            # 视觉中断超过 0.5s 也要作废, 否则新旧样本跨空档被连成一条线。
            if self._pv_cam != vis_cam:
                self._pv_cam = vis_cam
                self._pv_win = []
            if self._pv_win and now - self._pv_win[-1][0] > 0.50:
                self._pv_win = []
            self._pv_win.append((now, px, py))
            while len(self._pv_win) > 1 and \
                    now - self._pv_win[0][0] > 1.20:
                self._pv_win.pop(0)
            if now - self._pv_win[0][0] > 0.35:
                _v = fit_vel(self._pv_win)
                if _v is not None and math.hypot(_v[0], _v[1]) < 2.5:
                    self._pvx, self._pvy = _v
            self._pv_t = now
            # P73: 人速改用"距离变化率 + 本机前向速度"(标量观测)。
            # 原来的 2D 位置差分(EWMA)在 1.5m 处受 1° 方位量化影响,
            # 单轴噪声 ±0.26m/s; 人走到 2.5m 时涨到 ±0.4 —— 实测抓到
            # "人正快速接近(0.83→0.55m)而 ff=+0.50(全速推)"的反号样本,
            # 把机器人一路顶到 0.41m, 人体占满画幅 → 整帧判未检出 →
            # 丢失 → 原地扫描(120s 里 8 次/共 33s)。
            # 距离变化率只需标量, 且 v_person = v_robot + d(dist)/dt 严格
            # 自洽: 人不动时机器人怎么动都算出 0, 人匀速走时算出其真实
            # 径向速度。用原始 dist(不做 P47 抖动抑制, 否则 hold 期间斜率
            # 恒为 0 又变成"人速=0")。
            _rawd = float(vis['dist'])
            if self._rr_win and now - self._rr_win[-1][0] > 0.50:
                self._rr_win = []
            self._rr_win.append((now, _rawd))
            while len(self._rr_win) > 1 and now - self._rr_win[0][0] > 0.70:
                self._rr_win.pop(0)
            _ddt = fit_slope(self._rr_win)
            if _ddt is not None and abs(_ddt) < 2.5:
                self._pvvr = max(-0.60, min(0.85,
                                   self.robot_vfwd + _ddt))
            if vis_cam == 'front':
                if self._vis_prev is not None:
                    dt = now - self._vis_prev[0]
                    if dt > 0.15:
                        dd = (self._vis_prev[1] - vdist) / dt
                        self.vis_pspd = 0.7 * self.vis_pspd + \
                            0.3 * max(0.0, min(1.0, dd))
                        self._vis_prev = (now, vdist)
                else:
                    self._vis_prev = (now, vdist)
            else:
                self._vis_prev = None
            pspd = self.vis_pspd
            src = '视觉' if vis_cam == 'front' else f'视觉-{vis_cam}'
            brg = bearing                       # 相机直接测得的相对方位
        else:
            self._vis_prev = None
            self.vis_pspd = 0.0
            if self.cfg['allow_gt_track']:
                src = '真值'                    # 兼容: 旧的真值兜底行为
            else:
                src = '丢失'
                if self._last_vis_xy is not None:
                    px, py = self._last_vis_xy  # 停在最后确认位置
            brg = wrap(math.atan2(py - ry, px - rx) - ryaw)
            # 判定丢失 → 用最后 10 张抓拍推测人最后所处的位置
            if (self._track in ('TRACK', 'GOTO', 'SCAN')
                    and not self.cfg['allow_gt_track']
                    and lost_dt > self.cfg['lost_grace_s']):
                if not self._lost:
                    self._lost = True
                    self._lost_at = now
                n_hist = min(len(self.hist), 10)
                # P81: 先分清"人跑掉了"还是"人被障碍挡住了"。
                # 两种情况都表现为"没有检出", 但处理完全不同:
                #   跑掉   → 按最后 10 张抓拍推测位置前往(GOTO), 需求如此;
                #   被挡住 → 人一直在原地附近沿障碍走, 机器人视线被同一根
                #           柱子切断, GOTO 到"最后位置"仍在遮挡区里, 只能
                #           靠绕视(PEEK)把视线绕出来。
                _blk = None
                if int(self.cfg.get('peek_on', 1)):
                    # 判据用"推测位置"而不是"最后可见位置": 人走进柱子
                    # 后面时, 最后可见位置还在柱子**前面**(视线通畅), 用它
                    # 判会得出"没被挡"→ 走 GOTO 到柱子前 → 扫一圈还是看
                    # 不见。推测位置是"人现在大概在哪", 才是该判遮挡的点。
                    _tv, _ = self._infer_target()
                    _ck = _tv if _tv is not None else self._last_vis_xy
                    if _ck is not None:
                        _blk = _los_blocked(rx, ry, _ck[0], _ck[1])
                if _blk is not None:
                    self._track = 'PEEK'
                    self._peek_t0 = now
                    self._peek_xy = self._last_vis_xy
                    self._peek_obs = _blk
                    self._peek_vd = self._person_vdir()
                    self._peek_from = self._track
                    self._peek_n += 1
                    # 从 GOTO/SCAN 转入时必须先把 Nav2 停下来, 否则两侧同
                    # 时下发 /cmd_vel —— 实测这一步漏掉会导致"绕视原地不动
                    # + Nav2 还在往最后位置走"的叠加行为。
                    self.pub_nav_active.publish(Bool(data=False))
                    self._nav_cancel('转入绕视')
                    self.get_logger().warn(
                        'FOLLOWER ◐ %.1fs 无检出但视线被%s挡住 → 绕视: '
                        '沿人最后方向(%.2f,%.2f)m/s 同向绕行'
                        % (lost_dt, _blk[0], self._peek_vd[0],
                           self._peek_vd[1]))
                elif self._last_vis_xy is None or n_hist == 0:
                    self.get_logger().warn(
                        'FOLLOWER ✗ 目标丢失(无视觉历史/快照为空) → 原地扫描')
                    self._enter_scan(now, announce=False)
                elif self._scan_round < int(self.cfg['scan_rounds']):
                    tgt, desc = self._infer_target()
                    if tgt is not None:
                        self._scan_round += 1
                        self._search_target = tgt
                        self._search_t0 = now
                        self._goto_best = None
                        self._goto_prog_t = now
                        self._track = 'GOTO'
                        self.get_logger().warn(
                            'FOLLOWER ✗ 目标丢失(%.1fs 无检出), 依据最后 %d 张'
                            '抓拍推测: %s → 前往该位置'
                            % (lost_dt, n_hist, desc))
                        self._nav_send(*self._search_target)
                    else:
                        self._enter_scan(now, announce=False)
                else:
                    self._enter_scan(now, announce=True)

        # --- 2. 搜索/扫描状态机 (P32; P65: GOTO 由 Nav2 执行) ---
        if self._track == 'GOTO':
            gx, gy = self._search_target
            _gd = math.hypot(gx - rx, gy - ry)
            # P65: 导航与绕障/恢复行为全部交给 Nav2; follower 只管
            # 发目标、监听结果、超时兜底和保持 /nav2_active 开关。
            self.pub_nav_active.publish(Bool(data=True))
            # 视觉重捕获 → 取消导航回 TRACK(在上面视觉分支处理)
            if _gd < float(self.cfg['goto_reach_m']) and \
                    self._nav_result_future is None:
                self.get_logger().info(
                    'FOLLOWER 已到达人最后出现位置(%+.1f,%+.1f) 距离%.2fm, '
                    '原地 360° 扫描确认' % (gx, gy, _gd))
                self._nav_cancel('到达目标附近')
                self._enter_scan(now, announce=False)
            elif now - self._search_t0 > float(self.cfg['goto_timeout_s']):
                self.get_logger().warn(
                    'FOLLOWER 前往最后位置超时(%.0fs, 还差%.2fm), 转原地扫描'
                    % (float(self.cfg['goto_timeout_s']), _gd))
                self._nav_cancel('搜索超时')
                self._enter_scan(now, announce=False)
            elif (self._nav_goal_xy is not None
                  and math.hypot(self._nav_goal_xy[0] - gx,
                                 self._nav_goal_xy[1] - gy) > 0.3):
                # 目标点更新(新一轮推测) → 重发目标
                self._nav_cancel('目标点更新')
                self._nav_send(gx, gy)
            px, py = rx, ry          # 不参与内部速度律, 由 Nav2 驱动
            src = '搜索'
        elif self._track == 'PEEK':
            # P81: 遮挡期绕视 —— 不判定"目标丢失", 而是跟着人绕出遮挡区。
            # 退出只认两件事: ① 重新检出(视觉分支直接把状态拉回 TRACK);
            # ② 超时(交回 GOTO, 按需求"前往最后出现的位置")。
            _lx, _ly = self._peek_xy
            if (now - self._peek_t0) > float(self.cfg.get('peek_dur_s', 3.0)):
                self.get_logger().info(
                    'FOLLOWER 绕视 %.1fs 仍无检出 → 交回常规丢失处理'
                    % (now - self._peek_t0))
                if self._peek_from == 'SCAN':
                    self._enter_scan(now, announce=False)
                else:
                    self._track = 'TRACK'   # 下一拍重新判定 → GOTO/SCAN
            # 遮挡期间人还在走: 用进入时的速度线性外推"人在哪儿"
            _dtk = min(now - self._peek_t0, 4.0)
            px = _lx + self._peek_vd[0] * _dtk
            py = _ly + self._peek_vd[1] * _dtk
            src = '绕视'
        elif self._track == 'SCAN':
            if now > self._scan_until:
                tgt = None
                desc = ''
                if self._scan_round < int(self.cfg['scan_rounds']) and self.hist:
                    tgt, desc = self._infer_target()
                if tgt is not None:
                    self._scan_round += 1
                    self._search_target = tgt
                    self._search_t0 = now
                    self._goto_best = None
                    self._goto_prog_t = now
                    self._track = 'GOTO'
                    self.get_logger().info(
                        'FOLLOWER 扫描无果, 按快照再次推测 %s' % desc)
                    self._nav_send(*self._search_target)
                else:
                    self._enter_scan(now, announce=True)
            # P81: 原地扫描"扫不到"时也要动起来。
            # 实测"人躲到柱子后面": 机器人走到最后可见位置 → 原地转圈 20s,
            # 4 台相机被同一根柱子挡着, 永远扫不到人(人就在 2.8m 外)。
            # 转够 scan_seek_s 还没见着, 就沿法向横移一段把视线让出来,
            # 而不是继续空转。
            elif (int(self.cfg.get('peek_on', 1))
                  and (now - getattr(self, '_scan_t0', now))
                  > float(self.cfg.get('scan_seek_s', 3.0))):
                _tv, _ = self._infer_target()
                _ck = _tv if _tv is not None else self._last_vis_xy
                if _ck is not None:
                    self._track = 'PEEK'
                    self._peek_t0 = now
                    self._peek_xy = _ck
                    self._peek_obs = _los_blocked(rx, ry, _ck[0], _ck[1])
                    self._peek_vd = self._person_vdir()
                    self._peek_from = 'SCAN'
                    self._peek_n += 1
                    self.get_logger().warn(
                        'FOLLOWER ◐ 原地扫描 %.1fs 无检出(视线%s) → 绕视横移'
                        % (now - self._scan_t0,
                           '被%s挡住' % self._peek_obs[0]
                           if self._peek_obs else '通畅'))
            src = '扫描'

        if src != self._src_prev:
            if '真值' in (src, self._src_prev) or '视觉' in (src, self._src_prev):
                self.get_logger().info(f'FOLLOWER 感知源切换 → {src}')
            self._src_prev = src

        dist = math.hypot(px - rx, py - ry)

        # --- 3. 对正判定(P20): 目标是否落在 front 中线 ±band 内 ---
        # P77 §17-4: 进/退对正都加**时间滞回**。
        # 只看角度阈值时, brg 在 band 边界上 ±1~3° 的噪声就能让
        # ALIGN↔TRACK 每 0.5~1s 翻一次 —— 受控场景实测 mode 切换 56 次/75s,
        # 每次切换 w 都要从 0 拉到 ±0.35 再回 0, 这正是"w 换向 0.80 次/s"
        # 的主要来源(而不是之前猜的 SCAN↔ALIGN 对打: 那次 owner 统计里
        # align 占 100%, 根本没进过 SCAN)。现在要求越界/回带**持续**
        # align_enter_s / align_exit_s 才切换, 单帧噪声切不动模式。
        aim = abs(brg)
        if self._aligning:
            _want_align, _thr, _need = False, band * 0.65, \
                float(self.cfg['align_exit_s'])
        else:
            _want_align, _thr, _need = True, band, \
                float(self.cfg['align_enter_s'])
        _cross = (aim > _thr) if _want_align else (aim <= _thr)
        if _cross:
            if self._align_edge_t <= 0.0:
                self._align_edge_t = now
            if (now - self._align_edge_t) >= _need:
                self._align_edge_t = 0.0
                if _want_align:
                    self._aligning = True
                    self.get_logger().info(
                        f'FOLLOWER ⟳ 目标偏离中线 {math.degrees(brg):+.1f}°'
                        f' > {self.cfg["aim_band_deg"]:.0f}°, 先原地对正')
                else:
                    self._aligning = False
                    self.get_logger().info(
                        f'FOLLOWER ✓ 已对正目标(偏差 {math.degrees(brg):+.1f}°'
                        f' ≤ {self.cfg["aim_band_deg"]:.0f}°), 转入距离跟踪')
        else:
            self._align_edge_t = 0.0
        # 扫描态固定小角速度旋转
        if src == '扫描':
            self._aligning = False

        v_t = 0.0
        w_t = 0.0
        note = ''

        # --- 3.5 绕障抢占(P46/P47): 锁存式 Bug2 ---
        # 历史: P39 关航点(切点在临界角左右翻 → 死锁), P43b 关切线(侧向
        # 余量不足时 v=0, 只剩旋转 → 原地打转)。两次都是"启发式不可靠",
        # 但**完全不绕**的代价更大: 目标走到障碍后面时, 车头一朝向目标
        # 就被安全层急停, 机器人原地抖(实测 28s 只挪 0.3m,
        # w 在 +1.50/-1.15 每 0.5s 翻一次)。
        # 这一版的区别: **不在每个 tick 重算切点**。
        #   ① 触发后锁存绕行侧别与"进入时的目标距离 d0";
        #   ② 绕行期间车头恒定偏转 avoid_deg 朝向侧方 —— 这样前向量到的
        #      是侧方余量, 不会因为"目标在障碍后"而持续急停;
        #   ③ 退出条件用 Bug2 判据: 前方畅通 **且** 比进入时更接近目标;
        #   ④ 有 avoid_max_s 超时换边, 不会永久卡在错误侧。
        _dn = ''
        _pv8 = self.prox or {}
        _age8 = float(_pv8.get('age', 99.0) or 99.0)

        def _q(k):
            v = _pv8.get(k, -1.0)
            return v if (v and v > 0) else 99.0
        _df, _dfl, _dfr = _q('front'), _q('fl'), _q('fr')
        _dl, _dr = _q('left'), _q('right')
        _room_l = min(_dl, _dfl)
        _room_r = min(_dr, _dfr)
        _stop = float(self.cfg['lidar_stop_m'])
        _avoid_now = False

        def _dir_min(b):
            """P48: 方位角 b(相对车头, + = 左)方向上的最小余量。

            8 扇区中心是 0/±45/±90/±135/180, 取包围该角度的两个扇区的
            较小值(保守)。用于判断"目标方向是否已经通了" —— 通了就说明
            绕障可以把车头转回目标, 不必再用 dist 判据(绕行必然先远离)。
            """
            dd = (math.degrees(b) + 360.0) % 360.0
            tab = [(0.0, _q('front')), (45.0, _q('fl')),
                   (90.0, _q('left')), (135.0, _q('bl')),
                   (180.0, _q('back')), (225.0, _q('br')),
                   (270.0, _q('right')), (315.0, _q('fr')),
                   (360.0, _q('front'))]
            best = 99.0
            for i in range(len(tab) - 1):
                a0, v0 = tab[i]
                a1, v1 = tab[i + 1]
                if a0 - 22.5 <= dd <= a1 + 22.5:
                    best = min(best, v0, v1)
            return best

        if self.cfg['use_avoid'] and _age8 < 0.5:
            _blk = (_df < _stop * 1.55
                    or min(_dfl, _dfr) < _stop * 1.10) and dist > 1.20
            if not self._avoiding:
                if _blk:
                    if self._blk2_t0 <= 0.0:
                        self._blk2_t0 = now
                    elif now - self._blk2_t0 > float(self.cfg['avoid_trig_s']):
                        self._avoiding = True
                        self._avoid_t0 = now
                        self._avoid_d0 = dist
                        self._avoid_side = 1.0 if _room_l >= _room_r else -1.0
                        # P49: 锁存世界系绕行航向。
                        # 目标已在侧方 → 直接朝目标(它不在被挡的方向上,
                        #   朝它走本身就是在绕); 目标在正前方被挡 → 车头
                        #   偏转到余量更大的一侧。
                        if abs(brg) > math.radians(15.0):
                            self._avoid_hdg = wrap(ryaw + brg)
                            self._avoid_side = 1.0 if brg > 0 else -1.0
                        else:
                            self._avoid_hdg = wrap(
                                ryaw + self._avoid_side
                                * math.radians(float(self.cfg['avoid_deg'])))
                        self._avoid_hdg_t = now
                        self._blk2_t0 = 0.0
                        self.get_logger().info(
                            'FOLLOWER ⟳ 绕障介入(%s绕): 前%.2f 左前%.2f '
                            '右前%.2f 目标%.2fm → 车头偏转%.0f°'
                            % ('左' if self._avoid_side > 0 else '右',
                               _df, _dfl, _dfr, dist,
                               float(self.cfg['avoid_deg'])))
                else:
                    self._blk2_t0 = 0.0
            if self._avoiding:
                # P48: 退出判据改成"目标方向已经通了"。原来用
                # `dist < _avoid_d0 - 0.05` —— 但绕行的第一步必然是
                # 先远离目标(往侧方走), 这个条件几乎永远不成立, 只能等
                # avoid_max_s 超时换边, 表现为"绕 9s 没效果再换边绕"。
                _clear_goal = (_dir_min(brg)
                               > float(self.cfg['lidar_slow_m']) * 0.80)
                _tmo = now - self._avoid_t0 > float(self.cfg['avoid_max_s'])
                if _clear_goal or _tmo:
                    self._avoiding = False
                    self._vfh_w = None
                    self._blk2_t0 = 0.0
                    self.get_logger().info(
                        'FOLLOWER ✓ 绕障退出(历时%.1fs, %s, 目标%.2fm) → 恢复指向'
                        % (now - self._avoid_t0,
                           '超时换边' if _tmo else '目标方向已畅通', dist))
                else:
                    _avoid_now = True

        if _avoid_now:
            _side_room = _room_l if self._avoid_side > 0 else _room_r
            # P52: VFH + 0.4s 世界系锁存。
            # 纯 VFH(P51)会正反馈振荡: 扇区表锚定在**车头系**, 车头左转
            # 后目标方位同步减少, 而"最优扇区"仍是同一个, 误差恒为正 →
            # w 恒为正 → 绕圈(实测位移 0.70m, dist 却在 1.48~2.26 震荡,
            # 从未接近过目标)。
            # 现在: 选定后转成世界角锁存 0.4s, 期间按该世界角收敛(误差到
            # 0 即停), 0.4s 后重选 —— 既断开正反馈, 又能在走偏后修正
            # (P49 只在进入时锁一次, 走偏了不修正)。
            _pass_m = max(0.95, _stop + 0.30)
            if self._vfh_w is None or now - self._vfh_t > 0.40:
                _goal_d = (math.degrees(brg) + 360.0) % 360.0
                _tab8 = [(0.0, 'front'), (45.0, 'fl'), (90.0, 'left'),
                         (135.0, 'bl'), (180.0, 'back'), (225.0, 'br'),
                         (270.0, 'right'), (315.0, 'fr')]
                _vb = None
                for _a, _k in _tab8:
                    _vv = _q(_k)
                    if _vv < _pass_m:
                        continue
                    _c = abs(((_a - _goal_d + 180.0) % 360.0) - 180.0)
                    _as = ((_a + 180.0) % 360.0) - 180.0
                    if abs(_as) > 110.0:      # 身后方向: 加惩罚, 别掉头
                        _c += 45.0
                    if _vb is None or _c < _vb[0]:
                        _vb = (_c, _a, _vv)
                self._vfh_t = now
                if _vb is not None:
                    self._vfh_w = wrap(ryaw + math.radians(_vb[1]))
                    self._vfh_hint = ('扇区%+.0f° 余%.2f'
                                      % (_vb[1], _vb[2]))
                else:
                    self._vfh_w = None
                    self._vfh_hint = '无可行扇区'
            if self._vfh_w is not None:
                _err_hdg = wrap(self._vfh_w - ryaw)
                w_t = max(-1.6, min(1.6, 2.2 * _err_hdg))
                v_t = min(float(self.cfg['approach_speed']) * 0.72, 0.38,
                          max(float(self.cfg['avoid_min_v']),
                              (_df - _stop + 0.12) / 0.45))
                _hint = self._vfh_hint
            else:
                # 四面都不够宽 → 原地慢转找缺口
                w_t = 1.0
                v_t = 0.0
                _hint = self._vfh_hint
            # P77 §17-4: 绕障是最高优先级 owner, 可立即抢占转向
            w_t = self._arb_w(w_t, 'avoid', now)
            _dn = ('绕障[%s] 前%.2f 侧%.2f 目标%.2f v*%.2f '
                   '左%.2f右%.2f 左前%.2f右前%.2f 后%.2f'
                   % (_hint, _df, _side_room, dist, v_t, _dl, _dr,
                      _dfl, _dfr, _q('back')))
            # 绕障接管必须绕过 ALIGN —— 否则 brg 一跳出 band 就进"原地
            # 对正"并 return, 机器人只会原地转(实测 30s 只挪 0.23m,
            # w 在 ±1.5 之间每 1s 翻号)。
            self._aligning = False
            v_t, w_t, _sn = self._safety_gate(v_t, w_t, now)
            if _sn:
                _dn += ' |' + _sn
            self._publish_smooth(v_t, w_t, now)
            self._log(now, src, dist, pspd, rspd, w_t, brg, _dn, nscan,
                      rx, ry, px, py)
            return

        if src == '绕视':
            # P81: 遮挡期动作 —— 沿"机器人→人"的**切向**跟着人绕, 而不是
            # 朝人直冲(直冲会撞上挡住视线的柱子, 也会被安全层急停)。
            #   · 切向方向取"与人最后运动方向一致"的那一侧 —— 绕柱时人沿
            #     圆弧走, 机器人同向绕才能把视线绕出来; 反向绕会越离越远
            #     (实测第一版按"远离障碍"选侧: 检出率反而从 73.7% 掉到
            #     53.6%, 距离中位从 1.55m 涨到 2.10m、最远 4.34m)。
            #   · 人速不可信(<0.12m/s)时退化为"远离障碍的一侧"。
            #   · 加径向修正: 远了朝人靠、近了往外让, 绕行不丢距离带。
            _lx, _ly = self._peek_xy
            _dtk = min(now - self._peek_t0, 4.0)
            _qx = _lx + self._peek_vd[0] * _dtk
            _qy = _ly + self._peek_vd[1] * _dtk
            _ex, _ey = _qx - rx, _qy - ry
            _nn = math.hypot(_ex, _ey) or 1e-6
            _ex, _ey = _ex / _nn, _ey / _nn
            _t1x, _t1y = -_ey, _ex                  # 视线法向(左)
            _vd = self._peek_vd
            if math.hypot(_vd[0], _vd[1]) > 0.12:
                _s = 1.0 if (_t1x * _vd[0] + _t1y * _vd[1]) > 0.0 else -1.0
            elif self._peek_obs is not None:
                _ox, _oy = self._peek_obs[1], self._peek_obs[2]
                _s = -1.0 if (_t1x * (_ox - rx) + _t1y * (_oy - ry)) > 0.0 \
                    else 1.0
            else:
                # 既没有可信人速、又没有挡住视线的障碍(典型: 人原地不动
                # 藏在柱子后): 左右交替横移, 保证两轮内一定试过两侧。
                _s = 1.0 if (self._peek_n % 2) == 1 else -1.0
            _tx, _ty = _s * _t1x, _s * _t1y
            _k = max(-0.6, min(0.9, 0.8 * (_nn - self.target_d)))
            _hdg = wrap(math.atan2(_ty + _ey * _k, _tx + _ex * _k) - ryaw)
            # 期望方向不一定走得通 —— 侧移方向可能正对着挡住视线的那根柱子,
            # 安全层会把 v 掐成 0, 结果"绕视"变成原地打转(实测 6s 绕视里
            # 位移 0.00m, dh 一直 0.52~1.19m 被急停)。所以先用 8 扇区测距
            # 在期望方向 ±105° 内找一个**真的能走**的航向, 优先靠近期望值。
            _pv9 = self.prox or {}

            def _q9(kk):
                v = _pv9.get(kk, -1.0)
                return v if (v and v > 0) else 99.0
            _tab9 = [(0.0, _q9('front')), (45.0, _q9('fl')),
                     (90.0, _q9('left')), (135.0, _q9('bl')),
                     (180.0, _q9('back')), (225.0, _q9('br')),
                     (270.0, _q9('right')), (315.0, _q9('fr')),
                     (360.0, _q9('front'))]

            def _dmin9(b):
                dd = (math.degrees(b) + 360.0) % 360.0
                best = 99.0
                for i in range(len(_tab9) - 1):
                    a0, v0 = _tab9[i]
                    a1, v1 = _tab9[i + 1]
                    if a0 - 22.5 <= dd <= a1 + 22.5:
                        best = min(best, v0, v1)
                return best
            _pass = float(self.cfg.get('peek_pass_m', 0.60))
            for _off in (0, 15, -15, 30, -30, 45, -45, 60, -60, 75, -75,
                         90, -90, 105, -105):
                _cand = wrap(_hdg + math.radians(_off))
                if _dmin9(_cand) >= _pass:
                    _hdg = _cand
                    break
            w_t = max(-1.3, min(1.3,
                                float(self.cfg.get('peek_kp', 2.2)) * _hdg))
            v_t = float(self.cfg.get('peek_v', 0.35)) \
                * max(0.0, math.cos(_hdg)) ** 2
            w_t = self._arb_w(w_t, 'peek', now)
            v_t, w_t, _sn = self._safety_gate(v_t, w_t, now)
            self._publish_smooth(v_t, w_t, now)
            self._log(now, src, dist, pspd, rspd, w_t, brg,
                      '绕视(%s) 侧移航向%+.0f°' %
                      ('视线被%s挡住' % self._peek_obs[0]
                       if self._peek_obs else '扫描无果横移',
                       math.degrees(_hdg)),
                      nscan, rx, ry, px, py)
            return

        if src == '扫描':
            # P76: 原来是写死的 `w_t = recenter_rot_speed * 0.8`(即 +1.20,
            # 永远向左扫)。而 ALIGN 是按 brg 符号选向的。目标落在正后方
            # (|brg|≈145°~180°)时两者方向恰好相反: SCAN 往左、ALIGN 往右,
            # 于是 w 在 +1.20 / -1.50 之间每 1~3s 翻一次, 实测原地摇摆 45s
            # 也没转到人 —— 这就是"来回摆动"的直接原因。
            # 现在 SCAN 也按最后已知方位选向, 并与 ALIGN 保持同向。
            _sd = getattr(self, '_scan_dir', 0.0)
            if _sd == 0.0 or now - getattr(self, '_scan_dir_t', 0.0) > 8.0:
                _sd = 1.0 if (brg is None or brg >= 0.0) else -1.0
                self._scan_dir = _sd
                self._scan_dir_t = now
            w_t = _sd * self.cfg['recenter_rot_speed'] * 0.8
            # P77 §17-4: 扫描的转向也必须过仲裁 —— 不能与 ALIGN 各转各的
            w_t = self._arb_w(w_t, 'scan', now)
            v_t, w_t, _sn = self._safety_gate(v_t, w_t, now)
            self._publish_smooth(v_t, w_t, now)
            self._log(now, src, dist, pspd, rspd, w_t, brg, '原地扫描', nscan,
                      rx, ry, px, py)
            return

        if self._aligning:
            # ---- ALIGN: 原地纯旋转, 比例 + 软死区, 方向锁存防 ±180° 抖动 ----
            sgn = 1.0 if brg >= 0 else -1.0
            if aim > 2.1:
                if self._align_dir == 0.0:
                    self._align_dir = sgn
                sgn = self._align_dir
            elif aim < 1.05:
                self._align_dir = 0.0
            w_mag = self.cfg['recenter_kp'] * (aim - band * 0.5)
            w_mag = max(self.cfg['turn_min_w'],
                        min(self.cfg['recenter_rot_speed'], w_mag))
            w_t = sgn * w_mag
            # P77 §17-4: 对正转向过仲裁(与扫描共用同一个方向记忆)
            if now < self._detour_until:
                # 定向绕行窗口内转向权归 'detour' —— 对正只能等。
                # 这是"最短保持时间"的具体落地: 没有它, 绕行刚把车头掰开
                # 一帧, 对正下一帧就掰回来, 依旧原地打摆。
                w_t = self._arb_w(float(self.cfg['detour_w'])
                                  * self._detour_dir, 'detour', now)
            else:
                w_t = self._arb_w(w_t, 'align', now)
            # P22: 偏差不大(≤34°)时允许"边走边转"。纯原地旋转 v=0 会让逃跑的
            # 目标越拉越远, 实测 90s 内进出对正 44 次 —— 每次进对正就停车,
            # 目标跑开 → 又进对正, 形成"走走停停追不上"。大偏差(目标在左右/
            # 后相机)仍保持纯旋转, 因为那时前方是空的, 边走会撞。
            if aim < math.radians(34.0):
                # P36: 0.60 → 0.85。对正段占了追踪时间的一半以上,
                # 0.6 倍速度让机器人在"边转边走"时明显掉队。
                # P76: 但"边转边走"必须看距离 —— 原来无条件给
                # 0.85*approach(0.44m/s), 人在 0.6~1.0m 时照样全速往前顶,
                # 把人顶进 0.60m 以内。此时人体占满 front 画幅, 方位估计
                # 失效(diag 实测 brg 在 ±130°/±40° 之间乱跳), 对正与扫描
                # 交替翻向 → w 在 ±1.2~1.5 之间每 1~3s 翻一次, 原地摇摆。
                # 已经比目标距离近 0.15m 以上时, 只准转不准进。
                # P79: 门限跟着区间下限走(0.50m)。带宽 0.5 之后, 0.55~0.85m
                # 是**合法**跟随距离, 原来按 target_d-0.15=0.85 就"只转不准进"
                # 会把它当越界处理 → 不停后退。
                _cb = float(self.cfg['close_back_m'])
                if dist is not None and 0.0 < dist < self.target_d - _cb:
                    # P78: 光"停"不够 —— 人继续朝机器人走会把距离压到
                    # 0.4m 以内, 人体占满 front 画幅后直接丢失目标。
                    # 实测 t25: 74s 起 d=0.68→0.41, cmd_v 恒 0, 之后
                    # SCAN 45s 才重新找到人。
                    # 只在**真正危险区**(dist < 区间下限)才后退, 且必须先确认
                    # 后方净空。
                    v_t = 0.0
                    if int(self.cfg.get('close_back_on', 0)) and \
                            dist < self.target_d - _cb:
                        _dbk = self.prox.get('back', -1.0) if self.prox else -1.0
                        if _dbk is None or _dbk < 0.0:
                            _dbk = 9.9        # 无数据 → 按"后方开阔"处理
                        if _dbk > self.cfg['lidar_stop_m'] + 0.30:
                            # 比例让距: 刚越线时只退一点点(0.10m/s), 越近退得
                            # 越快(上限 0.28)。固定 -0.22 会把 0.84m 也一路退到
                            # 1.2m 以上, 把 far 抬高 3.2pt(t26 实测)。
                            _gap = (self.target_d - _cb) - float(dist)
                            _vb = float(self.cfg['close_back_v']) + \
                                float(self.cfg['close_back_k']) * max(0.0, _gap)
                            v_t = -min(0.28, _vb)
                elif int(self.cfg.get('back_hold_on', 1)) and \
                        float(dist) < float(self.cfg.get('back_hold_m', 0.85)) \
                        and lost_dt < 0.25:
                    # P80: 对正时**不是**无脑"边转边走"。原来只要
                    # dist >= target_d-close_back_m(=0.50) 就以 0.22m/s 前推,
                    # 而检测器在真值 0.70m 就看不见人 —— 于是"人已到 0.65m、
                    # 视觉刚丢"的那 1s 里机器人还在往前顶(t32 逐帧:
                    # 0.62→0.41m, v 恒 +0.22, note 一直是"对正+8°边转边走")。
                    # 现在 < back_hold_m 时改为让距/原地转, 不再前进。
                    _dbk3 = self.prox.get('back', -1.0) if self.prox else -1.0
                    if _dbk3 is None or _dbk3 < 0.0:
                        _dbk3 = 9.9
                    if _dbk3 > self.cfg['lidar_stop_m'] + 0.25:
                        _cl3 = max(0.0, -self._pvvr)
                        _vb3 = (float(self.cfg.get('back_hold_gain', 1.0)) * _cl3
                                + float(self.cfg.get('back_hold_k', 0.7))
                                * (float(self.cfg['back_hold_m']) - float(dist)))
                        v_t = -min(float(self.cfg.get('back_hold_vmax', 0.45)),
                                   max(float(self.cfg.get('back_hold_vmin', 0.06)),
                                       _vb3))
                    else:
                        v_t = 0.0
                elif float(dist) < self.target_d:
                    # P80 第二次修正: 已经比目标距离近时, 对正就**只转不走**。
                    # 上一版只把 <0.85m 改成让距, 但 0.85~1.0 这段仍然"边转
                    # 边走"(v=+0.22~0.28)。而视觉距离估计在相邻两帧之间会
                    # 跳 0.10~0.15m(1° 方位量化 + 检测框抖动), 阈值 0.85 正好
                    # 卡在噪声带上 —— t35 实测: 真值 0.90→0.68 的 0.6s 里一直
                    # 走"边转边走", 等 dist 掉到 0.85 以下才开始让距, 已经
                    # 顶到 0.68m, 随后人继续走近 → 0.41m 接触。
                    # ALIGN 的职责本来就是"转向", 距离不够才需要顺带挪一段;
                    # 已经比目标近了还往前挪, 与距离环的意图直接矛盾。
                    v_t = 0.0
                else:
                    # P79: 边转边走也吃同一个减速包络, 否则"对正"这一路会以
                    # 0.53m/s 直接顶到人身上再靠硬下界剁停。
                    v_t = 0.85 * self.cfg['approach_speed'] * self._slow_k(dist)
            else:
                v_t = 0.0
            note = (f'对正{math.degrees(brg):+.0f}°'
                    f'{"(边转边走)" if v_t > 0 else ""}'
                    f'{"(后退让距)" if v_t < 0 else ""}'
                    f'{"(方向锁定)" if self._align_dir != 0 else ""}')
            v_t, w_t, _sn = self._safety_gate(v_t, w_t, now, dist, brg)
            if _sn:
                note += ' ' + _sn
            self._publish_smooth(v_t, w_t, now)
            self._log(now, src, dist, pspd, rspd, w_t, brg, note, nscan,
                      rx, ry, px, py)
            return

        # ---- TRACK: 已对正 → 距离环决定前进/后退/保持 ----
        # P32: GOTO(前往人最后位置)态的目标是"到点", 不是"保持跟随距离"。
        # 若沿用 target_d=1.0 / guard_d=0.65, 机器人会在离目标点 1.0m 处就
        # 停住, 永远进不了 0.5m 的到点判定 —— 原实现里"先移动到人最后所处
        # 的位置"这一步其实走不完, 只能等超时。
        tok = self.cfg['track_dist_tol_m']
        if src == '搜索':
            # P65: GOTO 由 Nav2 驱动(cmd_mux 择路), follower 自身速度清零,
            # 只保留安全层监督职责。到点/超时判定在上面状态机完成。
            err_d = 0.0
            guard_d = 0.30
            self._int_d = 0.0
            self._publish(0.0, 0.0)
            return
        else:
            err_d = dist - self.target_d
            # 安全距(低于此值必须后退), 下面限速也要用。
            # P79: 不再固定 0.55, 跟着区间下限走 —— 区间放宽到 0.5~1.5 后,
            # guard_d 必须 <= 0.5+0.05, 否则"合法区内就判越界"。
            guard_d = max(float(self.cfg['slow_floor_m']) + 0.05,
                          self.target_d - tok - 0.25)
        tol = tok
        dtc = 0.05
        # P40: 人速前馈 —— 把人沿车头方向的速度分量直接叠到速度指令上。
        # 这样"跟着走"由前馈提供, "距离对不对"由误差环修正, 两者解耦;
        # 否则 kp 小则稳态差 0.5m(kp=0.6 时的实测), kp 大则过冲抖。
        # P73: 前馈源改为距离变化率观测(见视觉分支的说明)。
        _vfwd = self._pvvr
        _vg = float(self.cfg['vel_ff_gain'])
        _vmax = float(self.cfg['vel_ff_max'])
        v_ff = max(-0.30, min(_vmax, _vg * _vfwd))
        if lost_dt > 0.6:                     # 视觉陈旧 → 前馈立刻撤掉
            v_ff = 0.0
        # P22 修正: 原式 v_cap = max(approach_speed, pspd+0.25) 在目标快跑时
        # 会把上限抬到 0.9 m/s 以上。而视觉距离只有 5 Hz、且允许 0.8s 陈旧,
        # 0.6 m/s 意味着最多 0.5m 的"盲走" —— 实测直冲 0.58m(安全距 0.65),
        # 差点撞人。这里改成"接近速度"与"视觉时延内不冲进安全距"取小:
        #   v_lat = (dist - guard_d) / 视觉时延
        # 目标 1.5m 时 v_lat≈1.06(不限制), 1.0m 时 0.44, 近了自然收。
        # P40: 这段提到带内/带外的分叉之前 —— 带内驻留也要受同一个限速约束,
        # 否则"人走近时前馈仍按人速推车"会顶进安全距。
        lat = 0.8
        room = max(0.02, dist - guard_d)
        # P54: 原式 `min(max(approach_speed, v_ff), v_lat)` 把**抗过冲的
        # 安全帽当成了稳态限速**。v_lat = (dist-guard_d)/lat 在稳态下恰好
        # 等于人速时达到平衡 → 跟距被钉在 `guard_d + lat*v_p`:
        #   guard_d=0.65, lat=0.8, 人速 0.2~0.5 → 稳态距离 0.81~1.05m,
        #   实测 eval15 中位 0.908m、near_pct 48.9%、in_band 只剩 15.2%。
        # 意图其实是"视觉时延内不许冲进安全距", 也就是限制**相对接近率**
        # 而不是绝对速度。所以帽应取 `人速 + 剩余空间/时延`:
        #   v - v_p = room/lat  → lat 秒内正好走完 room, 不越界;
        # 人速为 0 时退化成原式(仍在 lat 秒内收敛到 guard_d)。
        v_lat = v_ff + room / lat
        v_cap = max(0.0, min(0.72, v_lat))
        # P79: 接近减速区。用户要求"人距 <2m 就必须开始减速, 避免撞上和
        # 频繁后退"。原来只有上面那条 v_lat —— 它在 2.0m 处 ≈1.8, 完全
        # 不起作用, 等于"看见人还在全速冲", 冲到 0.8m 再由硬下界剁成 0,
        # 然后距离环反向拉回: 这就是"撞上 + 频繁后退"的组合。
        # 现在叠一条**单调**的速度包络:
        #   d >= 2.0m      : 不减速(全速接近)
        #   0.5 < d < 2.0  : 乘 ((d-0.5)/1.5)^0.6, 单调收油
        #   d <= 0.5m      : 0 (区间下限, 不能再进)
        # 指数 0.6 让 2.0→1.5m 段降得缓(这时还得跟得上人), 1.0→0.5m 段
        # 骤降(56% → 8%), 真正危险的那 0.5m 里速度已经很慢。
        _sf = float(self.cfg['slow_floor_m'])
        self._slow_fac = self._slow_k(dist)
        if self._slow_fac < 0.999:
            v_cap *= self._slow_fac
            if now - getattr(self, '_last_slow_log', 0.0) > 3.0:
                self._last_slow_log = now
                self.get_logger().info(
                    f'FOLLOWER 接近减速 人{dist:.2f}m '
                    f'速度上限 {v_cap:.2f}m/s(系数{self._slow_fac:.2f})')
        # P73/P79: 近距离硬下界。人体近距离会把画面占满(W_REAL 的 55%
        # 容差直接失效), 挤进去就整帧判未检出 → 丢失 → 扫描。P79 起这条
        # 线跟着区间下限走(=0.52m), 不再写死 0.82 —— 0.82 在 0.5~1.5 的
        # 新口径里是合法区间中部, 在那儿剁成 0 会让人走进 0.5 内丢目标。
        if dist < _sf + 0.02:
            v_cap = 0.0
        # P42: 原来这里是"带内/带外"两个分支, 误差一穿过 ±tol, 指令就从
        # 纯前馈(v_ff≈0.03)跳到 PID 输出(≈0.40) —— 实测 1.07m 时 0.03、
        # 1.14m 时 0.39, 差 0.07m 指令就差 13 倍, 机器人一脚油门一脚刹车,
        # 距离在 1.06~1.15 之间抖, 永远 settle 不进 0.9~1.1。
        # 现在合成一个连续回路: 前馈负责"跟着人走", PID 负责"距离对不对",
        # 两者始终相加。人匀速走且距离达标时 err≈0 → v=v_ff, 正是驻留行为,
        # 完全不需要按 tol 分叉。只留 ±0.02m 窄死区吸收测距噪声。
        vtol = 0.02
        if abs(err_d) < vtol:
            err_d = 0.0
        # P79: 带宽 0.1→0.5 之后, 带边缘的 |err_d| 从 0.1 变成 0.5, 同一
        # kp(1.9) 会把反馈打到 ±0.45 的饱和平顶 —— 距离刚进 0.55m 就一脚
        # 倒车、刚出 1.45m 就一脚地板油, 正是"频繁后退"的另一半来源。
        # 软死区: 带内增益乘 inband_fb(0.4), 出带连续接回原增益(斜率为 1),
        # 折点处值相等(tol*ins), 不产生指令跳变。
        #   dist=0.85(旧版早就后退的位置) → err -0.15 → -0.06 → fb -0.11
        #   dist=0.40(真越界)             → err -0.60 → -0.30 → fb -0.57(钳)
        _ins = float(self.cfg['inband_fb'])
        if tol > 1e-6 and _ins < 1.0:
            if abs(err_d) <= tol:
                err_d *= _ins
            else:
                err_d = math.copysign(abs(err_d) - tol * (1.0 - _ins), err_d)
        # P43: 反馈总幅度必须钳住。前馈已经负责人速, 反馈只修残差;
        # 原来积分钳 ±0.9, 它单独就能下一脚 0.9m/s 的油门 —— 实测带内
        # (0.9~1.15m)指令在 -0.16~+0.53 之间摆, 极差 0.69。
        self._int_d = max(-0.15, min(0.15,
                           self._int_d + self.cfg['pid_ki_v'] * err_d * dtc))
        der_d = (err_d - self._prev_err_d) / dtc
        self._prev_err_d = err_d
        _fb = (self.cfg['pid_kp_v'] * err_d + self._int_d
               + self.cfg['pid_kd_v'] * der_d)
        _fb = max(-0.45, min(0.45, _fb))
        v_pid = v_ff + _fb
        # P56: 后退能力必须够。人要是一转身朝机器人走过来(随机行走里每几秒
        # 就有一次), 机器人只有 0.35m/s 的后退速度, 而人前进 0.4m/s ——
        # 净接近 0.05m/s, 距离被一点点啃到 0.6~0.8m 就再也回不来:
        # 实测 eval18 near_pct 60.6%、中位 0.815m, 控制器一直想退但退不动。
        # 按"差多少退多快"给上限, 只在明显偏近时才放开(后方安全由
        # _safety_gate 的 d_b 判据保证)。
        _vback = -0.55 if err_d < -0.12 else -0.35
        v_t = max(_vback, min(v_cap, v_pid))
        # P79: 后退闸门 —— 只在"接近区间下限"时才允许后退。
        # 这是实测出来的关键修正: 原来前馈把人朝机器人走来的速度直接做成
        # -0.30, 加 PID 后 v_t 变负。t24 实测人距 1.0~1.1m(**带中心**)时仍有
        # 41.7% 的帧在下发后退、40 次/100s —— 正是用户说的"频繁后退"。
        # 三个候选口径都实测过(每个 3 次重复):
        #   A. 0.70m 硬闸门         → back 1.4%  d中位 0.94  min 0.41  lost 3
        #   B. 0.80m + 0.70m 硬闸门 → 与 A 等价
        #   C. 0.80m + "距离变化率<0" 例外 → back **29.4%** d中位 1.03  min 0.41
        # C 多出的 28 个百分点后退**没有换来任何 min_d 改善**(都是 0.41), 因为
        # 人走走停停让 _pvvr 频繁变负, 判据分不清"真的逼近"和"距离抖动"。
        # 按"数字在噪声内时选机理正确的"取 A: 人的来向不该由机器人倒车抵消。
        # 真要更近(0.55m 以下)由 guard_d / 8 段防撞人接手, 那时是该退的。
        _bg = float(self.cfg['slow_floor_m']) + 0.20   # 0.70m
        if v_t < 0.0 and dist > _bg:
            v_t = 0.0
        # P80: 上式只解决了"别乱倒车", 但它把 0.70~0.85m 这段也一起封住了,
        # 于是人继续走来时机器人**只停不退**, 而检测器在真值 0.70m 就已经
        # 看不见人(实测 vis 原始值 0.39 → detected=false, 人体占满画幅)。
        # 结果: 停在 0.77m → 0.3s 后瞎 → 人走进 0.44m(裙摆0.26+车体0.18)
        # = 物理接触。t32 逐帧就是这样(76.0~76.4s)。
        # 所以 0.70~0.90m 需要一个**主动让距**: 退的速度跟着"人的逼近速度"
        # 走(不是固定值), 人快就退快、人停下就不退 —— 既保住可见性, 又不
        # 会像 P79-C 那样"距离一抖就倒车"(那版 back 29.4% 且 min_d 无改善)。
        _hold_m = float(self.cfg.get('back_hold_m', 0.85))
        _rear_ok = True
        if int(self.cfg.get('back_hold_on', 1)) and \
                dist < _hold_m and lost_dt < 0.25:
            _dbk2 = self.prox.get('back', -1.0) if self.prox else -1.0
            if _dbk2 is None or _dbk2 < 0.0:
                _dbk2 = 9.9                       # 无数据 → 按后方开阔
            _rear_ok = _dbk2 > self.cfg['lidar_stop_m'] + 0.25
            if _rear_ok:
                _cl = max(0.0, -self._pvvr)       # 人的逼近速度(m/s)
                _vb = (float(self.cfg.get('back_hold_gain', 1.0)) * _cl
                       + float(self.cfg.get('back_hold_k', 0.7))
                       * (_hold_m - dist))
                v_t = -min(float(self.cfg.get('back_hold_vmax', 0.45)),
                           max(float(self.cfg.get('back_hold_vmin', 0.06)),
                               _vb))
            else:
                v_t = 0.0
        # P80: 把前馈/限幅/闸门三个中间量一起导到 diag。
        # 没有它们, "0.71m 处还给 +0.24 前进" 只能靠猜是哪一项干的。
        self._dbg.update(ff=v_ff, pid=v_pid, cap=v_cap,
                         err=err_d, vt=v_t,
                         pv=self._pvvr, vr=self.robot_vfwd)
        # 保持航向: 目标留在 ±band 内
        # P22: 增益从 hold_kp(1.6)/上限0.9 提到 2.2/1.3 —— 目标以 0.7m/s 横移、
        # 距离 1m 时角速度需求达 0.7 rad/s, 原上限 0.9 贴着极限, 稍有滞后就
        # 掉回对正段, 造成每 2s 一次模式抖动。实测被控对象 w 跟踪精度 1.00,
        # 放大增益是安全的。
        if abs(brg) > math.radians(3.0):
            w_t = max(-1.3, min(1.3, self.cfg['hold_kp'] * brg))
        else:
            w_t = 0.0
        # P77 §17-4: 定向绕行窗口对 TRACK 同样有效(否则"受阻→绕行→前方
        # 一净空就回到 TRACK→立刻又被对正拉回"等于没绕)。
        if now < self._detour_until:
            w_t = self._arb_w(float(self.cfg['detour_w'])
                              * self._detour_dir, 'detour', now)

        # --- 4. 绕障已上移到 ALIGN 之前(P47) ---
        # 原来放在这里, 但 `if self._aligning:` 分支在它之前就 return 了,
        # 人站在障碍后面时 brg 跳进 ±50~120°, 直接进 ALIGN 原地转, 绕障
        # 永远执行不到。见"3.5 绕障抢占"。

        # --- 4b. 航点绕障(默认关, 保留开关) ---
        tx, ty, note, dwp = self._waypoint(rx, ry, px, py)
        if not self.cfg['use_waypoint']:
            # P39: 航点绕障在"障碍恰好在左/右绕临界角"时切点两侧来回跳,
            # 而绕障分支会把 v 置 0 强制转向 → 机器人原地打转卡死。
            # 安全层的"停-转-走"已能保证 0 碰撞, 这里直接不启用航点。
            note = ''
        if _dn:
            note = _dn
        # P46: 原判据只看 note 非空 —— 而 P43 的切线绕障会把标记写进
        # `_dn` 再转成 note, 于是"航点绕障"代码被激活, 用 `_waypoint()`
        # 的切点 tx,ty 覆盖刚设好的 v_t/w_t(且 w_lim=1.5)。所以切线绕障
        # 实际从未生效, 一直在跑航点逻辑。必须显式要求 use_waypoint。
        if self.cfg['use_waypoint'] and note:
            err_yaw = wrap(math.atan2(ty - ry, tx - rx) - ryaw)
            # P22 修正: 绕障只允许"限速 + 偏转", 不得越过接近速度与对正带的约束。
            # 原实现直接下 0.6 m/s、±1.8~±2.2 rad/s, 一旦触发就把机器人从目标
            # 方向上硬拽走 —— 日志里 指令v=+0.60 / w=±3.2 全部来自这一段。
            v_t = min(self.cfg['approach_speed'], 1.0 * dwp + 0.10)
            if dwp < 0.30:
                v_t = 0.0
            w_lim = self.cfg['recenter_rot_speed']
            w_t = max(-w_lim, min(w_lim, self.cfg['pid_kp_w'] * err_yaw))
            if abs(err_yaw) > 2.2:
                v_t = 0.0
                w_t = (w_lim if err_yaw >= 0 else -w_lim)

        # --- 5. (已并入 7.5 的全向安全层) ---
        # P26: 原来这里是"层1 深度相机"独立减速, 但它的数据源(prox)现在就是
        # 7.5 用的同一份 8 方位测距, 而且它的硬停阈值 0.48 比 7.5 的 0.62 更宽松
        # —— 留着只会让两套阈值互相覆盖。统一由 7.5 一处裁决。
        # --- 6. 避障层2: 真值地图硬保护 ---
        # P45: 重写。原实现三个缺陷叠加成**静默死锁**:
        #  ① 距离用**外接圆**: ob6(1.0x0.5, 外接半径 0.56) 的判据阈值
        #     0.56+0.56+0.15=1.27m, 而机器人到它真实表面还有 0.73m ——
        #     明明能从旁边过去, 却被判"正前方有障碍"。
        #  ② 只看夹角(cos>0.7≈±45°)不看**横向净空**, 障碍偏在侧前方 26°、
        #     侧向有 0.53m 余量时仍算"挡路"。
        #  ③ 停车后**不给转向、不写 note**。而 TRACK 的 w 只在 |brg|>3°
        #     时才设, 对正后 w 恒为 0 → 既不前进也不转, 静默死等到
        #     goto_timeout_s(20s)。日志实证: 卡在(-2.60,-5.03), 目标
        #     (-0.43,-4.95) 正前方 2.18m, 指令v=+0.00 w=+0.0 连续 20s,
        #     随后 12s 原地扫描 + 再来一轮 → 人丢失 38.3s。
        if v_t > 0:
            look = 0.35 + 0.5 * abs(v_t)
            nx = rx + math.cos(ryaw) * look
            ny = ry + math.sin(ryaw) * look
            # 墙体: 只有"已在硬边界外、且这一脚还在继续往外"才拦。
            # 原判据 abs(nx)>WALL_HARD 连**贴墙平行行驶**都会永久禁行
            # (y 分量不变, 仅仅因为 |ny| 超一点点)。
            if (abs(rx) > WALL_HARD and abs(nx) > abs(rx) + 1e-6) or \
                    (abs(ry) > WALL_HARD and abs(ny) > abs(ry) + 1e-6):
                v_t = 0.0
                note = (note + ' 墙前硬停').strip()
            else:
                # P53: 单点前瞻 + 外接圆横向判据 → 扫掠真值几何。
                # 原判据两处过保守: ① 只在前瞻一个点(0.35+0.5v)取样, 对
                # 0.2m 处的障碍视而不见; ② `_lat > _ext + ROBOT_R` 里的
                # _ext 是 box 的**外接圆**半径(ob2 的 1.2x0.6 → 0.671, 真实
                # 半宽只有 0.60/0.30), 横向净空被低估近 0.4m。
                # 这里只做"马上要撞了"的兜底; 分级减速交给 _safety_gate。
                _dpath, _ok6, _obj6, _s6 = path_free(rx, ry, ryaw)
                # P73 关键修复: 原来是 `if _dpath < 0.33 or not _ok6`。
                # _ok6 是"沿 1.6m 扫掠"的布尔量, 只要沿途任一点车心到
                # 障碍表面 <= ROBOT_R+0.10 = 0.28m 就为 False; 而当没有
                # 真正接触时 _obj6 是 None → 既不写 note 也不给转向, v_t
                # 被静默改成 0。实测: 目标 2.44m、正前方 1.48m 无遮挡、
                # ff/pid/cap 全部正常, 但 dh 恒为 0.00、指令 v 恒为 +0.00,
                # 120s 里 34.8% 的采样被这一条掐成 0 → 人越走越远。
                # 0.18m 车体半径 + 0.10m 余量过 0.25m 净空完全够, 不该停;
                # 现在只在"真挡住"(接触距离 <0.33m 或窄段前沿 <0.30m)时
                # 硬停, 其余窄道只限速。
                # P73 第二次修正: 不要用 "窄段前沿 <0.30m" 做硬停 ——
                # _s6 是"沿航向第一次净空不足"的弧长, 机器人**已经在**
                # 窄带里时(侧方就是立柱) _s6 从 0.05 起步, 这条判据等于
                # 永久冻结。实测 t8: 窄道硬停 50 次, 人从 1.0m 走到 5.7m
                # 而机器人一动不动。硬停只保留"马上要接触"(_dpath<0.33),
                # 窄段本身交安全层限速(见 _safety_gate)。
                if _dpath < 0.33:
                    v_t = 0.0
                    if isinstance(_obj6, str):
                        note = (note + ' 墙前硬停').strip()
                    elif _obj6 is not None:
                        # P45 关键: 必须给转向。停车 + w=0 = 永久死锁。
                        _lat6 = (-(_obj6[1] - rx) * math.sin(ryaw)
                                 + (_obj6[2] - ry) * math.cos(ryaw))
                        w_t = -1.10 if _lat6 > 0 else 1.10
                        note = (note + ' 真值硬保(%+.1f,%+.1f)间隙%.2f%s'
                                % (_obj6[1], _obj6[2],
                                   geo_gap(rx, ry, _obj6),
                                   '右绕' if _lat6 > 0 else '左绕')).strip()

        # --- 7. 近墙修正 ---
        if abs(rx) > WALL_SOFT or abs(ry) > WALL_SOFT:
            to_c = math.atan2(-ry, -rx)
            w_t += 1.6 * wrap(to_c - ryaw)
            if v_t > 0:
                v_t = min(v_t, 0.35)

        # --- 7.5 全向安全层 ---
        # 已抽成 _safety_gate(), 在 _tick 的每个出口统一调用。原因: 对正、
        # 原地扫描、防撞人后退、卡死脱困都会直接设速度, 只要有一处绕过
        # 安全层, 就会在那里撞 —— 实测 E 场景(斜冲 ob5)的 -0.062m 擦碰
        # 正是"倒车时没看后方"造成的。


        # --- 8. 防撞人 ---
        if dist < guard_d:
            back_clear = True
            for (ox, oy, r) in OBSTACLES:
                dx, dy = ox - rx, oy - ry
                d = math.hypot(dx, dy)
                if d < r + 0.45 and \
                        (dx * math.cos(ryaw) + dy * math.sin(ryaw)) / \
                        max(d, 1e-6) < -0.5:
                    back_clear = False
                    break
            if back_clear:
                v_t = max(min(v_t, -(0.2 + 0.7 * (guard_d - dist))), -0.5)
            else:
                v_t = 0.0
                l_, r_ = self.prox.get('left', -1), self.prox.get('right', -1)
                if l_ > 0 and r_ > 0:
                    w_t = 1.0 if l_ > r_ else -1.0
                else:
                    w_t = 1.0

        # --- 9. 贴墙应急 ---
        if abs(rx) > 5.55 or abs(ry) > 5.55:
            to_c = math.atan2(-ry, -rx)
            errc = wrap(to_c - ryaw)
            if abs(errc) > 0.5:
                v_t = 0.0
                w_t = 1.8 if errc >= 0 else -1.8
            else:
                v_t = 0.35
                w_t = 1.2 * errc
            if now - self._last_log > 1.0:
                self.get_logger().warn(
                    f'FOLLOWER 贴墙应急 转向场心后离开 (x={rx:+.2f}, y={ry:+.2f})')

        # --- 10. 卡死脱困(指令在走/在转, 但位姿不动) ---
        # P25 修正: 原实现判定卡死后把 _stuck_t0 立刻重置为当前时刻, 于是脱困
        # 指令只下发一个 tick(50ms), 下一个 tick 又从零开始 2.5s 计数 ——
        # 实测机器人顶住立柱后每 2.5s 才退 5cm, 基本等于不动。现在改成
        # "进入脱困窗口": 窗口内持续推进倒退+转向, 直到真的挪开 0.25m。
        escape_s = self.cfg['escape_s']
        if now < self._escape_until:
            v_t, w_t = self._escape_v, self._escape_w
            if not note:
                note = '脱困中'
            moved = math.hypot(rx - self._stuck_pos[0],
                               ry - self._stuck_pos[1])
            # 倒退前看一眼背后, 别自己撞上后面的柱子/墙
            if moved < 0.06 and self._escape_v < 0:
                d_b1 = self._sector_min(150.0, 180.0)
                d_b2 = self._sector_min(-180.0, -150.0)
                if (d_b1 > 0 and d_b1 < 0.55) or (d_b2 > 0 and d_b2 < 0.55):
                    self._escape_v = 0.0     # 背后贴死 → 只原地转
                    v_t = 0.0
            if moved > 0.25:
                self._escape_until = 0.0
                self._stuck_t0 = 0.0
                self.get_logger().info(
                    f'FOLLOWER ✓ 已脱困(挪开 {moved:.2f}m), 恢复跟踪')
        else:
            blocked_lin = abs(self.v) > 0.12 and rspd < 0.06
            blocked_rot = abs(w_t) > 0.7
            near_wall = abs(rx) > 5.4 or abs(ry) > 5.4
            if blocked_lin or blocked_rot:
                if self._stuck_t0 == 0.0:
                    self._stuck_t0 = now
                    self._stuck_pos = (rx, ry)
                    self._stuck_yaw = ryaw
                elif now - self._stuck_t0 > 1.8:
                    moved = math.hypot(rx - self._stuck_pos[0],
                                       ry - self._stuck_pos[1])
                    dyaw = abs(wrap(ryaw - self._stuck_yaw))
                    if (blocked_lin and moved < 0.12) or \
                            (blocked_rot and dyaw < 0.22):
                        to_c = math.atan2(-ry, -rx)
                        w_esc = 1.2 if wrap(to_c - ryaw) >= 0 else -1.2
                        if near_wall:
                            fw_look = 0.30
                            fx = rx + math.cos(ryaw) * fw_look
                            fy = ry + math.sin(ryaw) * fw_look
                            fw_ok = abs(fx) < WALL_HARD and abs(fy) < WALL_HARD
                            v_esc = 0.40 if fw_ok else -0.40
                        else:
                            v_esc = -0.50
                        self._escape_v = v_esc
                        self._escape_w = w_esc
                        self._escape_until = now + escape_s
                        self._stuck_pos = (rx, ry)
                        self.get_logger().warn(
                            f'FOLLOWER 卡死脱困 进入{escape_s:.1f}s窗口'
                            f'(平移{moved:.2f}m/旋转{math.degrees(dyaw):.0f}°'
                            f'{"贴墙" if near_wall else "倒车"})')
                    self._stuck_t0 = now
                    self._stuck_pos = (rx, ry)
                    self._stuck_yaw = ryaw
            else:
                self._stuck_t0 = 0.0

        # 最终裁剪: 任何路径算出的速度都要过这一关
        v_t, w_t, _sn = self._safety_gate(v_t, w_t, now, dist, brg)
        if _sn:
            note = _sn if not note else (note + ' ' + _sn)
        self._publish_smooth(v_t, w_t, now)
        self._log(now, src, dist, pspd, rspd, w_t, brg, note, nscan,
                  rx, ry, px, py)

    # ---------- 接近减速(P79) ----------
    def _slow_k(self, d):
        """接近减速系数: 远端 1.0 → 区间下限 0.0, 单调、C0 连续。

        用户口径: 人距 <2m 必须开始减速。曲线
            k = clamp((d - slow_floor_m) / (slow_zone_m - slow_floor_m)) ** slow_exp
        以 0.6 的指数让 2.0→1.5m 段降得缓(还要跟得上人)、1.0→0.5m 段骤降
        (真危险的那半米速度已经很小)。TRACK / ALIGN 两个前进出口共用。
        """
        if d is None:
            return 1.0
        sz = float(self.cfg['slow_zone_m'])
        sf = float(self.cfg['slow_floor_m'])
        if sz <= sf + 1e-6 or d >= sz:
            return 1.0
        r = (float(d) - sf) / (sz - sf)
        return max(0.0, min(1.0, r)) ** float(self.cfg['slow_exp'])

    # ---------- 转向仲裁(P77 §17-3/§17-4) ----------
    # P77 §17-4: 'detour'(定向绕行) 高于 'avoid' —— 它本身就是"已决定好方向
    # 的绕障", 且带固定时间窗, 窗口内不许任何模块抢转向。
    _W_PRIO = {'detour': 4, 'avoid': 3, 'peek': 2.5, 'scan': 2, 'align': 1,
               'track': 0}

    def _arb_w(self, w_want, owner, now):
        """转向(w)的**唯一出口** —— 优先级仲裁 + 双向滞回。

        背景(P76 实测): 受控场景 75s 内 w 换向 58 次 ≈ 0.77 次/s。归因发现
        多数换向不是"目标换边", 而是 SCAN 与 ALIGN 各算各的 w、在状态边界上
        交替接管后互相推翻 —— 两边的 `_scan_dir` / `_align_dir` 只能锁住自己,
        锁不住对方。所以把"谁决定转向"显式化:

        规则:
          1) 同时只有**一个** owner 能决定转向。优先级 avoid(3) > scan(2)
             > align(1) > track(0); 高优先级可**立即抢占**(安全相关不许等)。
             同级或降级必须"连续申请 w_owner_hold_s" 且 "当前 owner 已保持
             w_owner_hold_s" 才交接, 杜绝每帧来回切。
          2) owner 内部方向翻转要求**反向请求持续 w_hold_s**, 或幅值超过
             w_flip_urgent(真急转)。只有一帧的噪声反不了向。
          3) 不允许翻转时不归零, 而是"沿用旧方向 + 新幅值": 保持转动,
             避免"先停再反向"造成额外的二次迟滞。

        参数:
          w_want: 本模块期望的角速度(带符号)
          owner : 'avoid' / 'scan' / 'align' / 'track'
        返回: 实际应当下发的 w
        """
        cfg = self.cfg
        w_want = float(w_want)
        if not int(cfg.get('w_arb_on', 1)):     # A/B 开关: 关掉即完全直通
            return w_want
        while self._w_flip_win and now - self._w_flip_win[0] > 1.0:
            self._w_flip_win.pop(0)
        self._w_flip_rate = float(len(self._w_flip_win))

        # --- 1. owner 仲裁 ---
        cur = self._w_owner
        if owner != cur:
            if self._w_req != owner:
                self._w_req = owner
                self._w_req_t = now
            _cur_prio = self._W_PRIO.get(cur, -1)
            _prio = self._W_PRIO.get(owner, 0)
            _settled = ((now - self._w_req_t)
                        >= float(cfg['w_owner_hold_s'])
                        and (now - self._w_owner_t)
                        >= float(cfg['w_owner_hold_s']))
            if cur == '' or _prio > _cur_prio or _settled:
                self._w_owner = owner
                self._w_owner_t = now
                self._dbg['wown'] = owner
                self._dbg['wsw'] = self._dbg.get('wsw', 0) + 1
        else:
            self._w_req = owner
            self._w_req_t = self._w_owner_t

        # --- 2. 方向滞回 ---
        # 滞回只对"会互相打架的两个模块"(scan/align)生效。avoid 是安全动作,
        # 不许被滞回拖住 —— 绕障要往左就得马上往左, 迟 0.45s 可能擦上障碍。
        _hyst = owner in ('scan', 'align')
        if abs(w_want) < 0.06:          # 幅值太小: 不动方向记忆, 也不下发
            return 0.0
        s = 1.0 if w_want > 0.0 else -1.0
        if self._w_sign == 0.0 or not _hyst:
            self._w_sign = s
            self._w_opp_t = 0.0
            return w_want
        if s == self._w_sign:
            self._w_opp_t = 0.0
            return w_want
        # 反向请求: 要持续够久或足够急
        if self._w_opp_t <= 0.0:
            self._w_opp_t = now
        _urgent = abs(w_want) >= float(cfg['w_flip_urgent'])
        if _urgent or (now - self._w_opp_t) >= float(cfg['w_hold_s']):
            self._w_sign = s
            self._w_opp_t = 0.0
            self._w_flip += 1
            self._w_flip_win.append(now)
            self._w_flip_rate = float(len(self._w_flip_win))
            return w_want
        # P77: 不允许翻转时**回 0**, 不再"沿用旧方向 + 新幅值"。
        # 第一版是沿用旧方向, 实测把换向从 60 次抬到 72 次 —— 因为反向硬顶
        # 会造成过冲, 反而加剧了随后的过零。回 0 只是"这一帧不转", 是阻尼,
        # 既不会朝错方向顶过去, 也能在下一帧重新判断。
        return 0.0

    # ---------- 全向安全层 ----------
    def _safety_gate(self, v_t, w_t, now, tgt_d=None, tgt_b=None):
        """全向 8 方位安全层 —— 最高优先级的硬裁剪。

        P26。数据: /perception/proximity 的 8 个 45° 方位(4 台深度相机按 5
        扇区合成)。为什么不用 /scan: gpu_lidar 只支持 ogre2, 本项目为绕开
        VMware 白屏固定用 ogre1, /scan 的 360 条射线全是 range_min(0.12m);
        CPU 版 type="lidar" 又被 gz-sim8 拒绝("Sensor type LIDAR not
        supported yet")。深度相机是本环境唯一可靠的 360° 测距。

        关键: 必须在**所有**出口调用 —— 对正(ALIGN)、原地扫描、防撞人后退、
        卡死脱困都会直接设速度, 只要有一处绕过它, 就会在那里撞。

        返回 (v_t, w_t, note)
        """
        p8 = self.prox
        # P30: 先看"多久没收到新数据", 再看 payload 里的 age。
        # 只查 age 是不够的 —— depth_percept 一死, last message 的 age 就
        # 永远冻在最后那个小值上, 安全层会无限期拿过期数据当实时数据用。
        stale = now - self._prox_rx if self._prox_rx > 0.0 else 1e9
        if stale > 0.5:
            if now - self._prox_dead_log > 3.0:
                self._prox_dead_log = now
                self.get_logger().error(
                    'FOLLOWER 感知数据中断 %.1fs —— depth_percept 可能已退出, '
                    '禁止前进(仍可转向)' % stale)
            if v_t > 0.0:                      # 盲开比停下危险
                v_t = 0.0
            return v_t, w_t, '感知中断禁行'
        if p8.get('age', 99.0) >= self.cfg['lidar_max_age_s']:
            return v_t, w_t, ''

        def _pv(k):
            v = p8.get(k, -1.0)
            return v if v > 0 else 99.0

        def _lr(lo, hi):
            r = self._sector_min(lo, hi)
            return r if r > 0 else 99.0

        d_f, d_fl, d_fr = _pv('front'), _pv('fl'), _pv('fr')
        d_l, d_r = _pv('left'), _pv('right')
        d_b = min(_pv('bl'), _pv('back'), _pv('br'))
        # 雷达若在本环境真的可用(ogre2)则一并纳入, 取更保守值
        if self._lidar_ok and self.scan is not None \
                and now - self.scan[3] < self.cfg['lidar_max_age_s']:
            d_f = min(d_f, _lr(-26.0, 26.0))
            d_fl = min(d_fl, _lr(26.0, 65.0))
            d_fr = min(d_fr, _lr(-65.0, -26.0))
        stop_d = self.cfg['lidar_stop_m']
        slow_d = self.cfg['lidar_slow_m']
        tau = self.cfg['lidar_tau_s']
        side_d = self.cfg['lidar_side_m']
        note = ''
        # 斜 45° 方向折算到前向要乘 cos45°≈0.72(原来加常数 0.10, 在 1.5m
        # 量级上等于不折算, 斜前方的立柱一直被当成"前方还很空")
        # P31: 把"被跟随的人"从障碍里摘出去。
        # 跟到 1.0m 时人的躯干表面距相机只有 ~0.7m, 低于 slow_d(1.6), 安全
        # 层于是把要跟随的人当成障碍, 一路把速度压到 0.06~0.12m/s(实测 111s
        # 里 259 条全向限速), 机器人只能慢慢爬, 人一走动就掉队。
        if tgt_d is not None and 0.25 < tgt_d < 6.0:
            expect = max(tgt_d - 0.30, 0.25)      # 人躯干表面的期望距离
            lim = slow_d * 1.02                   # 抬到限速阈值之上
            if abs(tgt_b) <= math.radians(45.0) and abs(d_f - expect) < 0.45:
                d_f = max(d_f, lim)
            if tgt_b > math.radians(-10.0) and abs(d_fl - expect) < 0.45:
                d_fl = max(d_fl, lim)
            if tgt_b < math.radians(10.0) and abs(d_fr - expect) < 0.45:
                d_fr = max(d_fr, lim)
        # P44: 斜前方不再无条件乘 0.72。原来 45° 方向 0.90m 的障碍会被
        # 折算成 0.65m(< stop_d 0.66) —— 机器人明明可以从旁边过去, 却直接
        # 停车。实测 150s 里 far_pct 73%、中位距离 2.41m, 大部分是被这条
        # "看什么都像正前方"的规则卡住的。
        # P53: "正前方还能走多远" = 真值几何沿航向扫掠的结果, 8 扇区
        # (d_fl/d_fr) 只保留"限速"职责, 不再参与"停不停"。
        # 原 `min(d_f, min(d_fl,d_fr)*0.72)` 在 eval14 造成极限环:
        #   车心到 ob2 表面 0.38m, ob2 的角在车头左前 64°、0.395m 处;
        #   d_fl=0.34 → d_head=0.245 < stop_d 0.66 → 急停 → 强制绕右(w=-0.9)
        #   → ALIGN 把车头转回目标(+1.1) → 左前再次变窄 → 再急停 …
        #   实测 w 每 0.5s 翻号, 距离 1.05→2.12m 单调变大。
        #   而同一位置沿 39° 的法向间隙有 0.355m(车半径 0.18), 过得去。
        _dpath, _okp, _objp, _stight = path_free(self.rx, self.ry, self.ryaw)
        if self._avoiding:
            d_head = d_f             # 绕障时车头已偏向侧方, 看正前方即可
        else:
            # P73: 原 `elif not _okp: d_head = 0.0` 与 section 6 同一个病
            # —— 把"旁边有柱子"当成"前方堵死"。硬闸改用真实接触距离
            # _dpath; 只有当窄段前沿已贴到车头前 30cm 内才收紧。
            # P73: 硬闸只看真实接触距离 _dpath。窄段不能折进 d_head ——
            # 机器人**已经**在窄带里时 _stight 从 0.05 起, _stight+stop_d
            # 会把 d_head 拉到 0.71, 经 (dh-stop_d)/tau 斜坡后只剩 0.08,
            # 等于换了个马甲的死锁(t9 实测 far 63.7%)。窄段只按下面
            # 的 0.40m/s 上限限速。
            d_head = min(d_f, _dpath)

        self._dbg.update(dh=d_head, df=d_f, dfl=d_fl, dfr=d_fr,
                         dl=d_l, dr=d_r, db=d_b, dp=_dpath,
                         st=_stight, okp=1.0 if _okp else 0.0,
                         av=1.0 if self._avoiding else 0.0,
                         escd=self._esc_dir)
        # P77 §17-4: 前方彻底净空(> estop_release_k*stop_d) → 受阻计时清零。
        # 这样"驻留 hold_max_s → 绕行 detour_s → 净空 → 重新对正", 形成一个
        # 明确的状态循环, 不会因为计数残留而反复进出绕行窗口。
        if d_head > stop_d * float(self.cfg['estop_release_k']):
            self._hold_t0 = 0.0
        # P78: 受阻 → 解除 的**那一帧**记下时刻, 后续 resume_soft_s 内做
        # 前进软启动。判据用"受阻态"而不是"驻留态": 急停(不是驻留)解除时
        # 同样是 v 从 0 跳全速, 一样会顶到人。
        _stalled = (d_head <= stop_d) or self._estop_latch
        if self._was_stalled and not _stalled:
            self._resume_t0 = now
        self._was_stalled = _stalled
        # P73: 沿途存在"车心净空 < ROBOT_R+0.20"的窄段时只限速不限行。
        # 0.30m/s 下 10Hz 深度 + 0.04m 制动距离, 留得下反应余量;
        # 而原来的硬停会让"旁边有柱子"变成永久冻结(实测 50 次死锁)。
        if (not _okp) and _stight < 1.00 and v_t > 0.40:
            v_t = 0.40
            note = (note + ' 窄道限速0.40').strip()
        if v_t > 0.02:                                  # ---- 前进 ----
            # P77 §17-3: 急停的"吸住/释放"必须各有一个阈值, 否则 d_head 在
            # stop_d 上下浮动时会每帧吸/放一次。真实机理(diag 实证):
            # 吸住时 w 被强行改成 `1.15*_esc_dir`, 与对正**反号**; 放开时立刻
            # 被对正掰回去。同一帧里 note 同时出现"对正-5°"和"全向急停
            # 前0.63"、w=+1.15 —— 于是每 1.4s 一个乒乓(受控场景 66 次/75s),
            # 这就是"障碍旁来回摆"的直接来源, 而不是 esc_dir 自己左右翻
            # (esc_dir 已有 0.35m 换边余量 + 0.45s 保持, 实测很少真的翻)。
            _rel = stop_d * float(self.cfg['estop_release_k'])
            if d_head <= stop_d:
                if not self._estop_latch:
                    self._estop_hold_t = now
                self._estop_latch = True
            elif self._estop_latch and (
                    d_head > _rel
                    or now - self._estop_hold_t
                    > float(self.cfg['estop_max_s'])):
                # 兜底: 窄角里可能永远等不到 1.25*stop_d 的净空, 最多吸住
                # estop_max_s 就放行, 免得"转出去"变成新的死锁。
                self._estop_latch = False
            if d_head <= stop_d or self._estop_latch:
                v_t = 0.0
                _td = tgt_d if tgt_d is not None else self._last_dist
                # P77 §17-4 关键修复: "人就在眼前、但前进被挡" 时**驻留**,
                # 不做"哪边空往哪边绕"的转向。
                # 机理(受控场景实测): 人在 1.8m、偏 23°, 前方 0.6m 是立柱 →
                # 急停把 w 掰成 1.15*_esc_dir(背离人) → 下一帧对正又掰回来
                # → 1.4s 一个循环。后 40s 净位移只有 0.1~0.3m, 偏航却累计
                # 1500~1800°(净转 20~40°) —— 这就是"障碍旁来回摆动"的本体。
                # 但"一受阻就驻留"会把 1~2s 就能绕过去的瞬时阻挡也冻住
                # (回放实测 in_band 37.4→34.5、far 33.2→35.6); 而"驻留超时
                # 后强制定向绕行"更糟(车头掰开 78° 还往前顶: min_d
                # 0.649→0.451、居中度 99.3→96.5、丢人 1 次)。所以只有一条
                # 判据真正成立 —— **持续**:
                #   受阻 <  hold_after_s → 照旧"哪边空往哪绕"(瞬时阻挡)
                #   受阻 >= hold_after_s → 驻留对正(绕不过去, 转也白转)
                _hold_zone = (not self._avoiding) and 0.0 < _td <= \
                    float(self.cfg['hold_blocked_m'])
                if _hold_zone:
                    if self._hold_t0 <= 0.0:
                        self._hold_t0 = now
                    _blocked_s = now - self._hold_t0
                else:
                    self._hold_t0 = 0.0
                    _blocked_s = 0.0
                _hold_ready = _hold_zone and \
                    _blocked_s >= float(self.cfg['hold_after_s'])
                _detour_ok = _hold_ready and \
                    int(self.cfg.get('detour_on', 0)) and \
                    _blocked_s >= float(self.cfg['hold_max_s'])

                if _detour_ok:
                    # ---- 定向绕行窗口(A/B 用, 默认关): 一次只转一个方向 ----
                    if now >= self._detour_until:
                        _room = {1.0: d_fl, -1.0: d_fr}
                        # ① 优先沿"本来就想去"的方向(与上层意图同号且有余量)
                        _dd = 0.0
                        if abs(w_t) > 0.05:
                            _sd = 1.0 if w_t > 0 else -1.0
                            if _room[_sd] > stop_d * 0.92:
                                _dd = _sd
                        # ② 否则选余量大的一侧
                        if _dd == 0.0:
                            _dd = 1.0 if d_fl >= d_fr else -1.0
                        # ③ 方向迟滞: 除非对侧"明显更空", 否则不换边。
                        #    不设这条, d_fl/d_fr 会随自身转向互相追逐,
                        #    每个窗口都换方向 —— 又回到左右打摆。
                        if self._detour_dir != 0.0 and _dd != self._detour_dir:
                            if not (_room[-self._detour_dir]
                                    > _room[self._detour_dir]
                                    + float(self.cfg['detour_dir_hyst'])):
                                _dd = self._detour_dir
                        self._detour_dir = _dd
                        self._detour_t0 = now
                        self._detour_until = now + float(self.cfg['detour_s'])
                    w_t = self._arb_w(float(self.cfg['detour_w'])
                                      * self._detour_dir, 'detour', now)
                    note = (f'受阻{_blocked_s:.1f}s 定向绕'
                            f'{"左" if self._detour_dir > 0 else "右"}'
                            f' 前{d_head:.2f} 人{_td:.2f}')
                elif _hold_ready:
                    note = (f'目标受阻 驻留{_blocked_s:.1f}s '
                            f'前{d_head:.2f} 人{_td:.2f}'
                            f' 偏{math.degrees(w_t):+.0f}°/s')
                elif self._avoiding:
                    note = f'绕障中受阻 前{d_head:.2f}'
                    # P46: 绕障态的车头已经偏转 avoid_deg 朝向侧方, 这里的
                    # d_head 量的是**侧方**余量。若再另起一套 esc_dir 转向,
                    # 会与绕障的转向反向抵消 —— 实测 w 在 +1.50/-1.15 之间
                    # 每 0.5s 翻一次, 机器人原地抖 28s 只挪 0.3m。
                else:
                    # P27: 急停时的转向必须与上层意图协同。原来是"哪侧空就
                    # 往哪转", 与上层"目标在右就右转"直接对打 —— 实测 w 在
                    # -1.0 -> -1.4 -> +1.1 之间每帧翻号, 车在原地抖。
                    room_l, room_r = d_fl, d_fr
                    want = 0.0
                    if w_t > 0.05 and room_l > stop_d * 0.92:
                        want = 1.0                   # 想左转且左前有余量
                    elif w_t < -0.05 and room_r > stop_d * 0.92:
                        want = -1.0                  # 想右转且右前有余量
                    if want == 0.0:                  # 直冲, 或意图侧被堵死
                        want = 1.0 if room_l >= room_r else -1.0
                    if self._esc_dir == 0.0 or now - self._esc_t0 > 3.0:
                        self._esc_dir, self._esc_t0 = want, now
                    elif want != self._esc_dir \
                            and now - self._esc_t0 > self.cfg['esc_hold_s']:
                        # 滞回: 只有另一侧"明显更空(0.35m 以上)"才允许换边。
                        # 不设这条, d_fl/d_fr 会随自身转向互相追逐, 永远抖。
                        cur = room_l if self._esc_dir > 0 else room_r
                        oth = room_r if self._esc_dir > 0 else room_l
                        if oth > cur + 0.35 and oth > stop_d * 1.05:
                            self._esc_dir, self._esc_t0 = -self._esc_dir, now
                    w_t = 1.15 * self._esc_dir
                    note = (f'全向急停 前{d_f:.2f} 左前{d_fl:.2f}'
                            f' 右前{d_fr:.2f}'
                            f' 绕{"左" if self._esc_dir > 0 else "右"}'
                            f'{"" if d_head <= stop_d else " 保持%.2f>%.2f" % (d_head, _rel)}')
                # P57: 贴身回退 —— 已经贴到障碍表面时, "原地转"往往被物理
                # 挡住(车体外接圆扫进障碍), 安全层又会把后退判成
                # "d_b < stop_d → 后退受阻", 两条路都堵死 → 冻死。
                # 实测 evalN(PW_SEED=7) 卡进 ob5 的角 45s。真值清距 < 8cm
                # 时允许缓慢倒出, 只要后方不贴死。
                _dmin2 = 99.0
                for _g in GEO:
                    _gg2 = geo_gap(self.rx, self.ry, _g)
                    if _gg2 < _dmin2:
                        _dmin2 = _gg2
                if _dmin2 < ROBOT_R + 0.08 and d_b > stop_d * 0.80:
                    v_t = -0.20
                    note = (note + ' 贴身回退 清距%.2f'
                            % (_dmin2 - ROBOT_R)).strip()
                # P27: 原地转不开 → 先退再绕。静态障碍挡在目标正前方时
                # (人站在立柱后面), 纯原地转是死锁: 转到障碍离开视野,
                # 上层又把车头转回来, 于是永久"转-停-转"。
                # P46: 绕障态不后退 —— 车头已偏转去绕行, 再后退会来回蹭。
                if not self._avoiding:
                    if self._blk_t0 <= 0.0:
                        self._blk_t0 = now
                    elif now - self._blk_t0 > self.cfg['blk_s']:
                        if d_b > stop_d + 0.40 and \
                                now - self._blk_t0 < self.cfg['blk_s'] + 1.8:
                            v_t = -0.18
                            note += ' 受阻>%.1fs→后退脱困' % self.cfg['blk_s']
                        else:
                            self._blk_t0 = now   # 退够/后方也堵: 重新计时
            elif d_head < slow_d:
                v_cap = (d_head - stop_d) / tau
                # P44: 原式在 d_head=0.70 时只给 (0.70-0.66)/0.6 = 0.067m/s,
                # 机器人贴着障碍以 7cm/s 磨, 人一走动就彻底掉队。只要还有
                # 余量就至少给 0.22m/s(10Hz 深度 + 0.04m 制动距离, 安全);
                # 余量不足 0.04m 就交给下面的急停分支去"停-转"。
                if d_head > stop_d + 0.04:
                    v_cap = max(v_cap, 0.22)
                if v_t > v_cap:
                    v_t = max(0.0, v_cap)
                note = (f'全向限速 前{d_f:.2f} 左前{d_fl:.2f}'
                        f' 右前{d_fr:.2f} v<={v_cap:.2f}')
                if v_t > 0.05 and abs(d_fl - d_fr) > 0.25 \
                        and min(d_fl, d_fr) > stop_d:
                    w_t += 0.55 if d_fl > d_fr else -0.55
            if v_t > 0.05:                              # 侧向防擦
                if d_l < side_d:
                    w_t -= 0.9 * (side_d - d_l) / side_d
                if d_r < side_d:
                    w_t += 0.9 * (side_d - d_r) / side_d
                # P26: 侧翼必须包含 bl/br 两个后翼。绕障时障碍的方位会从
                # 90° 连续转到 135°, 只盯 left/right 会在这段漏掉, 于是
                # "擦着立柱边走" —— 实测 C/E 的 -0.033/-0.059 就是这么来的
                d_wing = min(d_l, d_r, _pv('bl'), _pv('br'))
                # P50: 侧翼余量只能**限速**, 不能直接禁行。
                # 原判据 `d_wing < stop_d(0.66) → v_t = 0` 是拿**正前方**
                # 的停车距离去判**侧向** —— 机器人沿墙/沿障碍正常直行时,
                # 侧向只有 0.5~0.6m 完全正常, 却会被永久禁行。
                # 实测(日志): 机器人 y=-5.39、右侧到墙 0.54m、正前方还有
                # 0.98m 空间、限速允许 0.54m/s, 结果 `指令v=+0.00` 恒为 0,
                # 26s 一动不动 → 视觉丢失 → GOTO/SCAN → 打转。
                # 这条规则正是"人一走到障碍附近, 机器人就卡住并丢目标"的
                # 总根源: 人靠近障碍 → 机器人跟着靠近 → 侧翼 < 0.66 →
                # 永久禁行 → 掉队 → 丢失。
                _wing_min = stop_d * 0.55          # 0.363m: 真正贴死才停
                if d_wing < _wing_min:
                    # P35: 原实现"限速到 0.10 继续挤"在实测中会慢慢钻进
                    # 立柱(0.10m/s x 15s = 1.5m, gap 走到 -0.42)。侧翼真
                    # 的贴死时必须停止前进, 并强制往空的一侧转。
                    v_t = 0.0
                    if abs(w_t) < 0.35:
                        w_t = 0.9 if d_r < d_l else -0.9
                elif d_wing < stop_d:
                    # 侧翼偏窄但不至于贴死: 只限速, 允许沿墙/沿障继续直行
                    _v_wing = max(0.12, (_wing_min + 0.30 - d_wing) / 0.45)
                    if v_t > _v_wing:
                        v_t = _v_wing
                    note = (note + ' 侧翼限速 侧%.2f v<=%.2f'
                            % (d_wing, _v_wing)).strip()
                # P55: 真值贴身兜底。8 扇区各有一个"该方向最近距离", 但扇区
                # 之间的斜后盲区会漏掉"贴角通过"的情况 —— 实测 eval17 出现
                # 一次 0.098m 间隙(评测口径=扣除车体外接半径后的表面间隙),
                # 而扇区侧翼最小只读到 0.36 没报警。真值几何是精确的, 直接
                # 用最近障碍的 geo_gap 兜底: 太近就只限速(不改转向, 避免与
                # 上层航向控制对打)。
                _dmin_t = 99.0
                for _g in GEO:
                    _gg = geo_gap(self.rx, self.ry, _g)
                    if _gg < _dmin_t:
                        _dmin_t = _gg
                if _dmin_t < ROBOT_R + 0.09 and v_t > 0.10:
                    v_t = 0.10
                    note = (note + ' 真值贴身%.2f v<=0.10'
                            % (_dmin_t - ROBOT_R)).strip()
            if d_head > stop_d:
                self._blk_t0 = 0.0                      # 已脱离受阻
        elif v_t < -0.05:                               # ---- 后退 ----
            # P76: 这里原来是 `if d_b < stop_d: v_t = 0.0` 一刀切, 是"障碍旁
            # 徘徊不前"的总根源。实测(受控场景 diag): 人停在机器人正前方
            # 0.67m(比 target_d=1.0 近) → 距离环持续给负速度想拉开 → 后方
            # 0.44m 是立柱 → d_b<stop_d(0.66) → v=0 → 误差不消失 →
            # 永远被拒 → 冻结 54.9 秒, 直到录制结束都没解开。
            # 关键: d_b 是"到障碍表面"的距离, stop_d 只是保守阈值, 不是
            # 物理边界。d_b=0.44 减去车体外接半径 0.18 还有 0.26m 真实余量,
            # 缓退 0.10m/s 完全安全(10Hz 深度, 每帧只挪 1cm)。
            if d_b < stop_d:
                if d_b > ROBOT_R + 0.10:
                    v_t = max(v_t, -0.10)          # 缓退, 不是禁退
                    note = f'后退缓退 后{d_b:.2f}'
                else:
                    v_t = 0.0
                    note = f'后退受阻 后{d_b:.2f}'
            elif d_b < slow_d:
                v_cap_b = -(d_b - stop_d) / tau
                if v_t < v_cap_b:
                    v_t = v_cap_b
                    note = f'后退限速 后{d_b:.2f}'

        if note and now - self._last_avoid_log > 1.0:
            self._last_avoid_log = now
            self.get_logger().info(
                f'FOLLOWER {note} | 左{d_l:.2f} 右{d_r:.2f} 后{d_b:.2f}')
        return v_t, w_t, note

    # ---------- 输出与日志 ----------
    def _publish_smooth(self, v_t, w_t, now):
        """线速度梯形限幅: 加速温和、刹车果断。

        P21 修正: 原实现上升/下降共用一个斜率(0.9 m/s²)。切进 ALIGN 时目标
        线速度已置 0, 但 self.v 仍以 0.9 m/s² 缓慢收敛, 于是在"原地对正"期间
        机器人还带着 0.2~0.3 m/s 往前提速平移(实测日志 指令v=+0.26)。一边转
        一边平移会把目标相对方位越拖越偏, 对正变慢甚至对不上。
        现在减速斜率放大到 4.0 m/s²(0.3 m/s 约 75 ms 停住), 加速仍保持 1.2。
        dt 用真实时间差, 不再硬编码 0.05。
        """
        dt = 0.05 if self._pub_t is None else now - self._pub_t
        dt = max(0.005, min(0.20, dt))
        self._pub_t = now
        # P80: 盲走禁令。放在梯形限幅**之前**, 这样是"刹车"而不是"瞬断"。
        # 只压正向: 后退让距、转向找人都要允许。0.30s 在 10Hz 检测下
        # 相当于"连续 3 帧没检出", 单帧漏检不受影响。
        _blind_s = float(self.cfg.get('blind_v_s', 0.30))
        _lvd = (now - self._vis_mono) if self._vis_mono > 0 else 1e9
        # allow_gt_track(仅对照用)没有视觉, 不能被这条禁令一起冻住
        # P81: 绕视(PEEK)必须豁免盲走禁令 —— 绕视的前提就是"看不见人",
        # 被这条禁令一冻, 机器人只能原地转(实测 6s 绕视位移 0.00m)。
        # 安全由三处保证: ① 方向先过 8 扇区净空(peek_pass_m)才走;
        # ② 速度只有 peek_v(0.35m/s); ③ 仍然过 _safety_gate。
        if _blind_s > 0.0 and v_t > 0.0 and _lvd > _blind_s \
                and not self.cfg['allow_gt_track'] \
                and self._track != 'PEEK':
            v_t = 0.0
            self._blind_win.append(now)
        while self._blind_win and now - self._blind_win[0] > 1.0:
            self._blind_win.pop(0)
        # P78: 受阻解除后的前进软启动(只压正向, 后退不受影响)
        _soft = float(self.cfg.get('resume_soft_s', 0.0))
        if self._resume_t0 > 0.0 and _soft > 0.0:
            _el = now - self._resume_t0
            if _el < _soft:
                if v_t > 0.0:
                    v_t *= max(0.0, _el) / _soft
            else:
                self._resume_t0 = 0.0
        a_up = 1.2 * dt          # 加速度上限 1.2 m/s²
        a_dn = 4.0 * dt          # 减速度上限 4.0 m/s²
        dv = v_t - self.v
        if self.v * v_t <= 0.0 or abs(v_t) < abs(self.v):
            lim = a_dn           # 减速 / 反向(先刹过零) → 用大斜率
        else:
            lim = a_up
        self.v += max(-lim, min(lim, dv))
        if v_t == 0.0 and abs(self.v) < 0.01:
            self.v = 0.0         # 掐掉残余, 避免对正段长期挂着小速度
        w_t = max(-2.2, min(2.2, w_t))
        # P77 §17-3: 换向率客观指标 —— 只看有幅度的 w(排除噪声过零),
        # 统计的是**最终下发**的符号翻转, 与仲裁器内部计数互相独立。
        if abs(w_t) >= 0.15:
            _ws = 1.0 if w_t > 0.0 else -1.0
            if _ws != self._w_dir_last:
                self._w_dir_last = _ws
                self._w_pub_win.append(now)
        while self._w_pub_win and now - self._w_pub_win[0] > 1.0:
            self._w_pub_win.pop(0)
        self._w_pub_rate = float(len(self._w_pub_win))
        # P36: self.v 是"真实 m/s"(PID/安全层/限速都用它), 下发前换算成
        # 指令值(实测平台 指令 × 0.62 ≈ 实际)。
        self._publish(self.v * float(self.cfg['cmd_gain']), w_t)
        self._last_w = w_t
        self._pub_diag(now)

    def _pub_diag(self, now):
        """P76: 10Hz 导出安全层内部量, 供离线归因。"""
        if now - self._diag_t < 0.10:
            return
        self._diag_t = now
        if self._track == 'PEEK':
            mode = 'PEEK'          # P81: 遮挡期绕视
        elif self._aligning:
            mode = 'ALIGN'
        elif self._avoiding:
            mode = 'AVOID'
        else:
            mode = 'MOVE'
        d = dict(t=round(now, 3), x=round(self.rx, 3), y=round(self.ry, 3),
                 yaw=round(self.ryaw, 3), mode=mode,
                 v=round(self.v * float(self.cfg['cmd_gain']), 3),
                 w=round(self._last_w, 3),
                 dist=round(self._last_dist, 3),
                 brg=round(math.degrees(self._last_brg), 1),
                 td=self.target_d, note=self._last_note)
        # P77: 转向仲裁/滞回/方位限幅的实测指标
        d['wown'] = self._w_owner          # 当前掌握转向的模块
        d['wfr'] = round(self._w_pub_rate, 2)    # 最近 1s 下发 w 换向次数
        d['wfa'] = round(self._w_flip_rate, 2)   # 仲裁器内部换向次数
        d['bclip'] = round(self._brg_clip_rate, 2)  # 近 1s 方位被限幅占比
        d['dt'] = 1 if now < self._detour_until else 0   # 定向绕行窗口
        d['dtd'] = round(self._detour_dir, 0)
        d['sl'] = round(self._slow_fac, 2)   # P79: 接近减速系数(1=不减速)
        d['bl'] = len(self._blind_win)       # P80: 近 1s 盲走禁令触发次数
        d['pk'] = int(self._peek_n)          # P81: 遮挡绕视累计次数
        d['pkd'] = (round(now - self._peek_t0, 2)
                    if self._track == 'PEEK' else 0.0)
        d['hs'] = round(now - self._hold_t0, 2) if self._hold_t0 > 0 else 0.0
        for k, val in self._dbg.items():
            d[k] = round(val, 3) if isinstance(val, float) else val
        try:
            self._diag_pub.publish(String(data=json.dumps(d)))
        except Exception:
            pass

    def _log(self, now, src, dist, pspd, rspd, w_t, brg, note, nscan,
             rx, ry, px, py):
        self._last_note = note            # P53: 供 /follower/status 诊断
        self._last_dist = dist            # P76: 供 /follower/diag
        self._last_brg = brg if brg is not None else 0.0
        if now - self._last_log > 0.5:
            self._last_log = now
            if self._aligning:
                mode = '对正'
            elif src == '扫描':
                mode = '扫描'
            elif src == '搜索':
                mode = '前往'
            elif src == '丢失':
                mode = '丢失'
            else:
                mode = '跟踪'
            self.get_logger().info(

                f'FOLLOWER [{mode}|{src}] 距离={dist:.2f}m'
                f'(目标{self.target_d:.2f}m) 目标方位={math.degrees(brg):+.0f}°'
                f' 人速={pspd:.2f} 指令v={self.v:+.2f} 实际v={rspd:.2f} '
                f'w={self._last_w:+.1f} {note}'
                f' |ff={self._dbg["ff"]:+.2f}'
                f' pid={self._dbg["pid"]:+.2f}'
                f' cap={self._dbg["cap"]:+.2f}'
                f' dh={self._dbg["dh"]:.2f}'
                f' df={self._dbg["df"]:.2f}'
                f' err={self._dbg.get("err", 0.0):+.2f}'
                f' vr={self.robot_vfwd:+.2f} pv={self._pvvr:+.2f}')
        if now - self._last_diag > 4.0:
            self._last_diag = now
            self.get_logger().info(
                f'FOLLOWER 诊断 雷达帧={nscan} 机器人=({rx:+.2f},{ry:+.2f},'
                f'{math.degrees(self.ryaw):+.0f}°) 人=({px:+.2f},{py:+.2f})')

    def _publish(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)
        self._publish_status()

    def _publish_status(self):
        """P32: 对外状态快照(5Hz 限流), 供评测脚本/上位机消费"""
        now = self._now()
        if now - self._pub_status_t < 0.2:
            return
        self._pub_status_t = now
        vv = self.vis or {}
        det = bool(vv.get('detected'))
        st = {'track': self._track, 'src': self._src_prev,
              'lost': bool(self._lost),
              'lost_s': round(now - self._lost_at, 2) if self._lost else 0.0,
              'scan_round': int(self._scan_round),
              'detected': det,
              'cam': str(vv.get('cam', '')) if det else '',
              'vis_dist': (round(float(vv.get('dist', -1.0)), 3)
                           if det else -1.0),
              'bearing_deg': (round(math.degrees(float(vv.get('bearing', 0.0))), 1)
                              if det else None),
              'goto': ([round(float(self._search_target[0]), 2),
                        round(float(self._search_target[1]), 2)]
                       if (self._track == 'GOTO' and self._search_target)
                       else None),
              'cmd_v': round(float(self.v), 3),
              'cmd_w': round(float(self._last_w), 3),
              'note': getattr(self, '_last_note', '')}
        m = String()
        m.data = json.dumps(st)
        self._status_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = Follower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
