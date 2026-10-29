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
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

import gz.transport13 as gz_transport
from gz.msgs10.pose_v_pb2 import Pose_V as GzPose_V

# ---- 跟踪配置(P15, 与视觉节点共用同一配置文件) ----
CONFIG_PATH = os.path.join(
    os.environ.get('VLA_HOME', '/home/earl/workspace/VLA'),
    'vla_demo', 'vision_config.json')

DEFAULT_CFG = {
    'track_dist_m': 1.0, 'track_dist_tol_m': 0.10,
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
    'vis_bias_m': 0.32,             # P38: 0.25→0.32(运动中位实测 -0.32)
                                    #      (实测 1~5m 偏差 -0.25 极稳)
    'pid_kp_v': 1.90, 'pid_ki_v': 0.06, 'pid_kd_v': 0.04,   # P43
    'pid_kp_w': 2.2, 'pid_ki_w': 0.40, 'pid_kd_w': 0.30,
    # P20: 对正(align)参数
    'aim_band_deg': 15.0,           # 目标允许偏离 front 中线的角度带
    'recenter_kp': 2.0,             # 对正比例增益 (rad/s per rad)
    'turn_min_w': 0.35,             # 对正最小角速度(克服静摩擦)
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
        self.pub = self.create_publisher(Twist, '/robot/cmd_vel', 10)
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
        # P45: GOTO 进度监视 —— 目标点被障碍卡住时提前退出, 别死等超时
        self._goto_best = None        # 历史最近目标点距离
        self._goto_prog_t = 0.0       # 最后一次"确实靠近了"的时刻
        self._lost_announced = False  # 是否已提示"目标丢失"
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
        # 对正状态(P20)
        self._aligning = False
        self._align_dir = 0.0     # ±180° 附近的方向锁存
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
        self._blk_t0 = 0.0            # 前方持续受阻的起始时刻(0=未受阻)
        # P46: 锁存式绕障状态
        self._avoiding = False        # 是否处于绕行态
        self._avoid_side = 1.0        # +1 左绕 / -1 右绕(进入时锁死, 不重算)
        self._avoid_d0 = 0.0          # 进入绕行时的目标距离(Bug2 的基准)
        self._avoid_t0 = 0.0          # 进入绕行时刻
        self._blk2_t0 = 0.0           # 绕障触发计时(与脱困用的 _blk_t0 分开)
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
            self.target_d = float(d) if d is not None \
                else self.cfg['track_dist_m']
            self.state = 'FOLLOW'
            self._int_d = 0.0
            self._prev_err_d = 0.0
            self.get_logger().info(
                f'FOLLOWER ★ 收到跟随指令(来自语音: "{cmd.get("text", "")}"), '
                f'目标距离 = {self.target_d:.2f}m — 开始跟随')
        elif action == 'STOP':
            self.state = 'IDLE'
            self.v = 0.0
            self._publish(0.0, 0.0)
            self.get_logger().info(
                f'FOLLOWER ★ 收到停止指令(来自语音: "{cmd.get("text", "")}"), '
                f'已停止跟随')
        else:
            self.get_logger().info(
                f'FOLLOWER 忽略指令: {json.dumps(cmd, ensure_ascii=False)}')

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
                inst = math.hypot(robot.position.x - prx,
                                  robot.position.y - pry) / (stamp - pt)
                self.robot_speed = 0.6 * self.robot_speed + 0.4 * min(inst, 2.0)
            if self._p_prev is not None and stamp > self._p_prev[0] + 1e-3:
                pt, ppx, ppy = self._p_prev
                inst = math.hypot(person.position.x - ppx,
                                  person.position.y - ppy) / (stamp - pt)
                self.person_speed = 0.6 * self.person_speed + 0.4 * min(inst, 2.0)
            self._r_prev = (stamp, robot.position.x, robot.position.y)
            self._p_prev = (stamp, person.position.x, person.position.y)
        self._last_pose_mono = time.monotonic()

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
                         float(msg.angle_increment), time.monotonic())

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
        self._prox_rx = time.monotonic()      # P30 看门狗

    def _vis_cb(self, msg):
        """相机人检测: {detected, cam, dist, bearing, bearing_cam, u, area}"""
        try:
            v = json.loads(msg.data)
        except Exception:
            return
        self.vis = v
        self._vis_msg_mono = time.monotonic()
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

    def _enter_scan(self, now, announce=False):
        """P32: 进入原地 360° 扫描态; announce=True 时明确提示"目标丢失" """
        self._track = 'SCAN'
        # P45: 12s 是 2.3 圈(转速 1.2rad/s 时一圈 5.2s), 纯浪费
        self._scan_until = now + float(self.cfg.get(
            'scan_dur_s2', self.cfg['scan_dur_s']))
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
        now = time.monotonic()
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
        now = time.monotonic()
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
            px = rx + math.cos(ryaw + bearing) * vdist
            py = ry + math.sin(ryaw + bearing) * vdist
            self._last_vis_xy = (px, py)      # 只由视觉刷新
            # P40: 由视觉世界坐标差分估计人速(前馈源)。
            # 注意两个坑: ① 跨相机切换时 bearing 会跳变, px/py 瞬间跳一下,
            # 差分出来是几十 m/s 的假峰, 必须按 |inst| 上限剔除;
            # ② 视觉 10Hz, 但 bearing 量化到 ~1°, 1m 处相当 1.7cm 抖动,
            # 所以用 EWMA(0.35) 压, 且 dt 太短/太长都不采样。
            _v_inst = None
            if self._pv_t is not None:
                dtp = now - self._pv_t
                if 0.09 < dtp < 0.60:
                    _v_inst = math.hypot(px - self._pv_x,
                                         py - self._pv_y) / dtp
                    if _v_inst <= 2.0:        # 剔除切换跳变造成的假峰
                        a = 0.35
                        self._pvx = (1 - a) * self._pvx + \
                            a * (px - self._pv_x) / dtp
                        self._pvy = (1 - a) * self._pvy + \
                            a * (py - self._pv_y) / dtp
            if _v_inst is None or _v_inst > 2.0 or (now - self._pv_t) > 0.60:
                self._pv_t, self._pv_x, self._pv_y = now, px, py
            else:
                self._pv_t, self._pv_x, self._pv_y = now, px, py
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
            if (self._track == 'TRACK' and not self.cfg['allow_gt_track']
                    and lost_dt > self.cfg['lost_grace_s']):
                if not self._lost:
                    self._lost = True
                    self._lost_at = now
                n_hist = min(len(self.hist), 10)
                if self._last_vis_xy is None or n_hist == 0:
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
                    else:
                        self._enter_scan(now, announce=False)
                else:
                    self._enter_scan(now, announce=True)

        # --- 2. 搜索/扫描状态机 (P32) ---
        if self._track == 'GOTO':
            gx, gy = self._search_target
            _gd = math.hypot(gx - rx, gy - ry)
            # P45: 到点阈值原硬编码 0.5m, 比安全层 stop_d(0.66) 还小 ——
            # 目标点在障碍旁边时数学上永远不可达, 只能等 20s 超时。
            if _gd < float(self.cfg['goto_reach_m']):
                self.get_logger().info(
                    'FOLLOWER 已到达人最后出现位置(%+.1f,%+.1f) 距离%.2fm, '
                    '原地 360° 扫描确认' % (gx, gy, _gd))
                self._enter_scan(now, announce=False)
            elif now - self._search_t0 > self.cfg['goto_timeout_s']:
                self.get_logger().warn(
                    'FOLLOWER 前往最后位置超时(%.0fs, 还差%.2fm), 转原地扫描'
                    % (self.cfg['goto_timeout_s'], _gd))
                self._enter_scan(now, announce=False)
            elif (self._goto_best is not None
                  and now - self._search_t0 > 2.0
                  and now - self._goto_prog_t
                  > float(self.cfg['goto_stall_s'])):
                # P45: 3s 内没能再靠近 8cm → 目标点被障碍/墙挡住, 过不去。
                # 与其静默死等 12s, 不如立刻转扫描 —— 人在动, 原地转一圈
                # 看到人的概率远高于对着障碍硬推。
                self.get_logger().warn(
                    'FOLLOWER 前往最后位置受阻(%.1fs 无进展, 还差%.2fm), '
                    '就地转扫描' % (now - self._search_t0, _gd))
                self._enter_scan(now, announce=False)
            else:
                if self._goto_best is None or _gd < self._goto_best - 0.08:
                    self._goto_best = _gd
                    self._goto_prog_t = now
                px, py = gx, gy
                brg = wrap(math.atan2(gy - ry, gx - rx) - ryaw)
                src = '搜索'
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
                else:
                    self._enter_scan(now, announce=True)
            src = '扫描'

        if src != self._src_prev:
            if '真值' in (src, self._src_prev) or '视觉' in (src, self._src_prev):
                self.get_logger().info(f'FOLLOWER 感知源切换 → {src}')
            self._src_prev = src

        dist = math.hypot(px - rx, py - ry)

        # --- 3. 对正判定(P20): 目标是否落在 front 中线 ±band 内 ---
        aim = abs(brg)
        if self._aligning:
            # 退出对正需进入 0.65*band 内, 防边界抖动
            if aim <= band * 0.65:
                self._aligning = False
                self.get_logger().info(
                    f'FOLLOWER ✓ 已对正目标(偏差 {math.degrees(brg):+.1f}°'
                    f' ≤ {self.cfg["aim_band_deg"]:.0f}°), 转入距离跟踪')
        else:
            if aim > band:
                self._aligning = True
                self.get_logger().info(
                    f'FOLLOWER ⟳ 目标偏离中线 {math.degrees(brg):+.1f}°'
                    f' > {self.cfg["aim_band_deg"]:.0f}°, 先原地对正')
        # 扫描态固定小角速度旋转
        if src == '扫描':
            self._aligning = False

        v_t = 0.0
        w_t = 0.0
        note = ''

        if src == '扫描':
            w_t = self.cfg['recenter_rot_speed'] * 0.8
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
            # P22: 偏差不大(≤34°)时允许"边走边转"。纯原地旋转 v=0 会让逃跑的
            # 目标越拉越远, 实测 90s 内进出对正 44 次 —— 每次进对正就停车,
            # 目标跑开 → 又进对正, 形成"走走停停追不上"。大偏差(目标在左右/
            # 后相机)仍保持纯旋转, 因为那时前方是空的, 边走会撞。
            if aim < math.radians(34.0):
                # P36: 0.60 → 0.85。对正段占了追踪时间的一半以上,
                # 0.6 倍速度让机器人在"边转边走"时明显掉队。
                v_t = 0.85 * self.cfg['approach_speed']
            else:
                v_t = 0.0
            note = (f'对正{math.degrees(brg):+.0f}°'
                    f'{"(边转边走)" if v_t > 0 else ""}'
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
            err_d = dist - 0.45
            guard_d = 0.30
        else:
            err_d = dist - self.target_d
            # 安全距(低于此值必须后退), 下面限速也要用
            guard_d = max(0.55, self.target_d - tok - 0.25)
        tol = tok
        dtc = 0.05
        # P40: 人速前馈 —— 把人沿车头方向的速度分量直接叠到速度指令上。
        # 这样"跟着走"由前馈提供, "距离对不对"由误差环修正, 两者解耦;
        # 否则 kp 小则稳态差 0.5m(kp=0.6 时的实测), kp 大则过冲抖。
        _vfwd = self._pvx * math.cos(ryaw) + self._pvy * math.sin(ryaw)
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
        v_lat = room / lat
        v_cap = min(max(self.cfg['approach_speed'], v_ff), v_lat)
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
        v_t = max(-0.35, min(v_cap, v_pid))
        # 保持航向: 目标留在 ±band 内
        # P22: 增益从 hold_kp(1.6)/上限0.9 提到 2.2/1.3 —— 目标以 0.7m/s 横移、
        # 距离 1m 时角速度需求达 0.7 rad/s, 原上限 0.9 贴着极限, 稍有滞后就
        # 掉回对正段, 造成每 2s 一次模式抖动。实测被控对象 w 跟踪精度 1.00,
        # 放大增益是安全的。
        if abs(brg) > math.radians(3.0):
            w_t = max(-1.3, min(1.3, self.cfg['hold_kp'] * brg))
        else:
            w_t = 0.0

        # --- 4. 绕障(P46): 锁存式 Bug2 ---
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
                _clear = (_df > float(self.cfg['lidar_slow_m'])
                          * float(self.cfg['avoid_exit_v'])
                          and min(_dfl, _dfr) > 0.85)
                _tmo = now - self._avoid_t0 > float(self.cfg['avoid_max_s'])
                if (_clear and dist < self._avoid_d0 - 0.05) or _tmo:
                    self._avoiding = False
                    self._blk2_t0 = 0.0
                    self.get_logger().info(
                        'FOLLOWER ✓ 绕障退出(历时%.1fs, %s, 目标%.2fm) → 恢复指向'
                        % (now - self._avoid_t0,
                           '超时换边' if _tmo else '前方畅通', dist))
                else:
                    _avoid_now = True

        if _avoid_now:
            _side_room = _room_l if self._avoid_side > 0 else _room_r
            _aim = wrap(brg + math.radians(float(self.cfg['avoid_deg']))
                        * self._avoid_side)
            w_t = max(-1.7, min(1.7, 2.4 * _aim))
            _rm = min(_df, _side_room)
            # 侧方越窄走得越慢, 但不允许停下 —— 一停就退回"原地转"
            v_t = min(float(self.cfg['approach_speed']) * 0.72, 0.40,
                      max(float(self.cfg['avoid_min_v']),
                          (_rm - _stop + 0.12) / 0.45))
            _dn = ('绕障%s 前%.2f 侧%.2f 目标%.2f'
                   % ('左' if self._avoid_side > 0 else '右',
                      _df, _side_room, dist))
            note = _dn
        else:
            self._det_dir = 0.0
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
                _g_hit = None
                _g_lat = 0.0
                for _g in GEO:
                    # 用"向前 look 之后"的位置算真实间隙, 留 0.10m 余量
                    if geo_gap(nx, ny, _g) > ROBOT_R + 0.10:
                        continue
                    _fwd = ((_g[1] - rx) * math.cos(ryaw)
                            + (_g[2] - ry) * math.sin(ryaw))
                    if _fwd < -0.05:
                        continue                  # 已经在身后
                    _lat = (-(_g[1] - rx) * math.sin(ryaw)
                            + (_g[2] - ry) * math.cos(ryaw))
                    _ext = _g[3] if _g[0] == 'cyl' else math.hypot(_g[3], _g[4])
                    if abs(_lat) > _ext + ROBOT_R:
                        continue                  # 侧向净空够, 能擦过去
                    _g_hit, _g_lat = _g, _lat
                    break
                if _g_hit is not None:
                    v_t = 0.0
                    # P45 关键: 必须给转向。停车 + w=0 = 永久死锁。
                    w_t = -1.10 if _g_lat > 0 else 1.10
                    note = (note + ' 真值硬保(%+.1f,%+.1f)%s'
                            % (_g_hit[1], _g_hit[2],
                               '右绕' if _g_lat > 0 else '左绕')).strip()

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
        _d_diag = min(d_fl, d_fr)
        d_head = min(d_f, _d_diag * 0.72) if _d_diag < 0.85 else d_f

        if v_t > 0.02:                                  # ---- 前进 ----
            if d_head <= stop_d:
                v_t = 0.0
                # P46: 绕障态的车头已经偏转 avoid_deg 朝向侧方, 这里的
                # d_head 量的是**侧方**余量。若再另起一套 esc_dir 转向,
                # 会与绕障的转向反向抵消 —— 实测 w 在 +1.50/-1.15 之间
                # 每 0.5s 翻一次, 机器人原地抖 28s 只挪 0.3m。
                if self._avoiding:
                    note = f'绕障中受阻 前{d_head:.2f}'
                # P27: 急停时的转向必须与上层意图协同。原来是"哪侧空就
                # 往哪转", 与上层"目标在右就右转"直接对打 —— 实测 w 在
                # -1.0 -> -1.4 -> +1.1 之间每帧翻号, 车在原地抖。
                room_l, room_r = d_fl, d_fr
                want = 0.0
                if w_t > 0.05 and room_l > stop_d * 0.92:
                    want = 1.0                       # 想左转且左前有余量
                elif w_t < -0.05 and room_r > stop_d * 0.92:
                    want = -1.0                      # 想右转且右前有余量
                if self._avoiding:
                    pass                          # 绕障自己管转向, 这里不插手
                else:
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
                            f' 绕{"左" if self._esc_dir > 0 else "右"}')
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
                if d_wing < stop_d:
                    # P35: 原实现"限速到 0.10 继续挤"在实测中会慢慢钻进
                    # 立柱(0.10m/s × 15s = 1.5m, gap 走到 -0.42)。侧翼已
                    # 进入停车距离时必须停止前进, 并强制往空的一侧转。
                    v_t = 0.0
                    if abs(w_t) < 0.35:
                        w_t = 0.9 if d_r < d_l else -0.9
            if d_head > stop_d:
                self._blk_t0 = 0.0                      # 已脱离受阻
        elif v_t < -0.05:                               # ---- 后退 ----
            # "防撞人"和"卡死脱困"都会下发负速度, 原来这两条路径完全不看
            # 后方 —— E 场景擦碰就是倒车撞上背后的立柱
            if d_b < stop_d:
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
        # P36: self.v 是"真实 m/s"(PID/安全层/限速都用它), 下发前换算成
        # 指令值(实测平台 指令 × 0.62 ≈ 实际)。
        self._publish(self.v * float(self.cfg['cmd_gain']), w_t)
        self._last_w = w_t

    def _log(self, now, src, dist, pspd, rspd, w_t, brg, note, nscan,
             rx, ry, px, py):
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
                f'w={self._last_w:+.1f} {note}')
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
        now = time.monotonic()
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
              'cmd_w': round(float(self._last_w), 3)}
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
