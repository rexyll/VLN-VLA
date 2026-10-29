#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""person_walk.py —— "人"的随机行走控制器

行为: 启动 3 秒后开始在 12m x 12m 工作区内随机行走,
     速度在 0.25~0.65 m/s 间随机变化, 随机转向, 偶尔明显减速/停顿;
     靠近墙壁或障碍物时进入 ESCAPE 状态径直走回场地中心,
     长时间没有位移(被卡住)时也强制逃逸。
"""
import json
import math
import os
import random
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from vla_follow import scene_layout as L

# P91: gz 真值(可选)。与 follower 同源: `/world/<w>/dynamic_pose/info`。
try:
    import gz.transport13 as gz_transport
    from gz.msgs10.pose_v_pb2 import Pose_V as GzPose_V
    HAVE_GZ = True
except Exception:                                     # pragma: no cover
    HAVE_GZ = False

ARENA_HALF = 5.3   # 工作区内边界(墙内侧 6.0m, 留出人体半径)
WALL_MARGIN = 0.35  # 距墙多少米算危险
# (x, y, 危险半径) 与 world 中障碍物一致, 含人体半径余量(收紧以减少逃逸循环)
# P22 修正: 只保留 work_area.sdf 里真实存在的 3 根立柱。
# 原表有 4 项在世界中没有任何几何体, 人一走进这些"幽灵危险区"就转 ESCAPE
# 并原地抖动(实测位置长时间卡在 (+1.00,+0.00) 不动)。
# 半径 = 立柱半径 + 0.2m 人体余量。
# P24 修正: 与 follower 同步为 SDF 里真实存在的 7 个障碍(半径 +0.2m 人体余量)
OBSTACLES = [
    (3.5, 2.0, 0.62),
    (-3.0, 3.2, 0.72),
    (0.5, -3.2, 0.60),
    (-4.2, -2.5, 0.70),
    (4.2, -3.0, 0.62),
    (-1.5, -4.5, 0.76),
    (2.2, 4.6, 0.55),
    # P90: 桌子 0.94x0.66 @ (2.60, 0.35)。外接圆 0.574 + 0.2 人体余量。
    # 接水位 PERSON_SPOT=(2.169,1.169) 距桌心 0.926m > 0.774m, 人站那儿是安全的;
    # 但随机游走时必须绕开, 否则会从桌面里穿过去。
    (2.60, 0.35, 0.77),
]


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def in_danger(x, y, margin=0.0):
    """当前位置是否贴墙/撞障碍(margin 为额外余量)"""
    if abs(x) > ARENA_HALF - WALL_MARGIN + margin or \
            abs(y) > ARENA_HALF - WALL_MARGIN + margin:
        return True
    for (ox, oy, r) in OBSTACLES:
        if math.hypot(x - ox, y - oy) < r + margin:
            return True
    return False


class PersonWalk(Node):

    def __init__(self):
        super().__init__('person_walk')
        # P74: 时间基准 —— 优先 /clock, 回退 wall clock
        self._t0_wall = time.monotonic()
        self._t0_sim = None
        self._dt_prev = None
        self.pub = self.create_publisher(Twist, '/person/cmd_vel', 10)
        self.create_subscription(Odometry, '/person/odom', self._odom_cb, 10)
        self.create_subscription(Odometry, '/robot/odom', self._robot_cb, 10)
        # ---- P91: gz 真值 ----
        # 差速里程计是**轮子积分**: 走 1.2m + 转两次弯后与真值差 20cm(实测
        # odom 认 (2.15,1.12) 已"到位"时, 真值还在接水位外 206mm)。随机行走
        # 无所谓, 但 GOTO 要的是"人站到 PERSON_SPOT"——递杯位按那一点算,
        # 站偏了杯子就递不到手边。所以 GOTO 段改用真值闭环。
        # 真值只在这一个状态里用: 其余行为仍走 odom, 保持"人只知道自己的
        # 里程"这一设定, 免得随机行走意外获得上帝视角。
        self._gt = {}
        self._gt_t = 0.0
        self._gt_logged = False
        # P91: GOTO 绕障 —— 记下"从哪一侧绕", 一旦选定就别在对称处左右抖
        self._avoid_side = 0
        self._stuck_xy = None
        self._stuck_t = 0.0
        if HAVE_GZ:
            try:
                self._gz = gz_transport.Node()
                self._gz.subscribe(GzPose_V,
                                   '/world/follow_world/dynamic_pose/info',
                                   self._gt_cb)
            except Exception as e:                    # pragma: no cover
                self.get_logger().warn('真值订阅失败, GOTO 退回里程计: %s' % e)
        # ---- P75: 开环回放 / 录制 (A/B 对照用) ----
        # 回放模式把"人"变成时间的纯函数: 按录制轴下发 (v,w), 完全不看机器人。
        # 录制模式记录每拍实际下发的 (tau, v, w), tau 以本节点仿真时钟起点为零。
        self._rec_fp = None
        self._rp = None
        self._rp_i = 0
        self._rp_off = 0.0
        self._rp_log = -9.0
        self._rec_path = os.environ.get('PW_RECORD', '').strip()
        self._rp_path = os.environ.get('PW_REPLAY', '').strip()
        if self._rec_path and self._rp_path:
            print('[PERSON] PW_RECORD 与 PW_REPLAY 同时设置, 忽略 PW_RECORD',
                  flush=True)
            self._rec_path = ''
        self._rec_hdr_done = False
        self._rec_t0 = None
        if self._rec_path:
            self._rec_fp = open(self._rec_path, 'w', encoding='utf-8')
            self.get_logger().info('PERSON 录制模式 -> %s' % self._rec_path)
        if self._rp_path:
            self._rp, self._rp_hdr = self._load_replay(self._rp_path)
            self._rp_t0 = None
            self._rp_t0_logged = False
            if self._rp_hdr and 't0_abs' in self._rp_hdr:
                self.get_logger().info(
                    'PERSON 回放头: t0_abs=%.3fs seed=%s'
                    % (float(self._rp_hdr['t0_abs']),
                       self._rp_hdr.get('seed', '')))
            else:
                self.get_logger().warn(
                    'PERSON 回放文件无 t0_abs(旧格式), 回退为节点相对时间轴')
            try:
                self._rp_off = float(os.environ.get('PW_REPLAY_OFFSET', '0') or 0.0)
            except ValueError:
                self._rp_off = 0.0
            self.get_logger().info(
                'PERSON 回放模式 <- %s (%d 拍, offset=%.3fs)'
                % (self._rp_path, len(self._rp), self._rp_off))
        self._lock = threading.Lock()
        self.x, self.y, self.yaw = 1.0, 0.0, 0.0
        self.rx, self.ry = 0.0, 0.0  # 机器人位置(动态避让)
        self.state = 'WAIT'
        self.state_until = self._now() + 3.0
        self.speed = 0.0
        self.target_speed = 0.0
        self.turn_rate = 0.0
        self._last_pos_log = 0.0
        self._vmod_t = 0.0          # P33: WALK 段内速度抖动计时
        # ---- P90: "去喝水" 指令链路 ----
        # 外部(脚本/语音)发 /person/command {"action":"GO_DRINK"}, 人就走过去
        # 站定、面向机器人、说那句"我要喝水", 然后静止等人把水递过来。
        self.pub_voice = self.create_publisher(String, '/voice/request', 10)
        self.create_subscription(String, '/person/command', self._cmd_cb, 10)
        self._drink_target = None
        self._drink_said = False
        self._face_yaw = None        # P95: 到位后"面向机器人"的朝向快照
        self._arrived = False        # P95: "已经走到接水位"的锁存(只判一次)
        self._drink_t0 = 0.0
        self.DRINK_TIMEOUT = 300.0   # 递水整链 ~4min, 超时就回去随机走
        # ---- P94: 接水位扰动(验证"人站偏了还能不能递到手边") ----
        # 指令里带 jitter 时, 目标 = PERSON_SPOT + d*(cos a, sin a), 站定后
        # 的朝向再叠一个 yaw 偏置。默认 0 = 严格站到 PERSON_SPOT(原行为)。
        # 用途: 端到端验收时把"人站得准"这个前提拿掉, 看 drink_task 的
        # `_present_target`(朝实测人手递) + `_approach_if_far`(底盘补位)
        # 兜不兜得住。演示/日常跑不要带这个字段。
        self._drink_yaw_off = 0.0
        self._goto_t0 = 0.0
        self.GOTO_TIMEOUT = 120.0    # 走不到接水位就放弃(正常 4~5m 约 20s)
        # ---- P95: 喝水是"人的随机自发行为", 不是脚本喂出来的 ----
        # 平时机器人一直在跟随; 人自己想喝水了才走到桌边开口 → 机器人让位
        # 做倒水/递水/接回/放回 → 撤离桌边 → 交还底盘 → 继续跟随。手动发
        # GO_DRINK 只是"替他决定一次"(演示/验收用)。
        #   关掉: PW_AUTO_DRINK=0;  调间隔: PW_DRINK_GAP_MIN/MAX(秒)
        self._auto_drink = os.environ.get('PW_AUTO_DRINK', '1').strip() \
            not in ('0', 'false', 'no', 'off')
        self._drink_gap = (float(os.environ.get('PW_DRINK_GAP_MIN', '90')),
                           float(os.environ.get('PW_DRINK_GAP_MAX', '180')))
        self._next_drink_t = self._now() + \
            float(os.environ.get('PW_DRINK_FIRST', '45'))
        # 每次站的位置带一点随机偏差(人不会两次站得一模一样)。这同时也把
        # P94 的鲁棒性兜底(朝实测人手递杯 + 臂伸不够底盘补)放进日常演示里
        # 持续跑, 而不是只在验收脚本里跑一次。
        #   关掉: PW_DRINK_JITTER=0
        self._jitter_m = float(os.environ.get('PW_DRINK_JITTER', '0.06') or 0.0)
        self._jitter_deg = float(
            os.environ.get('PW_DRINK_JITTER_DEG', '8') or 0.0)
        self._auto_drink_n = 0
        if self._auto_drink:
            self.get_logger().info(
                'PERSON 喝水随机触发: 首次 %.0fs, 之后间隔 %.0f~%.0fs, '
                '站偏 ±%.2fm/±%.0f°'
                % (float(os.environ.get('PW_DRINK_FIRST', '45')),
                   self._drink_gap[0], self._drink_gap[1],
                   self._jitter_m, self._jitter_deg))
        # ---- P93: 喝完水离开桌边 ----
        # 杯子递回给机器人之后, 人先走到桌外(L.PERSON_LEAVE_SPOT)再恢复
        # 随机活动。不能原地直接切 WALK: 人还站在接水位(距服务位 0.92m),
        # 一进 WALK 就会被 robot_close(0.70m) 判"机器人贴脸"触发 ESCAPE,
        # 等于刚喝完水就被机器人顶着跑。
        self._leave_target = None
        self._leave_t0 = 0.0
        self.LEAVE_TIMEOUT = 90.0    # 走 2.6m 最多 ~15s, 90s 兜底
        # 逃逸与防卡死
        self._clear_since = None
        self._progress_pos = (1.0, 0.0)
        self._progress_t = self._now()
        self.timer = self.create_timer(0.05, self._tick)  # 20 Hz
        self.get_logger().info('PERSON "人"节点就绪, 3 秒后开始在工作区内随机行走')

    def _now(self):
        """P74: 当前时间(s), 仿真时钟优先。"""
        try:
            ns = int(self.get_clock().now().nanoseconds)
        except Exception:
            ns = 0
        if ns > 0:
            if self._t0_sim is None:
                self._t0_sim = ns * 1e-9
            return ns * 1e-9 - self._t0_sim
        return time.monotonic() - self._t0_wall

    def _now_abs(self):
        """P75b: 绝对仿真时间(s)。无仿真时钟时返回 None(该拍不录/不动)。"""
        try:
            ns = int(self.get_clock().now().nanoseconds)
        except Exception:
            ns = 0
        return ns * 1e-9 if ns > 0 else None

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self.x, self.y, self.yaw = p.x, p.y, yaw

    def _robot_cb(self, msg):
        p = msg.pose.pose.position
        with self._lock:
            self.rx, self.ry = p.x, p.y

    def _avoid_dir(self, x, y, th, d_goal, now):
        """通往目标的方向 `th` 上若挡着障碍, 改走它的**切向**。

        GOTO 不能复用随机行走那套 ESCAPE(一触发就往场地中心跑, 永远到不了
        桌边), 但也不能完全不避障 —— 实测人从 (3.50,-1.70) 直线奔向
        (2.17,1.17) 正好穿过桌子, 顶在桌沿上一动不动 400 多秒。

        这里用最朴素的切向绕行: 场景里的障碍都是**凸**的(柱子/桌子), 沿
        表面走一定能绕过去。绕行侧一旦选定就锁住(`_avoid_side`), 否则在正
        对障碍的对称位置会左右横跳。
        """
        # "挡路"的判据必须同时满足两条, 缺一不可:
        #   ① 障碍到**人→目标这条线段**的垂距 < r + 余量(真的在路中间)
        #   ② 垂足落在[人, 目标]区间内(而不是已经走过去了/还在身后)
        # 只按"方位角接近"判会把**目标附近的侧向障碍**也算成挡路: 实测人
        # 已到 (2.07,1.49)、距接水位仅 0.33m, 东南侧的桌子仍被判挡路, 于是
        # 它掉头绕桌, 在 0.33~0.83m 之间来回震荡 400 秒也进不了 0.06m。
        tx, ty = math.cos(th), math.sin(th)
        cand = None
        for (ox, oy, r) in OBSTACLES:
            ax, ay = ox - x, oy - y
            da = math.hypot(ax, ay)
            if da > r + 0.75:                 # 太远, 不管
                continue
            t = ax * tx + ay * ty             # 沿去路方向的投影
            if t < -r or t > d_goal + r:      # 不在"人↔目标"区间内
                continue
            perp = abs(-ax * ty + ay * tx)    # 到线段的垂距
            if perp > r + 0.25:               # 没挡在路中间
                continue
            if cand is None or t < cand[0]:   # 取**最先挡路**的那个
                cand = (t, ax, ay, da, r)
        if cand is None:
            self._avoid_side = 0
            self._stuck_xy = None
            return th, False
        _, ax, ay, da, r = cand
        if da < r + 0.15:                     # 已经贴上了: 先径向退出来
            return math.atan2(-ay, -ax), True
        # ---- 卡住兜底: 3s 内挪不到 3cm 就换一侧绕(切向选错了会顶住)
        if self._stuck_xy is None:
            self._stuck_xy = (x, y, now)
        elif math.hypot(x - self._stuck_xy[0], y - self._stuck_xy[1]) < 0.03:
            if now - self._stuck_xy[2] > 3.0:
                self._avoid_side = -self._avoid_side if self._avoid_side else -1
                self._stuck_xy = (x, y, now)
                self.get_logger().info('PERSON 去喝水被挡住, 换一侧绕行')
        else:
            self._stuck_xy = (x, y, now)
        if self._avoid_side == 0:
            t1 = math.atan2(ay, ax) + math.pi / 2
            t2 = math.atan2(ay, ax) - math.pi / 2
            self._avoid_side = 1 if abs(wrap(t1 - th)) < abs(wrap(t2 - th)) \
                else -1
        return math.atan2(ay, ax) + self._avoid_side * math.pi / 2, True

    def _gt_cb(self, msg):
        """gz 真值: 模型名 -> (x, y, yaw)。回调在 gz 线程, 只写不读。"""
        d = {}
        for p in msg.pose:
            q = p.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            d[p.name] = (p.position.x, p.position.y, yaw)
        with self._lock:
            self._gt = d
            self._gt_t = time.monotonic()

    def _pos(self):
        with self._lock:
            return self.x, self.y, self.yaw

    # -------------------------------------------------- P75 录制/回放
    def _load_replay(self, path):
        """读回录制文件, 返回按 tau 升序的 [(tau, v, w), ...]。"""
        seq = []
        hdr = None
        try:
            with open(path, 'r', encoding='utf-8') as f:
                for ln in f:
                    ln = ln.strip()
                    if not ln:
                        continue
                    try:
                        o = json.loads(ln)
                    except Exception:
                        continue
                    if 'hdr' in o:
                        hdr = o
                        continue
                    if 't' in o and 'v' in o and 'w' in o:
                        seq.append((float(o['t']), float(o['v']),
                                    float(o['w'])))
        except IOError as e:
            print('[PERSON] 回放文件读取失败: %s' % e, flush=True)
        seq.sort(key=lambda r: r[0])
        # P75c: 把绝对时间基重基到 0 —— 调用方一律用相对 tau,
        # 否则 seq[0][0] 是绝对时刻(约 9s), 回放要等 9s 才开始。
        if hdr and 't0_abs' in hdr:
            try:
                _b = float(hdr['t0_abs'])
                seq = [(t - _b, v, w) for (t, v, w) in seq]
            except (TypeError, ValueError):
                pass
        return seq, hdr

    def _replay_tick(self):
        """开环回放: 只用 tau 查表, 不看机器人位置/障碍, 不跑状态机。"""
        ta = self._now_abs()
        if ta is None:
            self._publish(0.0, 0.0)
            return
        if self._rp_t0 is None:
            if self._rp_hdr and 't0_abs' in self._rp_hdr:
                self._rp_t0 = float(self._rp_hdr['t0_abs'])
            else:
                self._rp_t0 = ta
            # P76: 手写的场景文件(如 tmp_p76_scn.py)没法预知节点的绝对起始时刻,
            # 会把 t0_abs 写成 0。此时按"节点首拍"锚定, 否则 now=绝对时间(约9s)
            # 一上来就 >= seq[0][0]=0, 整条轨迹会被瞬间跳过。
            if self._rp_t0 <= 0.0:
                self._rp_t0 = ta
            if not self._rp_t0_logged:
                self._rp_t0_logged = True
                _dl = ta - self._rp_t0
                self.get_logger().info(
                    'PERSON 回放锚点 t0_abs=%.3f 本节点首拍=%.3f (差 %+.2fs)'
                    % (self._rp_t0, ta, _dl))
                if _dl > 0.5:
                    self.get_logger().warn(
                        'PERSON 回放起点晚了 %.2fs, 前段轨迹被跳过' % _dl)
                elif _dl < -0.5:
                    self.get_logger().info(
                        'PERSON 回放提前 %.2fs, 静止等待对齐' % (-_dl))
        now = ta - self._rp_t0 + self._rp_off
        seq = self._rp
        if not seq:
            self._publish(0.0, 0.0)
            return
        if now < seq[0][0] or now > seq[-1][0] + 2.0:
            # 录制轴之前/之后: 静止, 保证时间轴严格对齐
            self._publish(0.0, 0.0)
            return
        while self._rp_i + 1 < len(seq) and seq[self._rp_i + 1][0] <= now:
            self._rp_i += 1
        i = self._rp_i
        t0, v0, w0 = seq[i]
        v, w = v0, w0
        if i + 1 < len(seq):
            t1, v1, w1 = seq[i + 1]
            if t1 > t0:
                a = (now - t0) / (t1 - t0)
                a = 0.0 if a < 0.0 else (1.0 if a > 1.0 else a)
                v = v0 + (v1 - v0) * a
                w = w0 + (w1 - w0) * a
        self._publish(v, w)
        if now - self._rp_log > 2.0:
            self._rp_log = now
            x, y, yaw = self._pos()
            self.get_logger().info(
                'PERSON(回放) tau=%.2fs v=%.3f w=%+.3f 位置=(%+.2f, %+.2f)'
                % (now, v, w, x, y))

    def _pick_walk(self):
        # P33: 速度幅值拉开(原 0.20~0.45 变化不明显, 体现不出"时快时慢")
        # P35: 上限 0.60→0.55 —— 机器人 approach_speed=0.52, 人要能被跟上
        # P36: 0.55→0.50, 给速度环留出真正的跟踪余量
        # P39: 0.50→0.40 —— 对正段只有 0.85*0.52=0.44, 人走 0.50 追不上
        self.target_speed = random.uniform(0.12, 0.40)
        self.turn_rate = 0.0
        self.state = 'WALK'
        self.state_until = self._now() + random.uniform(5.0, 9.0)
        self.get_logger().info(f'PERSON 行走中 v={self.target_speed:.2f}m/s')

    def _next_state(self):
        x, y, _ = self._pos()
        # 前方 1.5s 预判有危险时不选择转向/停顿, 继续直线走
        ax = x + math.cos(self.yaw) * 0.15 * 1.5
        ay = y + math.sin(self.yaw) * 0.15 * 1.5
        if in_danger(ax, ay, 0.2):
            self.state_until = self._now() + 1.0
            return
        r = random.random()
        if self.state == 'WALK':
            if r < 0.55:
                rate = random.uniform(0.3, 0.7) * random.choice([-1, 1])
                angle = random.uniform(0.35, 1.6)
                self.state = 'TURN'
                self.turn_rate = rate
                self.target_speed = 0.15
                self.state_until = self._now() + abs(angle / rate)
                self.get_logger().info(f'PERSON 转向 {math.degrees(angle):.0f}°')
            elif r < 0.78:
                self.state = 'SLOW'
                self.turn_rate = 0.0
                self.target_speed = 0.08
                self.state_until = self._now() + random.uniform(2.0, 4.0)
                self.get_logger().info('PERSON 明显减速(慢走)')
            else:
                self.state = 'PAUSE'
                self.turn_rate = 0.0
                self.target_speed = 0.0
                self.state_until = self._now() + random.uniform(1.5, 3.5)
                self.get_logger().info('PERSON 停顿')
        else:
            self._pick_walk()

    def _tick(self):
        if self._rp is not None:
            self._replay_tick()
            return
        now = self._now()
        # P74: dt 由时钟实测, 不再硬编码 —— 定时器抖动会让速度
        # 积分漂移, 同种子两次跑的轨迹因此发散
        dt = 0.05 if self._dt_prev is None else \
            max(0.005, min(0.20, now - self._dt_prev))
        self._dt_prev = now
        x, y, yaw = self._pos()
        with self._lock:
            rx, ry = self.rx, self.ry
        # ---- P95: 随机活动期间自己"想喝水"(常驻跟随状态下的自发行为) ----
        self._maybe_drink(now)
        # ---- P90: 去喝水 / 喝水中, 走专门的控制器 ----
        if self.state == 'GOTO':
            self._goto_tick(now, x, y, yaw, rx, ry)
            return
        if self.state == 'LEAVE':
            self._leave_tick(now, x, y, yaw, rx, ry)
            return
        if self.state == 'DRINK':
            self._publish(0.0, 0.0)
            if now - self._drink_t0 > self.DRINK_TIMEOUT:
                self.get_logger().info('PERSON 喝水超时, 回到随机行走')
                self._pick_walk()
            return
        # P22: 原阈值 1.3m 与跟随目标距离 track_dist_m=1.0 直接冲突 ——
        # 机器人刚走到 1.0m 人就开逃, 机器人永远在追。
        # P33: 0.85 仍然太高 —— 距离环允许下探到 0.9m, 一旦过冲(0.85)
        # 人就主动远离, 等于机器人被"追着跑"的对象反击, 1.0±0.1 永远收敛
        # 不了。改成 0.45m 只在贴身(机器人压上来)时让开, 正常跟随不干预。
        # P38: 0.45 → 0.70 —— 人会主动贴到机器人脸上(实测 d=0.41), 那时
        # 相机已经看不到人体表面, 机器人只能判丢失。合理的人机最小间距
        # 应落在跟随容差带(0.9m)之外。
        robot_close = math.hypot(x - rx, y - ry) < 0.70
        static_danger = in_danger(x, y, 0.1)
        # 卡死检测: 8 秒内位移 < 0.3m 且不在等待/停顿 → 强制逃逸
        if self.state in ('WALK', 'TURN', 'SLOW'):
            moved = math.hypot(x - self._progress_pos[0],
                               y - self._progress_pos[1])
            if now - self._progress_t > 8.0:
                if moved < 0.3:
                    self.state = 'ESCAPE'
                    self._clear_since = None
                    self.get_logger().info('PERSON 疑似卡住, 强制返回场地中心')
                self._progress_pos = (x, y)
                self._progress_t = now
        # 进入危险区/离机器人过近 → ESCAPE
        if self.state not in ('ESCAPE', 'WAIT') and (robot_close or static_danger):
            self.state = 'ESCAPE'
            self._clear_since = None
            self._escape_from_robot = robot_close and not static_danger
            if robot_close and not static_danger:
                self.get_logger().info('PERSON 离机器人过近, 绕开')
            else:
                self.get_logger().info('PERSON 靠近边界/障碍, 返回场地中心')

        if self.state == 'WAIT':
            self._publish(0.0, 0.0)
            if now >= self.state_until:
                self._pick_walk()
            return

        if self.state == 'ESCAPE':
            # 总时长看门狗: 逃逸 12s 仍未清场 → 强制随机行走(防死循环)
            if getattr(self, '_esc_enter', 0.0) == 0.0:
                self._esc_enter = now
            elif now - self._esc_enter > 12.0:
                self.get_logger().info('PERSON 逃逸超时, 强制随机行走')
                self._esc_enter = 0.0
                self._esc_bias = 0.0
                self._esc_t0 = 0.0
                self._clear_since = None
                self._progress_pos = (x, y)
                self._progress_t = now
                self._pick_walk()
                return
            if getattr(self, '_escape_from_robot', False) and not static_danger:
                desired = math.atan2(y - ry, x - rx)  # 远离机器人
            else:
                desired = math.atan2(-y, -x)          # 场地中心
            # 进度看门狗: 逃逸受阻(原地卡住)则随机偏置方向, 防止目标被障碍挡死
            if getattr(self, '_esc_t0', 0.0) == 0.0:
                self._esc_t0 = now
                self._esc_pos = (x, y)
            elif now - self._esc_t0 > 6.0:
                if math.hypot(x - self._esc_pos[0], y - self._esc_pos[1]) < 0.25:
                    self._esc_bias = random.uniform(1.0, 2.2) * random.choice([-1, 1])
                    self.get_logger().info(
                        f'PERSON 逃逸受阻, 方向偏置 {math.degrees(self._esc_bias):.0f}°')
                self._esc_t0 = now
                self._esc_pos = (x, y)
            desired += getattr(self, '_esc_bias', 0.0)
            err = wrap(desired - yaw)
            self.turn_rate = max(-1.2, min(1.2, 2.0 * err))
            # 先原地转对准再走: 避免弧线起步撞上机器人把它推走
            self.target_speed = 0.25 if abs(err) < 0.6 else 0.0
            amax = 0.5 * dt
            ds = self.target_speed - self.speed
            self.speed += max(-amax, min(amax, ds))
            self._publish(self.speed, self.turn_rate)
            clear = (not in_danger(x, y, 0.35)) and \
                    math.hypot(x - rx, y - ry) > 1.4
            if clear:
                if self._clear_since is None:
                    self._clear_since = now
                elif now - self._clear_since > 1.2:
                    self._clear_since = None
                    self._esc_bias = 0.0
                    self._esc_t0 = 0.0
                    self._esc_enter = 0.0
                    self._progress_pos = (x, y)
                    self._progress_t = now
                    self._pick_walk()
            else:
                self._clear_since = None
            if now - self._last_pos_log > 2.0:
                self._last_pos_log = now
                self.get_logger().info(
                    f'PERSON 位置=({x:+.2f}, {y:+.2f}) v={self.speed:.2f} 状态=ESCAPE')
            return

        if now >= self.state_until:
            self._next_state()
        # P33: WALK 段内也做速度抖动, 制造"时快时慢"(原实现每段速度恒定)
        if self.state == 'WALK' and now - self._vmod_t > 2.5:
            self._vmod_t = now
            if random.random() < 0.6:
                self.target_speed = random.uniform(0.12, 0.40)
        amax = 0.35 * dt  # 人的加/减速度, 保证速度变化平滑
        ds = self.target_speed - self.speed
        self.speed += max(-amax, min(amax, ds))
        # 前方 1.5s 预判有危险 → 提前向场地中心转向
        ax = x + math.cos(yaw) * max(self.speed, 0.15) * 1.5
        ay = y + math.sin(yaw) * max(self.speed, 0.15) * 1.5
        w = self.turn_rate
        if in_danger(ax, ay, 0.2) or math.hypot(ax - rx, ay - ry) < 1.3:
            to_center = math.atan2(-y, -x)
            w = max(-1.2, min(1.2, 2.0 * wrap(to_center - yaw)))
            self.speed = min(self.speed, 0.2)
        self._publish(self.speed, w)
        if now - self._last_pos_log > 2.0:
            self._last_pos_log = now
            self.get_logger().info(
                f'PERSON 位置=({x:+.2f}, {y:+.2f}) 朝向={math.degrees(yaw):.0f}° '
                f'v={self.speed:.2f} 状态={self.state}')

    # ---------- P90: "去喝水" 指令链路 ----------
    def _cmd_cb(self, msg):
        try:
            cmd = json.loads(msg.data)
        except Exception:
            self.get_logger().warn('PERSON 无法解析指令: %s' % msg.data)
            return
        act = cmd.get('action')
        if act == 'GO_DRINK':
            # P94: jitter = {"d": 米, "a": 方位角°, "yaw": 朝向偏置°}
            self._start_drink(cmd.get('jitter'), auto=False)
        elif act == 'DRINK_DONE':
            # P93: 机器人把杯子接回去之后就"两清"了 —— 人先走出桌子区域,
            # 再恢复随机活动; 机器人那边还在放回杯子, 放完会撤离桌边并
            # 交权给 follower, 一路跟上来。这俩是并行的, 谁也不等谁。
            if self.state in ('GOTO', 'DRINK'):
                self._leave_target = L.PERSON_LEAVE_SPOT
                self._leave_t0 = self._now()
                self.state = 'LEAVE'
                self.speed = 0.0
                # P95: 下一次"想喝水"从**喝完**起算, 不是从"决定去喝"起算。
                # 差得很关键: 机器人接回杯子之后还要放回 + 撤离 + 交权, 约
                # 60s; 而人走开只要十几秒。若冷却从"决定去喝"起算(间隔
                # 90~180s), 可能出现"人已经站到桌边开口了, 机器人还在撤离"
                # —— 第二条 DRINK 进了 drink_task 的队列, 随后撤离结束发的
                # FOLLOW 会把这条新请求的 cmd_mux 喝水锁一起解掉, 第二轮任务
                # 期间 follower 就来抢底盘。从喝完起算: 机器人 60s 收尾,
                # 人最早 90s 才再开口, 时序上隔开了。
                self._next_drink_t = (self._now()
                                      + random.uniform(*self._drink_gap))
                self.get_logger().info(
                    'PERSON ★ 喝完水, 离开桌边 → (%.2f, %.2f), 下次想喝 '
                    '%.0fs 后'
                    % (self._leave_target[0], self._leave_target[1],
                       self._next_drink_t - self._now()))
        elif act == 'STOP':
            self.state = 'WAIT'
            self.state_until = self._now() + 2.0
            self._publish(0.0, 0.0)
        else:
            self.get_logger().info('PERSON 忽略指令: %s' % msg.data)

    # ---------- P95: 喝水行为(手动指令 / 随机自发走同一条路) ----------
    def _start_drink(self, jit=None, auto=False):
        """走到(带扰动的)接水位 → 站定面向机器人 → 说"我要喝水"。

        P95 之前只有 `_cmd_cb` 一条入口, 喝水永远是"外部喂指令"的结果;
        现在随机自发与手动下发共用这一条路, 行为完全一致, 只是日志里
        写明是谁发起的。
        """
        j = jit or {}
        try:
            _d = float(j.get('d', 0.0))
            _a = math.radians(float(j.get('a', 0.0)))
            yaw_off = math.radians(float(j.get('yaw', 0.0)))
        except (TypeError, ValueError):
            _d, _a, yaw_off = 0.0, 0.0, 0.0
        self._drink_yaw_off = yaw_off
        self._drink_target = (L.PERSON_SPOT[0] + _d * math.cos(_a),
                              L.PERSON_SPOT[1] + _d * math.sin(_a))
        self._drink_said = False
        self._face_yaw = None        # P95: 到位后"面向机器人"的朝向快照
        self._arrived = False        # P95: "已经走到接水位"的锁存(只判一次)
        self._goto_t0 = self._now()
        self.state = 'GOTO'
        self.speed = 0.0
        # 失败保护底: 正常冷却是在"喝完"时(DRINK_DONE)才排下一次的, 但中途
        # GOTO 超时 / 机器人没响应 / 任务失败时收不到那条, 这里先垫一个
        # 下限(45s), 免得退化成"每秒重试一次"。
        self._next_drink_t = self._now() + 45.0
        self.get_logger().info(
            'PERSON ★ %s"去喝水", 目标 = (%.2f, %.2f)%s'
            % ('自己想' if auto else '收到',
               self._drink_target[0], self._drink_target[1],
               '  [站偏 d=%.2fm a=%+.0f° yaw=%+.0f°]'
               % (_d, math.degrees(_a), math.degrees(yaw_off))
               if (_d or yaw_off) else ''))

    def _maybe_drink(self, now):
        """随机活动期间自己"想喝水" → 走到桌边开口, 剩下的交给机器人。

        触发条件: 开了自动喝水 + 冷却过了 + 当前确实在随机活动(不在
        GOTO/DRINK/LEAVE/ESCAPE 里)。ESCAPE 排除在外是刻意的 —— 那是人正
        在脱离障碍/贴身状态, 中途改成"去喝水"会和绕障逻辑互相抢方向盘。
        """
        if not self._auto_drink or now < self._next_drink_t:
            return
        if self._rp is not None:          # 回放模式: 人是时间的纯函数
            return
        if self.state in ('GOTO', 'DRINK', 'LEAVE', 'ESCAPE'):
            return
        jit = None
        if self._jitter_m > 0.0:
            jit = {'d': random.uniform(0.0, self._jitter_m),
                   'a': random.uniform(-180.0, 180.0),
                   'yaw': random.uniform(-self._jitter_deg, self._jitter_deg)}
        self._auto_drink_n += 1
        self._start_drink(jit, auto=True)

    def _truth_pose(self, x, y, yaw, rx, ry):
        """gz 真值优先; 真值缺失/过期(>1s)时才退回里程计。

        P91: 差速里程计走几米就漂 20cm —— GOTO 是按"站到 PERSON_SPOT" 算
        的, 用漂过的坐标会停在别处。机器人位置同理(用来"面向机器人")。
        """
        with self._lock:
            gt = self._gt
            fresh = (time.monotonic() - self._gt_t) < 1.0
        if fresh and 'person' in gt:
            if not self._gt_logged:
                self._gt_logged = True
                self.get_logger().info(
                    'PERSON 真值闭环: 真值(%+.2f,%+.2f) 里程计(%+.2f,%+.2f) '
                    '差 %.0fmm'
                    % (gt['person'][0], gt['person'][1], x, y,
                       math.hypot(gt['person'][0] - x,
                                  gt['person'][1] - y) * 1000.0))
            x, y, yaw = gt['person']
        if fresh and 'robot' in gt:
            rx, ry = gt['robot'][0], gt['robot'][1]
        return x, y, yaw, rx, ry

    def _drive_to_xy(self, now, x, y, yaw, target, tol=0.10, tag='行走'):
        """朝 target 走: 绕障 + 转向阻尼。返回是否已到位。

        tol 说明(GOTO 段): 递杯位是按"人站在 PERSON_SPOT"算出来的(机器人
        前伸 0.62m = 人身前 0.30m 的手)。容差 0.22m 时人实际停在 0.22m 外,
        杯子就递不到手边 —— 实测停在 (2.01,1.28) 而不是 (2.17,1.17); 但
        也不能收到 0.06m: 差速底盘 + 真值闭环的稳态就在 6.4cm 附近(慢速
        0.09m/s 下已无超调, 是转向死区在顶), 永远差半步进不了圈。
        drink_task 递杯时按**实测人手方向**取目标, 0.10m 内都够得着 ——
        P95: 卡在 tol 边界上不前进的病(#51 的变种)已经从两头治好了 ——
        "走到位"改**锁存**(`_arrived`, 不再每拍重判)、"贴脸了就边转边蹭"
        (d≤0.20 时不再 `v=0` 纯转)。所以 tol 保持 0.10m(P91 验证过的值),
        不放宽 —— 人站得越准, 后面递杯要补的位就越少。
        """
        gx, gy = target
        d = math.hypot(gx - x, gy - y)
        if d <= tol:
            return True
        th, avoiding = self._avoid_dir(x, y, math.atan2(gy - y, gx - x),
                                       d, now)
        err = wrap(th - yaw)
        # P91: **v=0 的纯转向是死区**。实测 10s 的 yaw 序列: 指令恒
        # (0, +1.0), 真值 yaw 却在 -1.0 ~ -1.46 之间以 ~3s 周期摆动 ——
        # 纯 P 转向 + 底盘惯性 = 极限环; 而且此时人距目标只剩 0.10m,
        # 走不进圈就永远说不了话。轮子在**滚动**时(走 1.3m 那段, err
        # 稳定在 -0.07)转向阻尼是够的, 所以小误差区间给一点前进当阻尼:
        # |err|≤1.2(cos>0.36) 时 0.09m/s 爬向目标, 只在接近背对时才纯转。
        self.turn_rate = max(-1.0, min(1.0, 1.5 * err))
        if abs(err) > 1.2 and d > 0.20:
            self.speed = 0.0               # 几乎背对且还远: 原地转
        elif d <= 0.20:
            # P95: **贴脸了就别再纯转**。原来 `|err|>1.2 → v=0` 是 #51 的修法
            # (避免"边转边朝目标拱出去"), 但那只在还有一段距离时成立: 实测人
            # 停在离目标 0.120m、要转 83° 朝目标点, err=1.45 → v=0 原地转 → 转向
            # 极限环, 卡在 0.120~0.121 之间再也进不了 tol=0.12 的圈。
            # 0.20m 以内边转边蹭过去: 拱也拱不出多远(0.06m/s × 每拍 0.05s)。
            self.speed = 0.06
        elif avoiding:
            self.speed = 0.22              # 绕行时慢一点, 别蹭上去
        elif d > 0.35:
            self.speed = min(0.42, 0.9 * d)
        else:
            # 最后 0.35m 固定慢速(0.09m/s ≈ 每拍 4.5mm), 比例减速会抖
            self.speed = 0.09
        self._publish(self.speed, self.turn_rate)
        if now - self._last_pos_log > 2.0:
            self._last_pos_log = now
            self.get_logger().info(
                'PERSON %s: 位置=(%+.2f,%+.2f) 距目标 %.3fm yaw=%+.2f '
                'err=%+.2f v=%.2f w=%+.2f%s'
                % (tag, x, y, d, yaw, err, self.speed, self.turn_rate,
                   ' 绕行' if avoiding else ''))
        return False

    def _leave_tick(self, now, x, y, yaw, rx, ry):
        """P93: 喝完水 → 离开桌边 → 到点后恢复随机活动。

        与 GOTO 一样**不看 robot_close** —— 人刚递完杯子时机器人就在
        0.92m 外, 一触发 ESCAPE 就会沿着桌子来回躲, 走不到撤离点。
        """
        if self._leave_target is None:
            self._pick_walk()
            return
        if now - self._leave_t0 > self.LEAVE_TIMEOUT:
            self.get_logger().warn('PERSON 离开超时, 直接恢复随机行走')
            self._pick_walk()
            return
        x, y, yaw, rx, ry = self._truth_pose(x, y, yaw, rx, ry)
        if not self._drive_to_xy(now, x, y, yaw, self._leave_target,
                                 tol=0.25, tag='离开桌边'):
            return
        self._publish(0.0, 0.0)
        self.get_logger().info(
            'PERSON ★ 已离开桌边(%.2f, %.2f), 继续随机活动' % (x, y))
        self._leave_target = None
        self._pick_walk()

    def _goto_tick(self, now, x, y, yaw, rx, ry):
        """走到接水位 → 面向机器人 → 说"我要喝水" → 转 DRINK 静止等待。

        与随机行走的区别: **不看 robot_close**。人本来就要走到机器人跟前
        0.92m 处, 那是 WALK 状态下的"过近"阈值(0.70m)之外, 但路上可能路过,
        一触发 ESCAPE 就永远到不了桌边。
        """
        # P94: 走不到就放弃。原来只有 DRINK 段有超时, GOTO 卡住会永久停在
        # 这里(比如扰动把目标点推进了桌子的避障圈, 绕障一直把人往外推)。
        if now - self._goto_t0 > self.GOTO_TIMEOUT:
            self.get_logger().warn(
                'PERSON 去喝水超时(%.0fs, 目标 (%.2f,%.2f) 未到达), '
                '回到随机行走' % (self.GOTO_TIMEOUT, self._drink_target[0],
                                 self._drink_target[1]))
            self._publish(0.0, 0.0)
            self._pick_walk()
            return
        # P91: 真值优先(见 __init__ 里的说明)。里程计只在真值缺失/过期时用。
        x, y, yaw, rx, ry = self._truth_pose(x, y, yaw, rx, ry)
        # P95: "走到了"要**锁存**。原来每拍都重判 `d <= tol`, 而人停在 tol 边界
        # (0.120 vs 0.12) 上时, 一旦朝向被"面向机器人"带偏, 下一拍 `d` 又 > tol
        # 就被打回走路逻辑、重新去追目标点方向 —— 表现为"站定 → 去喝水 → 站定"
        # 来回横跳, 永远开不了口(实测连着两组各卡满 150s)。走到位一次就认账。
        if not self._arrived:
            if not self._drive_to_xy(now, x, y, yaw, self._drink_target,
                                     tol=0.10, tag='去喝水'):
                return
            self._arrived = True
        # 到位: 站定 + 面向机器人 + 开口
        # P91: 与 GOTO 同一个坑 —— 纯转向(v=0)会摆, 微量前进当阻尼。
        # P94: `_drink_yaw_off` 是验收用的朝向偏置(默认 0 = 正对机器人)。
        #
        # P95: 朝向目标**只取一次快照**, 不能每拍现算。人走向桌边时 follower
        # 一直在后面跟着(这是设计内的: 机器人平时就在跟随), 人到桌边站定后
        # 机器人还在绕着他调整到 1.0m —— 每拍重算朝向的话目标一直在漂, 实测
        # yaw 摆到 +2.87rad、err 恒为 2.0~2.3rad, 人原地转个不停, 150s 都开
        # 不了口(连着两组 side± 都是这么废掉的)。人看一眼机器人在哪、转过去
        # 说话, 之后机器人怎么动都不影响他已经做完的转身。
        if self._face_yaw is None:
            # P95: 朝向目标取**服务位**, 不是"机器人当前位置"。人走到桌边时
            # follower 正带着机器人在他周围绕(跟到 1.0m、还在调整), 朝机器人
            # 现位置转过去, 等机器人回到服务位干活时人的手就偏了 —— 实测偏
            # 32°(base) 到 85°(far), 手到车的距离从 0.66m 变成 1.186m, 递杯
            # 直接差出 122mm。而做喝水任务时机器人**必然**站在服务位
            # (prepare_present 第一件事就是 place_robot), 所以面向服务位才是
            # 对的 —— 这也正是 PERSON_SPOT / CUP_PRESENT_R 那些布局数字的
            # 前提(人站接水位、面向服务位)。
            self._face_yaw = math.atan2(L.SERVE_POSE[1] - y,
                                        L.SERVE_POSE[0] - x) \
                + self._drink_yaw_off
            self.get_logger().info(
                'PERSON 站定(%.2f, %.2f), 转向服务位 %.0f°%s 后开口'
                '(机器人此刻在 (%.2f,%.2f))'
                % (x, y, math.degrees(self._face_yaw),
                   '(含 %+.0f° 偏置)' % math.degrees(self._drink_yaw_off)
                   if self._drink_yaw_off else '', rx, ry))
        err = wrap(self._face_yaw - yaw)
        if abs(err) > 0.15:
            self.turn_rate = max(-1.0, min(1.0, 1.5 * err))
            # err≤1.2 时朝机器人方向 0.02m/s 爬(2s 才 4cm, 不会走出接水位)
            self._publish(0.02 if abs(err) <= 1.2 else 0.0, self.turn_rate)
            return
        self._publish(0.0, 0.0)
        if not self._drink_said:
            self._drink_said = True
            m = String()
            m.data = 'text:我要喝水'
            self.pub_voice.publish(m)
            self.state = 'DRINK'
            self._drink_t0 = self._now()
            self.get_logger().info(
                'PERSON ★ 已到桌边(%.2f, %.2f), 面向机器人, 说: 我要喝水'
                % (x, y))

    def _publish(self, v, w):
        # P75: 录制实际下发量 —— 放在 _publish 里, 保证录到的就是执行量
        if self._rec_fp is not None:
            try:
                ta = self._now_abs()
                if ta is not None:
                    if not self._rec_hdr_done:
                        self._rec_hdr_done = True
                        self._rec_t0 = ta
                        self._rec_fp.write(json.dumps({
                            'hdr': 1,
                            'seed': os.environ.get('PW_SEED', ''),
                            't0_abs': round(ta, 4),
                            'rate_hz': 20.0}) + '\n')
                    self._rec_fp.write(json.dumps({
                        't': round(ta, 4),
                        'v': round(float(v), 4),
                        'w': round(float(w), 4)}) + '\n')
                    self._rec_fp.flush()
            except Exception:
                pass
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)


def main(args=None):
    # P56: 支持固定随机种子, 让 A/B 对比跑在**同一条人行走轨迹**上。
    # 原来 random 未播种, 每次评测人的路径都不一样 —— 实测同一版代码
    # 三次评测 in_band 分别是 75.3% / 61.4% / 15.9%, 单次结果没有可比性。
    # 用法: PW_SEED=7 ros2 launch ...
    import os
    _sd = os.environ.get('PW_SEED', '').strip()
    if _sd:
        _seed = int(_sd)
        random.seed(_seed)
        print('[PERSON] 随机种子 = %d' % _seed, flush=True)
    rclpy.init(args=args)
    node = PersonWalk()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if getattr(node, '_rec_fp', None):
                node._rec_fp.flush()
                node._rec_fp.close()
                print('[PERSON] 录制文件已关闭', flush=True)
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
