#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""follower.py —— 语音指令驱动的跟随控制器 (方案 v3)

控制方案:
1. 位姿: Gazebo 真值(dynamic_pose/info, gz-transport 直连)
2. 速度: 前馈+P  v = 人速前馈 + Kp*(dist - D)
   消除纯 P 的稳态滞后; 人停→机器人停
3. 绕障(真值地图): "机器人→人"连线与障碍物膨胀圆相交时,
   取障碍侧方切点为中间航点, 锁定绕行侧直到连线畅通 ——
   无局部极小值; 近墙时限速并向场内修正
4. 防撞人: 距人<1.2m 后退拉开; 卡死(有指令无位移)摆动脱困
注: GPU 激光在本机渲染栈输出全 0.12(已实测), 仅作可视化,
    不参与控制; 雷达帧计数保留在诊断日志中。
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


def load_track_cfg():
    cfg = {'track_dist_m': 1.0, 'track_dist_tol_m': 0.10,
           'recenter_rot_speed': 1.5, 'approach_speed': 0.2,
           'pid_kp_v': 0.6, 'pid_ki_v': 0.20, 'pid_kd_v': 0.08,
           'pid_kp_w': 2.2, 'pid_ki_w': 0.40, 'pid_kd_w': 0.30}
    try:
        with open(CONFIG_PATH, encoding='utf-8') as f:
            for k in cfg:
                v = json.load(f).get(k)
                if v is not None:
                    cfg[k] = float(v)
    except Exception:
        pass
    return cfg

# ---- 真值地图: 与 world 一致 ----
WALL_SOFT = 5.15        # 接近该值开始向场内修正(提前+加强)
WALL_HARD = 5.75        # 不得越过
OBSTACLES = [           # (x, y, 障碍半径)
    (3.5, 2.0, 0.57),
    (-3.0, 3.2, 0.67),
    (0.5, -3.2, 0.47),
    (-4.2, -2.5, 0.57),
    (4.2, -3.0, 0.37),
    (-1.5, -4.5, 0.62),
    (2.2, 4.6, 0.42),
]
ROBOT_R = 0.18          # 机器人外接半径
INFLATE = ROBOT_R + 0.38  # 绕障膨胀余量


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.x * q.x))


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
        self.target_d = 2.0
        self.rx, self.ry, self.ryaw = 0.0, 0.0, 0.0
        self.px, self.py = 1.0, 0.0
        self._r_prev = None
        self._p_prev = None
        self.robot_speed = 0.0
        self.person_speed = 0.0
        self.scan_count = 0
        self.prox = {'front': -1.0, 'left': -1.0, 'right': -1.0,
                     'back': -1.0, 'age': 99.0}
        self.vis = None
        self._vis_mono = 0.0
        self._vis_prev = None
        self.vis_pspd = 0.0
        self._src_prev = ''
        # 快照式跟踪状态机(P12): TRACK | GOTO | SCAN
        self._track = 'TRACK'
        self._search_target = None
        self._search_t0 = 0.0
        self._scan_until = 0.0
        self._reacq_until = 0.0
        self.hist = []
        # PID 状态(P16)
        self._int_d = 0.0
        self._prev_err_d = 0.0
        self._int_w = 0.0
        self._prev_err_w = 0.0
        self.v = 0.0
        self._turn_dir = 0.0
        self._recenter_dir = 0.0  # 对正方向锁存(±180°奇点防抖)
        self._stuck_t0 = 0.0
        self._stuck_pos = (0.0, 0.0)
        self._stuck_yaw = 0.0
        self._bypass = None       # (障碍索引, 绕行侧)
        self._bypass_t0 = 0.0     # 当前绕行侧开始时间
        self._bypass_arc = 0.0    # 轮廓跟随的弧进度(rad)
        self._bypass_last = None  # (i, side, arc, t) 短暂释放记忆
        self._last_pose_mono = 0.0
        self._last_log = 0.0
        self._last_diag = 0.0
        self.create_timer(0.05, self._tick)  # 20 Hz
        self.get_logger().info(
            'FOLLOWER 跟随控制器v3就绪(前馈+P / 真值地图绕障), 等待语音指令')

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
            # 语音显式给距离则覆盖; 否则用配置默认(P15: track_dist_m)
            self.target_d = float(d) if d is not None \
                else self.cfg['track_dist_m']
            self.state = 'FOLLOW'
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
        with self._lock:
            self.scan_count += 1

    def _prox_cb(self, msg):
        try:
            p = json.loads(msg.data)
        except Exception:
            return
        self.prox = {'front': p.get('front', -1.0),
                     'left': p.get('left', -1.0),
                     'right': p.get('right', -1.0),
                     'back': p.get('back', -1.0),
                     'age': p.get('age', 99.0)}

    def _vis_cb(self, msg):
        """相机人检测: {detected, cam, dist, bearing, u, area}"""
        try:
            v = json.loads(msg.data)
        except Exception:
            return
        self.vis = v
        self._vis_mono = time.monotonic()

    def _hist_cb(self, msg):
        """快照历史: {seq:[{t,cam,bearing,dist,wx,wy}...], lateral:bool}"""
        try:
            h = json.loads(msg.data)
        except Exception:
            return
        self.hist = h.get('seq', [])

    def _infer_target(self):
        """由最近快照序列推测目标去向: 先方位(相机序列)后距离(世界外推)"""
        CAM_CN = {'front': '前', 'left': '左', 'right': '右', 'back': '后'}
        if not self.hist:
            return None, '无快照历史'
        last = self.hist[-1]
        lx, ly = float(last['wx']), float(last['wy'])
        tx, ty = lx, ly
        if len(self.hist) >= 2:
            prev = self.hist[-2]
            dt = max(float(last['t']) - float(prev['t']), 0.4)
            vx = (lx - float(prev['wx'])) / dt
            vy = (ly - float(prev['wy'])) / dt
            if math.hypot(vx, vy) > 0.05:
                ex, ey = vx * 2.5, vy * 2.5   # 外推 2.5s
                e = math.hypot(ex, ey)
                if e > 2.5:
                    ex, ey = ex * 2.5 / e, ey * 2.5 / e
                tx, ty = lx + ex, ly + ey
        tx = max(-5.3, min(5.3, tx))
        ty = max(-5.3, min(5.3, ty))
        seq = [h.get('cam', '?') for h in self.hist]
        seq_cn = ','.join(CAM_CN.get(c, c) for c in seq[-5:])
        lateral = len(set(seq)) > 1
        desc = (f'方位序列[{seq_cn}]'
                f'{"横向移动" if lateral else "同相机"}, '
                f'末位置({lx:+.1f},{ly:+.1f}) → 前往({tx:+.1f},{ty:+.1f})')
        return (tx, ty), desc

    # ---------- 绕障航点 ----------
    def _waypoint(self, rx, ry, px, py):
        """返回 (tx, ty, note, 到航点距离)。
        连线穿障碍膨胀圆时, 取切点为初始航点; 机器人到达切点而连线仍被挡
        时, 航点沿圆弧向绕行侧推进(轮廓跟随), 直到连线畅通;
        短暂释放后重新命中同一障碍时保留侧别与弧进度(防边缘翻转);
        单侧绕行超过 10s 或走满一圈仍被挡 → 换边重来。"""
        best = None
        for i, (ox, oy, r) in enumerate(OBSTACLES):
            R = r + INFLATE
            d = seg_dist(ox, oy, rx, ry, px, py)
            # 迟滞带: 新触发需深入 0.15m; 已在绕行则越出 0.05m 才释放
            # (防止连线掠过圆边缘时绕行状态高频翻转)
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
                # 记忆最近一次绕行(2.5s 内重命中同障碍则续用)
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
            # (速率限制 ≥1.5s: 防止 20Hz 连续推进让航点"逃跑"绕圈)
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
            self.get_logger().warn('FOLLOWER 丢失真值位姿, 已停车')
            return
        # --- 视觉优先 + 快照式跟踪状态机(P12) ---
        vis = self.vis
        vision_ok = (vis is not None and vis.get('detected')
                     and now - self._vis_mono < 0.8
                     and vis.get('dist', -1) > 0)
        if vision_ok:
            if self._track != 'TRACK':
                self.get_logger().info('FOLLOWER ★ 重新捕获目标, 恢复跟踪')
                self._track = 'TRACK'
                self._reacq_until = now + 8.0
            vis_cam = str(vis.get('cam', 'front'))
            bearing = float(vis['bearing'])   # 相对机器人前向
            vdist = float(vis['dist'])
            px = rx + math.cos(ryaw + bearing) * vdist
            py = ry + math.sin(ryaw + bearing) * vdist
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
        else:
            self._vis_prev = None
            self.vis_pspd = 0.0
            src = '真值'
            # 视觉丢失 2s → 按快照序列推测去向并搜索(P12)
            if self._track == 'TRACK' and self._vis_mono > 0 \
                    and now - self._vis_mono > 2.0 and self.hist:
                tgt, desc = self._infer_target()
                if tgt is not None:
                    self._search_target = tgt
                    self._search_t0 = now
                    self._track = 'GOTO'
                    self.get_logger().info(f'FOLLOWER ★ 目标丢失→搜索 {desc}')
            if self._track == 'GOTO':
                if now - self._search_t0 > 30.0:
                    self._track = 'TRACK'
                    self.get_logger().warn('FOLLOWER 搜索超时, 真值兜底恢复跟踪')
                else:
                    gx, gy = self._search_target
                    if math.hypot(gx - rx, gy - ry) < 0.5:
                        self._track = 'SCAN'
                        self._scan_until = now + 10.0
                        self.get_logger().info(
                            f'FOLLOWER 到达最后出现位置({gx:+.1f},{gy:+.1f}),'
                            f' 原地扫描搜索')
                        px, py = rx, ry
                    else:
                        px, py = gx, gy
                    src = '搜索'
            elif self._track == 'SCAN':
                if now > self._scan_until:
                    if now - self._search_t0 < 30.0 and self.hist:
                        tgt, desc = self._infer_target()
                        if tgt is not None:
                            self._search_target = tgt
                            self._track = 'GOTO'
                            self.get_logger().info(
                                f'FOLLOWER 扫描无果, 再次推测 {desc}')
                    else:
                        self._track = 'TRACK'
                        self.get_logger().warn('FOLLOWER 搜索超时, 真值兜底')
                px, py = rx, ry
                src = '扫描'
        if src != self._src_prev and ('真值' in (src, self._src_prev)):
            self.get_logger().info(
                f'FOLLOWER 感知源切换 → {src}'
                + ('' if src == '视觉' else ''))
            self._src_prev = src
        elif src != self._src_prev:
            self._src_prev = src
        dist = math.hypot(px - rx, py - ry)
        err = dist - self.target_d

        # 航点(直连人或绕障切点)
        tx, ty, note, dwp = self._waypoint(rx, ry, px, py)
        err_yaw = wrap(math.atan2(ty - ry, tx - rx) - ryaw)

        # --- 速度 ---
        if note:
            # 绕障期间: 直接驶向切点航点, 不做逼近人的距离控制
            v_t = min(0.6, 1.0 * dwp + 0.15)
            if dwp < 0.30:
                v_t = 0.0  # 已到切点, 等待连线评估
        else:
            # PID 距离环(P16): 带内保持, 带外 PID 平滑接近/跟随;
            # 上限随人速自适应(保底 approach_speed, 人移动时 人速+0.25)
            err_d = dist - self.target_d
            tol = self.cfg['track_dist_tol_m']
            if abs(err_d) <= tol:
                v_t = 0.0
                self._int_d = 0.0
            else:
                dtc = 0.05
                self._int_d = max(-1.5, min(1.5,
                                   self._int_d + self.cfg['pid_ki_v'] * err_d * dtc))
                der_d = (err_d - self._prev_err_d) / dtc
                self._prev_err_d = err_d
                v_pid = self.cfg['pid_kp_v'] * err_d + self._int_d \
                    + self.cfg['pid_kd_v'] * der_d
                v_cap = max(self.cfg['approach_speed'], pspd + 0.25)
                v_t = max(-0.3, min(v_cap, v_pid))
            if v_t > 0 and dist < 1.5:
                v_t = min(v_t, 0.25)  # 靠近人时限速

        # 方向衰减: 目标在侧后方时减速, 正后方(>125°)停车转身
        if abs(err_yaw) > 2.2:
            v_t = 0.0
        elif v_t > 0:
            v_t *= max(0.35, math.cos(err_yaw))
        # 人在非前向相机视野 → 先纯旋转对正(角速度在转向段之后统一覆盖)
        if src.startswith('视觉-'):
            v_t = 0.0
        # 搜索状态速度覆盖(P12): 前往最后出现位置
        if src == '搜索':
            v_t = max(0.05, min(0.7, 0.9 * (dist - 0.3)))

        # --- 转向 ---
        if abs(err_yaw) > 2.2:
            if self._turn_dir == 0.0:
                self._turn_dir = 1.0 if err_yaw > 0 else -1.0
            w_t = self._turn_dir * 2.0
        elif self._turn_dir != 0.0 and abs(err_yaw) >= 1.2:
            w_t = self._turn_dir * 2.0
        else:
            self._turn_dir = 0.0
            # PID 航向环(P16): 平滑转向
            dtc = 0.05
            self._int_w = max(-1.0, min(1.0,
                               self._int_w + self.cfg['pid_ki_w'] * err_yaw * dtc))
            der_w = (err_yaw - self._prev_err_w) / dtc
            self._prev_err_w = err_yaw
            w_t = max(-2.2, min(2.2, self.cfg['pid_kp_w'] * err_yaw
                                + self._int_w + self.cfg['pid_kd_w'] * der_w))

        # --- 固定角速度覆盖(转向段之后, 保证不被覆盖) ---
        # P15: 目标在非前向相机 → 按设定角速度立即旋转, 使目标回到前向视野
        # ±180°附近用方向锁存(>120°锁侧, <60°释放), 防止奇点处转向抖动
        if src.startswith('视觉-') and vision_ok:
            v_t = 0.0
            rot = self.cfg['recenter_rot_speed']
            if abs(bearing) > 2.1:
                if self._recenter_dir == 0.0:
                    self._recenter_dir = 1.0 if bearing >= 0 else -1.0
                w_t = self._recenter_dir * rot * 1.3
            elif self._recenter_dir != 0.0 and abs(bearing) > 1.05:
                w_t = self._recenter_dir * rot * 1.3
            else:
                self._recenter_dir = 0.0
                w_t = rot if bearing >= 0 else -rot
        # P12: 到点原地扫描旋转
        elif src == '扫描':
            v_t = 0.0
            w_t = 0.8

        # --- 避障层1: 深度相机(实时反应层, 帧龄<0.6s 才生效) ---
        prox = self.prox
        if prox['age'] < 0.6:
            f_, l_, r_ = prox['front'], prox['left'], prox['right']
            if f_ > 0:
                if f_ < 1.2 and v_t > 0:
                    v_t = min(v_t, max(0.18, 0.9 * f_ - 0.28))
                    if l_ > 0 and r_ > 0:
                        w_t += 1.0 if l_ > r_ else -1.0  # 向空侧偏转
                if f_ < 0.30:
                    v_t = min(v_t, 0.0)  # 近距硬停
        # --- 避障层2: 真值地图硬保护(兜底, 朝障碍/墙方向禁止前进) ---
        if v_t > 0:
            look = 0.35 + 0.5 * abs(v_t)   # 前瞻随速度缩放
            nx = rx + math.cos(ryaw) * look
            ny = ry + math.sin(ryaw) * look
            if abs(nx) > WALL_HARD or abs(ny) > WALL_HARD:
                v_t = 0.0  # 前方即将越墙界
            else:
                for (ox, oy, r) in OBSTACLES:
                    dx, dy = ox - rx, oy - ry
                    d = math.hypot(dx, dy)
                    if d < r + INFLATE + 0.15 and \
                            (dx * math.cos(ryaw) + dy * math.sin(ryaw)) / max(d, 1e-6) > 0.7:
                        v_t = 0.0  # 障碍在前方±45°内且很近
                        break

        # --- 近墙修正(提前触发, 加强转向) ---
        if abs(rx) > WALL_SOFT or abs(ry) > WALL_SOFT:
            to_c = math.atan2(-ry, -rx)
            w_t += 1.6 * wrap(to_c - ryaw)
            if v_t > 0:
                v_t = min(v_t, 0.35)

        # --- 防撞人(后退时检查身后障碍) ---
        # 阈值随跟踪距离缩放, 必须低于带宽下限(P17): 否则机器人在
        # 正常跟踪位(1.0±0.1m)就持续触发后退 —— 表现为"人动机反向走"
        guard_d = max(0.55,
                      self.target_d - self.cfg['track_dist_tol_m'] - 0.25)
        if dist < guard_d:
            back_clear = True
            for (ox, oy, r) in OBSTACLES:
                dx, dy = ox - rx, oy - ry
                d = math.hypot(dx, dy)
                if d < r + 0.45 and \
                        (dx * math.cos(ryaw) + dy * math.sin(ryaw)) / max(d, 1e-6) < -0.5:
                    back_clear = False  # 障碍在身后
                    break
            if back_clear:
                v_t = max(min(v_t, -(0.2 + 0.7 * (guard_d - dist))), -0.5)
            else:
                v_t = 0.0
                # 确定式侧摆: 向深度更空的一侧转(时间交替式会导致净旋转≈0)
                l_, r_ = self.prox.get('left', -1), self.prox.get('right', -1)
                if l_ > 0 and r_ > 0:
                    w_t = 1.0 if l_ > r_ else -1.0
                else:
                    w_t = 1.0

        # --- 贴墙应急: 先原地转向场心, 再前进离开墙体 ---
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

        # --- 卡死脱困(位置窗口 + 旋转失速双检测) ---
        blocked_lin = abs(self.v) > 0.15
        blocked_rot = abs(w_t) > 0.8
        near_wall = abs(rx) > 5.4 or abs(ry) > 5.4
        if blocked_lin or blocked_rot:
            if self._stuck_t0 == 0.0:
                self._stuck_t0 = now
                self._stuck_pos = (rx, ry)
                self._stuck_yaw = ryaw
            elif now - self._stuck_t0 > 2.5:
                moved = math.hypot(rx - self._stuck_pos[0],
                                   ry - self._stuck_pos[1])
                dyaw = abs(wrap(ryaw - self._stuck_yaw))
                if (blocked_lin and moved < 0.15) or \
                        (blocked_rot and dyaw < 0.26):
                    to_c = math.atan2(-ry, -rx)
                    w_esc = 1.2 if wrap(to_c - ryaw) >= 0 else -1.2
                    if near_wall:
                        # 贴墙卡死: 前后交替抖动; 前进相位必须过墙体前瞻,
                        # 否则只退不进(防止抖动把机器人怼进墙里)
                        fw_look = 0.30
                        fx = rx + math.cos(ryaw) * fw_look
                        fy = ry + math.sin(ryaw) * fw_look
                        fw_ok = abs(fx) < WALL_HARD and abs(fy) < WALL_HARD
                        if fw_ok and int(now * 1.25) % 2 == 0:
                            v_t = 0.45
                        else:
                            v_t = -0.45
                    else:
                        v_t = -0.55
                    w_t = w_esc
                    self.get_logger().warn(
                        f'FOLLOWER 卡死脱困(平移{moved:.2f}m/旋转'
                        f'{math.degrees(dyaw):.0f}°{"贴墙抖动" if near_wall else "倒车"})')
                self._stuck_t0 = now
                self._stuck_pos = (rx, ry)
                self._stuck_yaw = ryaw
        else:
            self._stuck_t0 = 0.0

        # --- 加速度限制 + 角速度最终总钳制(多层叠加后限幅) ---
        amax = 0.9 * 0.05
        dv = v_t - self.v
        self.v += max(-amax, min(amax, dv))
        w_t = max(-2.2, min(2.2, w_t))
        self._publish(self.v, w_t)

        # --- 日志 ---
        if now - self._last_log > 0.5:
            self._last_log = now
            self.get_logger().info(
                f'FOLLOWER 跟随中[{src}] 距离={dist:.2f}m(目标{self.target_d:.2f}m) '
                f'人速={pspd:.2f} 指令v={self.v:.2f} 实际v={rspd:.2f} '
                f'w={w_t:+.1f} 偏航={math.degrees(err_yaw):+.0f}° {note}')
        if now - self._last_diag > 4.0:
            self._last_diag = now
            self.get_logger().info(
                f'FOLLOWER 诊断 雷达帧={nscan} 机器人=({rx:+.2f},{ry:+.2f}) '
                f'人=({px:+.2f},{py:+.2f}) {note or "直连"}')

    def _publish(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)


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
