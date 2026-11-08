#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""drink_task.py —— "倒水递水"任务主节点 (P85 骨架)

职责(按里程碑逐层加):
  P85 ①39 路 `/drink/cmd/*` 的**唯一发布者**, 稳定 20Hz
      ②上电回 home(Q_REST + 十指张开 + 水流/水位/人的肩 归零)
      ③`/drink/ck` 自检: 手掌位置与"指令 / 运动学 FK / gz 真值"三方对齐
  P86 抓水壶   P87 抓杯+倒水   P88 放壶+双手捧杯   P89 递/接/放回

为什么强调"唯一发布者"
=====================
gz 侧 `gz-sim-joint-position-controller-system` 在
`<use_velocity_commands>true</use_velocity_commands>` 下把 JointVelocityCmd
当**硬约束**直接交给求解器。若两个节点同时往同一路 `/drink/cmd/*` 写, 谁后
到谁生效 —— 关节会在两个目标之间来回跳(实测现象是"到位了但一直微抖, 采样
误差 ±3°")。所以 39 路指令只留一个节拍源; 其他模块(arm_kin/joint_servo)只
往总线里写目标值, 自己不发。

为什么 20Hz 而不是"发一次就不管"
==============================
插件确实会保持最后一条指令。但持续下发换来两件事:
  ①上游断流能立刻看见(`ros2 topic hz` 掉到 0), 而不是留下"关节僵在最后一
    个位置"这种极难定位的现象;
  ②P86 之后的连续轨迹本来就是流式插值, 节拍定下来后协议不用改。

自检为什么敢直接对 gz 真值比
==========================
P84 曾记录"dynamic_pose/info 里 link 位姿名字错位一位(R_L1 的 pose 其实是
肩部偏置)"。但那是**19 个 gz 世界叠加**造成的假象(见 TaskPlan §6.0)。
清洁启动后 `tests/diag/probe_palm_gt.py` 实测 FK 与真值 **0.0mm / 0.00°**。
所以 `/drink/ck` 一次给三列 —— 指令 / FK / 真值, 任一处偏了都能立刻分清是
"通道没送到"还是"运动学参数错"还是"物理被撞了"。

用法:
    ros2 run vla_follow drink_task            # 正常起, 上电回 home 后保持 20Hz
    ros2 run vla_follow drink_task --once     # 回 home → 自检 → 退出(测试用)
    ros2 service call /drink/ck std_srvs/srv/Trigger
    ros2 service call /drink/home std_srvs/srv/Trigger
"""
import json
import math
import sys
import time
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from vla_follow import arm_kin as K
from vla_follow import scene_layout as L
from vla_follow.carry import CarryBus
from vla_follow.joint_servo import JointServo
from vla_follow import liquid as _liquid_mod
from vla_follow.liquid import LiquidCtl

# gz 真值(手掌位置): 用来做第三方比对。缺 gz 绑定时降级为"只比 FK"。
try:
    import gz.transport13 as gz_transport
    from gz.msgs10.pose_v_pb2 import Pose_V as GzPose_V
    HAVE_GZ = True
except Exception:                                     # pragma: no cover
    HAVE_GZ = False

PUB_HZ = 20.0
CARRY_HZ = 50.0            # 抓握附着目标位姿的下发频率
ARM_TOL_DEG = 1.5          # 手臂到位判据
HAND_TOL_DEG = 4.0         # 手指到位判据(指骨短、限位软, 放宽)
CK_POS_TOL = 0.010         # 自检: FK 与真值位置容差 10mm
CK_ROT_TOL = math.radians(3.0)


# ---------------------------------------------------------------- 通道清单
def all_cmd_names():
    """39 路 `/drink/cmd/*` 的话题后缀, 顺序固定(便于逐列打印)。"""
    out = []
    for s in ('R', 'L'):
        out += ['%s_%s_joint' % (s, j[0]) for j in K.JOINTS]
        out += ['%s_joint' % n for n in K.hand_joints(s)]
    # 水壶水流(jet, prismatic) / 杯内水位(water, prismatic) / 人的肩(person)
    out += ['jet_joint', 'water_joint', 'person_arm_joint']
    # P96: 人的两条手臂各自的外展(左 +v / 右 −v, 见 gen_person_ext)
    out += ['person_arm_l_joint', 'person_arm_r_joint']
    return out


CMD_NAMES = all_cmd_names()
assert len(CMD_NAMES) == 39, len(CMD_NAMES)


def default_cmd():
    """39 路指令的**安全默认值** = home 位形。

    为什么不能默认 0
    ---------------
    q=0 对 7 轴臂意味着"大臂小臂笔直沿 -z 垂下", 与 Q_REST 差 30~50°; 手指
    q=0 则是半握。若总线初值是 0, 节点刚起来的头几百毫秒就到处的定时器把
    0 灌给关节, 机械臂会先被拽到笔直位形; 紧接着 home() 再把它拉回 Q_REST。
    实测代价: R 侧 home 从 <1s 变成 12s 并触发超时(残差 7.9°), 看上去像
    "关节跟不上", 实际是自己先发了一条错误指令。
    所以默认值直接取 home —— 上电到第一帧之间, 关节收到的就是它该在的位置。
    """
    v = {}
    for s in ('R', 'L'):
        for j, q in zip(K.JOINTS, K.Q_REST):
            v['%s_%s_joint' % (s, j[0])] = float(q)
        for n, g in zip(K.hand_joints(s), K.grip_values(s, K.GRIP_OPEN)):
            v['%s_joint' % n] = float(g)
    # 水流(jet)/水位(water)/人的肩 都归零
    v['jet_joint'] = 0.0
    v['water_joint'] = 0.0
    v['person_arm_joint'] = 0.0
    # P96: 人的两只手 —— 默认值取**合拢**(手心间距 184mm, 正好捧住杯壁)。
    # 不能默认 0: 0 对应的手心间距只有 170mm, 手掌会陷进杯壁 6mm; 更不能
    # 默认张开 —— 节点刚起来那几百毫秒人在"张着手走路", 而杯还没递过来。
    # 右侧反号(轴同为 (0,0,1), 见 gen_person_ext)。
    v['person_arm_l_joint'] = L.PERSON_HAND_CLOSE
    v['person_arm_r_joint'] = -L.PERSON_HAND_CLOSE
    return v


class CmdBus(object):
    """39 路 Float64 指令的唯一发布者。

    所有模块只通过 set() 写目标; 真正 `publish()` 只发生在 20Hz 定时器里。
    """

    def __init__(self, node, names=None, init=None):
        self.node = node
        self.names = list(names or CMD_NAMES)
        self.pub = {n: node.create_publisher(Float64, '/drink/cmd/%s' % n, 10)
                    for n in self.names}
        base = default_cmd() if init is None else init
        self.val = {n: float(base.get(n, 0.0)) for n in self.names}
        self.count = 0            # 累计发布帧数(测速用)
        self.t0 = time.time()
        # The main task thread writes targets while an executor timer may
        # publish them. Keep multi-joint updates and reads coherent.
        self._lock = threading.RLock()

    def set(self, names, values):
        with self._lock:
            for n, v in zip(names, values):
                if n in self.val:
                    self.val[n] = float(v)

    def get(self, name):
        with self._lock:
            return self.val.get(name, 0.0)

    def get_many(self, names):
        with self._lock:
            return np.array([self.val.get(n, 0.0) for n in names], float)

    def publish_all(self):
        # Never hold the state lock while calling into ROS middleware.
        with self._lock:
            snapshot = dict(self.val)
            self.count += 1
        for n, p in self.pub.items():
            m = Float64()
            m.data = snapshot[n]
            p.publish(m)

    def hz(self):
        with self._lock:
            count = self.count
        dt = time.time() - self.t0
        return count / dt if dt > 0 else 0.0


# ---------------------------------------------------------------- 位姿小工具
def gz_pose_to_T(p):
    q = p.orientation
    return K.frame(K.quat_to_mat(q.x, q.y, q.z, q.w),
                   (p.position.x, p.position.y, p.position.z))


def yaw_of(quat):
    return math.atan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                      1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z))


def _drift(Ta, Tb):
    """两次真值之间杯子在**水平面**上滑了多远(mm); 缺一帧就给 None。

    用平面距离而不是 3D: 杯是被钉在桌面上的自由体, 我们关心的是"有没有被
    蹭着推走", 竖直方向那点沉降不算。
    """
    if Ta is None or Tb is None:
        return None
    return round(math.hypot(float(Tb[0, 3]) - float(Ta[0, 3]),
                            float(Tb[1, 3]) - float(Ta[1, 3])) * 1e3, 1)


class DrinkTask(Node):
    """喝水任务主节点。P85 只实现"通道 + home + 自检", 动作序列在 P86 之后加。"""

    def __init__(self, observer_only=False):
        super().__init__('drink_task' if not observer_only else 'drink_observer')

        # ---- 输入: 关节实测 / 底盘位姿 / gz 真值
        self.js = {}
        self.links = {}
        self.base = (0.0, 0.0, 0.0)
        self._have_odom = False
        self._truth_base = False
        self.create_subscription(JointState, '/drink/joint_state',
                                 self._js_cb, 20)
        self.create_subscription(Odometry, '/robot/odom', self._odom_cb, 20)
        if HAVE_GZ:
            self._gz = gz_transport.Node()
            self._gz.subscribe(GzPose_V,
                               '/world/follow_world/dynamic_pose/info',
                               self._gt_cb)

        # ---- 输出: 39 路指令总线(唯一发布者)
        self.bus = CmdBus(self)
        # observer_only: 端到端验收脚本只**看**不动手。此时不挂 20Hz/50Hz
        # 定时器, 于是 39 路指令和抓取位姿一帧都不会发出去 —— 否则会和
        # 常驻的 drink_task 抢同一个发布者, 杯子被两个目标拽来拽去。
        self.carry = CarryBus(self)
        # P98: 运动学液面(体积守恒/水平液面/连续水柱), update 挂 50Hz carry tick
        self.liquid = LiquidCtl(self)
        if not observer_only:
            self.create_timer(1.0 / PUB_HZ, self._tick)
            self.create_timer(1.0 / CARRY_HZ, self._carry_tick)

        # ---- 状态发布(给上层/测试用)
        self.pub_state = self.create_publisher(String, '/drink/state', 10)
        self.state = 'BOOT'
        # ---- P90: 指令入口 ----
        # 语音(/voice/request) → voice_node → /robot/command {"action":"DRINK"}
        # → 本节点排队执行整套任务。回调里只置标志: `run_drink` 内部要
        # `spin_for`(即再进 spin_once), 在回调里重入 executor 会报错。
        self._want_drink = False
        self._want_home = False
        self._want_hang = False
        self._busy = False
        self.pub_person = self.create_publisher(String, '/person/command', 10)
        # P93: 任务收尾后要把底盘**交还**给 follower(它收到 DRINK 时让过权,
        # cmd_mux 也上了 300s 锁)。这两个都是收到 /robot/command 才动作。
        self.pub_cmd = self.create_publisher(String, '/robot/command', 10)
        self.create_subscription(String, '/robot/command', self._cmd_cb, 10)
        # P104: 订阅 follower 状态，在交权后确认 FOLLOW 指令确已被接收。
        self._follower_status = {}
        self._follower_status_t = 0.0
        self.create_subscription(String, '/follower/status',
                                 self._follower_status_cb, 10)
        # ---- 底盘速度(原地转身/驶入/撤离)。P102: 走**独立话题** /drink/cmd_vel,
        # 由 cmd_mux 在喝水锁内放行 —— 原来和 cmd_mux 一起抢 /robot/cmd_vel,
        # 两边都 20Hz, 谁后到谁生效, 撤离能慢 5 倍甚至原地不动。
        self.pub_vel = self.create_publisher(Twist, '/drink/cmd_vel', 10)
        # 兜底直发(没有 cmd_mux 的单点脚本用; 见 drive())
        self.pub_vel_raw = (None if observer_only
                            else self.create_publisher(Twist,
                                                       '/robot/cmd_vel', 10))

        # ---- 服务
        self.create_service(Trigger, '/drink/ck', self._srv_ck)
        self.create_service(Trigger, '/drink/home', self._srv_home)
        self.create_service(Trigger, '/drink/hang', self._srv_hang)

        # ---- 伺服封装: 全部走共享总线
        self.arm = {s: JointServo(self, s, tol_deg=ARM_TOL_DEG, bus=self.bus)
                    for s in ('R', 'L')}
        self.hand = {s: JointServo(self, s,
                                   joint_names=['%s_joint' % n
                                                for n in K.hand_joints(s)],
                                   tol_deg=HAND_TOL_DEG, bus=self.bus)
                     for s in ('R', 'L')}

        self._last_hb = 0.0
        self.get_logger().info(
            'drink_task 已起: 39 路指令 @%.0fHz, 人体手高 %.2fm, 服务位 %s'
            % (PUB_HZ, L.PERSON_HAND_Z, str(L.SERVE_POSE)))

    # ------------------------------------------------------------ 回调
    def _follower_status_cb(self, msg):
        try:
            self._follower_status = json.loads(msg.data)
            self._follower_status_t = time.monotonic()
        except Exception:
            pass

    def _js_cb(self, msg):
        self.js.update(zip(msg.name, msg.position))

    def _odom_cb(self, msg):
        # 踩坑记录(P86): gz 的 diff-drive 里程计是**轮子积分**, 底盘被
        # `place`/`set_pose` 直接摆到别处时它不会跟着变 —— 实测机器人已在
        # (1.75, 0.35) 而 odom 仍报 (0, 0)。一旦让 odom 覆盖 base, 机械臂的
        # 正解/逆解就整体错位 2.3m: 表现为 "IK 残差 1577mm / 限位余量 7°"
        # 这种既解不动又不像够不着的怪数值, 而且 [4] 通过 [2] 失败, 因为
        # 中间只要有一拍 odom 消息就把 `sync_base_from_truth()` 抹掉了。
        # 结论: base 以 gz 真值为准(机械臂本来就挂在模型上, 模型世界位姿
        # 才是唯一自洽的底座), odom 只在真值话题缺席时兜底。
        if self._truth_base:
            return
        p = msg.pose.pose.position
        self.base = (p.x, p.y, yaw_of(msg.pose.pose.orientation))
        self._have_odom = True

    def _gt_cb(self, msg):
        for p in msg.pose:
            self.links[p.name] = p
        # P98 排查: 道具键会间歇性从 links 里"消失"(抓取时在、place_held
        # 时变 None → FAIL_PREPARE)。节流打印真值流的关键键存在性, 定位
        # 是"流里没有"还是"字典丢了"。
        self._gt_n = getattr(self, '_gt_n', 0) + 1
        if self._gt_n % 150 == 0:      # ~5s @30Hz
            self.get_logger().info(
                'GT links_n=%d kettle=%s cup=%s disc=%s' % (
                    len(self.links), 'kettle' in self.links,
                    'cup' in self.links,
                    'cup_water_disc' in self.links))

    def spin_for(self, s):
        """阻塞推进 ROS 时间: 定时器、订阅都会照常跑。"""
        t0 = time.time()
        while time.time() - t0 < s:
            rclpy.spin_once(self, timeout_sec=0.005)

    def spin_sim(self, s, cap=6.0):
        """**按仿真时间**阻塞 s 秒(墙上时间放宽 cap 倍兜底)。

        "等 0.3s 让手停稳再量"这种话, 说的是机械臂在仿真里还要滑 0.3s。RTF
        只有 0.32 时, 墙上 0.3s 只等于 0.09s 仿真时间, 量到的还是滑行中的
        位姿 —— 于是 place_held 以为没到位, 再补一步, 越补越飘(实测放回残差
        52.3mm 直接判失败)。凡是"量之前先停稳"的地方都改走这里。
        """
        try:
            sim = bool(self.get_parameter('use_sim_time').value)
        except Exception:
            sim = False
        if not sim:
            self.spin_for(s)
            return
        t0 = time.time()
        s0 = self.get_clock().now().nanoseconds * 1e-9
        while True:
            rclpy.spin_once(self, timeout_sec=0.005)
            el = time.time() - t0
            if self.get_clock().now().nanoseconds * 1e-9 - s0 >= s \
                    or el > s * cap:
                return

    def drive(self, vx=0.0, wz=0.0):
        """发一帧底盘速度。gz 的 DiffDrive 会锁存最后一条, 但仍按 20Hz 补发。

        发两份: `/drink/cmd_vel` 给 cmd_mux(喝水锁内由它放行, 这是"正式通道"),
        `/robot/cmd_vel` 给"没有 cmd_mux 的场景"(单点脚本 test_p87/p88/p89 直接
        调 prepare_present, 那时没人中转)。两个值永远相同, 同时在场也不会打架。
        """
        m = Twist()
        m.linear.x = float(vx)
        m.angular.z = float(wz)
        self.pub_vel.publish(m)
        if self.pub_vel_raw is not None:
            self.pub_vel_raw.publish(m)

    # ------------------------------------------------------------ P90 喝水任务
    def _cmd_cb(self, msg):
        try:
            cmd = json.loads(msg.data)
        except Exception:
            return
        if cmd.get('action') == 'DRINK':
            if self._busy:
                self.get_logger().warn('DRINK 正在执行, 忽略重复的喝水请求')
                return
            self._want_drink = True
            self.get_logger().info(
                'DRINK ★ 收到喝水请求(来源=%s, 内容="%s"), 排队执行'
                % (cmd.get('source', '?'), cmd.get('text', '')))

    def _set_state(self, s, extra=None):
        """切阶段 + **立即广播**一次。

        20Hz 的 `_tick` 里只有 5s 一次的心跳, 而端到端脚本要靠 state 跟阶段
        (哪个时刻该截图)。所以状态一变就发一条, 别等心跳。

        `extra` 是附加到同一条消息里的过程数据(P94: 递杯补位结果), 免得
        只读的验收脚本为了拿一个数字再去翻日志。
        """
        if s == self.state and extra is None:
            return
        self.state = s
        payload = {'state': s}
        if extra:
            payload.update(extra)
        self.pub_state.publish(String(data=json.dumps(payload)))

    def _tell_person(self, action):
        self.pub_person.publish(String(data=json.dumps({'action': action})))

    def _tell_robot(self, action):
        """给机器人自己下指令(/robot/command)。

        P93: 放回杯子之后发 `FOLLOW` —— follower 收到就恢复跟随、cmd_mux
        收到就解 300s 的喝水锁。一条消息两处生效, 顺序无关。

        连发 3 帧: 单向话题在发现( discovery )完成前会静默丢消息(§6.4
        #24), 而这条是整个"交权"的唯一触发, 丢了机器人就永远停在桌边。
        """
        m = String(data=json.dumps({'action': action}))
        for _ in range(3):
            self.pub_cmd.publish(m)
            self.spin_for(0.2)

    def _arm_state(self, tag, sides=('L', 'R')):
        """记下这一刻两侧手臂的姿态与人体规则体检(P103 验收用)。

        取数口径与运行时日志同源(`drink_motion.arm_state` → 判据
        `arm_kin.violation_list`), 所以"日志里没星号"与"验收通过"是一回事。
        """
        from vla_follow import drink_motion as M
        if not hasattr(self, 'arm_audit'):
            self.arm_audit = {}
        out = {}
        for _s in sides:
            st = M.arm_state(self, _s)
            if st is not None:
                out[_s] = st
        self.arm_audit[tag] = out
        bad = sorted({b for st in out.values() for b in st['bad']})
        self.get_logger().info(
            'ARMCHK[%s] %s'
            % (tag, ('★ ' + ' '.join(bad)) if bad else 'OK(无违反人体规则)'))
        return out

    def _elbow_report(self, tag):
        """量两只肘此刻朝哪(P101 验收用), 返回 {side: 度量}。

        口径见 `arm_kin.elbow_metrics`: `out_line` 是"肘往体侧架出去多少"
        (与手臂伸出多远无关), `out_base` 是相对车身的横向外扩, `droop` 是
        肘低于肩-腕轴线的量。三个都播出去, 判据在验收脚本里定。
        """
        from vla_follow import drink_motion as M
        out = {}
        for _s in ('R', 'L'):
            _q = M.q_meas(self, _s)
            if _q is None:
                continue
            out[_s] = K.elbow_metrics(_s, _q)
        if out:
            def _f(s, k):
                return (out.get(s) or {}).get(k, 0.0) or 0.0
            self.get_logger().info(
                'DRINK 肘部[%s]: L 外扩线 %+.0fmm/基础 %+.0fmm 垂 %+.0fmm '
                'up_roll %+.2f | R 外扩线 %+.0fmm/基础 %+.0fmm 垂 %+.0fmm '
                'up_roll %+.2f  伸出 %.3f/%.3f m'
                % (tag, _f('L', 'out_line_mm'), _f('L', 'out_base_mm'),
                   _f('L', 'droop_mm'), _f('L', 'up_roll'),
                   _f('R', 'out_line_mm'), _f('R', 'out_base_mm'),
                   _f('R', 'droop_mm'), _f('R', 'up_roll'),
                   _f('L', 'reach_m'), _f('R', 'reach_m')))
        return out

    def _present_target(self):
        """递杯目标。实现搬到 `M.present_target_at_hand()` 去了 ——
        P96 的验收脚本也要用同一个目标, 两边各写一份就会一边量到 34.6mm、
        另一边量到 7mm(实测踩到过: 脚本用的是名义目标)。
        """
        from vla_follow import drink_motion as M
        return M.present_target_at_hand(self)

    def _approach_if_far(self):
        """P94: 臂伸满还够不着人手时, 底盘朝人挪一段再递。

        递杯距离是"车 → 人手", 人往外站 0.15m 就变成 0.77m, 而臂的安全前伸
        只有 0.70m —— 剩下 70mm 靠底盘补。不补的话 `hand_to` 交接时杯子会
        从"离手 70mm 处"瞬移过去(钉到人手上)，画面上是杯子跳一下。

        只前进、不重新转身: 进 PRESENT 前机器人已经在 `prepare_present` 里
        转身面向人了(HANDOVER_YAW), 此时"车→人"就是车头方向, `drive_to`
        走的是直线。挪动上限 0.12m(服务位到桌心 0.85m, 挪完还有 0.79m,
        大于桌面外接圆 0.574 + 底盘半径 0.17 = 0.744)。
        """
        from vla_follow import drink_motion as M
        self.sync_base_from_truth(quiet=True)
        bx, by, _byaw = self.base
        Tp = M.object_pose(self, 'person')
        Th = M.person_hand_world(self, Tp)
        if Th is None:
            return {'ok': False, 'moved': False, 'why': '拿不到人手真值'}
        dx, dy = Th[0, 3] - bx, Th[1, 3] - by
        d_hb = math.hypot(dx, dy)
        # P94: 三个点都打出来。只看"车→人手"这一个数没法区分"人站远了"和
        # "手点算错了" —— 后者表现为手到人中心的距离明显 ≠ PERSON_HAND_FWD
        # (0.30m), 那通常是 person_arm_joint 没归零或 person_arms link 的
        # 相对位姿取错。
        if Tp is not None:
            self.get_logger().info(
                'DRINK 递杯几何: 车(%.2f,%.2f) 人(%.2f,%.2f) 手(%.2f,%.2f) '
                '手到人 %.3fm(名义 %.2f) 车→手 %.3fm'
                % (bx, by, Tp[0, 3], Tp[1, 3], Th[0, 3], Th[1, 3],
                   math.hypot(Th[0, 3] - Tp[0, 3], Th[1, 3] - Tp[1, 3]),
                   L.PERSON_HAND_FWD, d_hb))
        gap = d_hb - L.PRESENT_APPROACH_CLEAR
        if gap <= L.PRESENT_APPROACH_GAP:
            self.get_logger().info(
                'DRINK 递杯间距: 车→人手 %.3fm, 不用补位%s'
                % (d_hb, ' —— 超出臂上限 %.2f 共 %.0fmm, 由 place_held 按'
                   '实测人手方向送到最远处'
                   % (L.CUP_PRESENT_R_MAX,
                      (d_hb - L.CUP_PRESENT_R_MAX) * 1000.0)
                   if d_hb > L.CUP_PRESENT_R_MAX else ' (在臂可及范围内)'))
            return {'ok': True, 'moved': False, 'd_hb': round(d_hb, 3)}
        ux, uy = dx / d_hb, dy / d_hb
        # 能挪多少由**桌子**说了算: 沿"车→人"方向步进试探, 贴到桌面
        # PRESENT_TABLE_PAD 之内就不再往前。别用桌子的外接圆(TABLE_R=0.62)
        # 估 —— 那个圆把桌角外侧一大片都划成禁区, 实测只允许挪 0.147m,
        # 而人往外站 0.15m 时需要挪 0.194m, 差出来的 47mm 正好是判据边缘
        # 上那点余量。矩形判据下同一条射线可以挪 0.37m(见 §6.4 #56)。
        step = 0.0
        while step + 0.02 <= L.PRESENT_APPROACH_MAX:
            if L.table_clearance(bx + (step + 0.02) * ux,
                                 by + (step + 0.02) * uy,
                                 pad=L.PRESENT_TABLE_PAD) < 0.0:
                break
            step += 0.02
        step = min(gap, step)
        if step < 0.02:            # 桌子和人夹住了, 一步也挪不动
            self.get_logger().warn(
                'DRINK 递杯补位: 车→人手 %.3fm, 但朝人方向已贴到桌子, '
                '挪不动(杯子会停在离手 %.0fmm)' % (d_hb,
                                                   (d_hb - 0.70) * 1000.0))
            return {'ok': False, 'moved': False, 'why': '贴桌, 挪不动',
                    'd_hb': round(d_hb, 3)}
        # P96: `drive_to` 的前进速度有个 0.06m/s 的下限(防 P 控制在目标前
        # 停死), 而停稳还要等 0.5s —— 换算下来就是 ~30mm 的固有残差。
        # yaw+ 这组要补 38mm, 残差 31mm, 一轮下来基本白挪(实测车→人手
        # 0.728 → 0.721m)。所以: v_min 降到 0.02、容差收到 12mm, 并且
        # **按实测重算方向再补一轮**, 把残差真正吃掉。
        d2 = d_hb
        total = 0.0
        drive_ok_all = True
        drive_residuals = []
        for i in range(3):
            self.sync_base_from_truth(quiet=True)
            bxn, byn, _ = self.base
            Thn = M.person_hand_world(self, M.object_pose(self, 'person'))
            if Thn is None:
                break
            dxn, dyn = Thn[0, 3] - bxn, Thn[1, 3] - byn
            dn = math.hypot(dxn, dyn)
            if dn <= L.PRESENT_APPROACH_CLEAR + L.PRESENT_APPROACH_GAP:
                d2 = dn
                break
            # 每轮最多挪"还差多少", 但别超过桌子允许的范围
            gap_n = dn - L.PRESENT_APPROACH_CLEAR
            uxn, uyn = dxn / dn, dyn / dn
            sn = 0.0
            while sn + 0.02 <= L.PRESENT_APPROACH_MAX:
                if L.table_clearance(bxn + (sn + 0.02) * uxn,
                                     byn + (sn + 0.02) * uyn,
                                     pad=L.PRESENT_TABLE_PAD) < 0.0:
                    break
                sn += 0.02
            sn = min(gap_n, sn)
            if sn < 0.02:
                break
            ok, res_d = M.drive_to(self, (bxn + sn * uxn, byn + sn * uyn),
                                   v_max=0.12, tol_mm=12.0, timeout=25.0,
                                   k=0.9, v_min=0.02, tag='递杯补位%d' % (i + 1))
            total += sn
            drive_residuals.append(round(float(res_d), 1))
            if not ok:
                drive_ok_all = False
                self.get_logger().error(
                    'DRINK 递杯补位%d 底盘行驶失败(残差 %.1fmm)，不再继续伸手递杯'
                    % (i + 1, res_d))
                break
            self.sync_base_from_truth(quiet=True)
            bxn, byn, _ = self.base
            Thn = M.person_hand_world(self, M.object_pose(self, 'person'))
            d2 = (math.hypot(Thn[0, 3] - bxn, Thn[1, 3] - byn)
                  if Thn is not None else -1.0)
            self.get_logger().info(
                'DRINK 递杯补位%d: 车→人手 → %.3fm (行驶残差 %.0fmm, '
                '累计挪 %.0fmm)' % (i + 1, d2, res_d, total * 1000.0))
        self.get_logger().info(
            'DRINK 递杯补位: 车→人手 %.3f → %.3fm (目标 ≤%.2f)'
            % (d_hb, d2, L.PRESENT_APPROACH_CLEAR))
        within_safe_reach = d2 > 0.0 and (
            d2 <= L.PRESENT_APPROACH_CLEAR + L.PRESENT_APPROACH_GAP)
        approach_ok = bool(drive_ok_all and within_safe_reach)
        return {'ok': approach_ok, 'moved': bool(total > 0.0),
                'd_hb': round(d_hb, 3), 'd_after': round(d2, 3),
                'step_m': round(total, 3), 'drive_ok': bool(drive_ok_all),
                'drive_residuals_mm': drive_residuals,
                'drive_res_mm': round((d_hb - d2) * 1000.0, 1)}

    def run_drink(self, fill=0.65, drink_s=6.0):
        """倒水 → 双手捧杯转身 → 前伸递到人的手 → 人喝 → 双手接回 → 放回原位。

        全程复用 P87~P89 已验证的原语, 不在这里写新的闭环逻辑 —— 端到端
        出问题时, 单点验收脚本(test_p87/p88/p89)依然能当作二分排查的把手。
        """
        from vla_follow import drink_motion as M
        from vla_follow import drink_poses as P

        self._busy = True
        self.arm_audit = {}          # P103: 每次任务重新采集姿态审计
        t0 = time.time()
        res = {'ok': False}
        try:
            # 0) 水位归零 —— 水面是两个滑动副、不是真流体, 不归零的话重复
            #    演示时"水涨"这一下就看不到了(上一轮还留在杯里)。
            if 'cup' in self.carry.holding():
                pass                      # 手里还端着杯子: 让它自然保持
            else:
                water_zero_ok = self.carry.joint_ramp(
                    'cup', 'water_joint', 0.0, dur=0.4)
                if not water_zero_ok:
                    water_zero_ok = self.carry.joint(
                        'cup', 'water_joint', 0.0)
                if not water_zero_ok:
                    self._set_state('FAIL_WATER_RESET')
                    self.get_logger().error(
                        'DRINK 水位归零命令无 ACK，拒绝开始新一轮倒水')
                    return False
                self.liquid.reset_cycle()   # P98: 杯内水量记账同步清零
            # 1) 倒水 + 捧杯 + 原地转身面向人
            self._set_state('PREPARE')
            pr = M.prepare_present(self, fill=fill)
            if not pr['ok']:
                # P104: 失败时若物体仍被抓持，保持附着供安全恢复/人工检查；
                # 不得为了“清理场景”在半空中 release，避免道具掉落或飞出。
                self._set_state('FAIL_PREPARE')
                self.get_logger().error('DRINK 失败: 倒水/捧杯阶段')
                return False
            # P101: **双手捧杯**(carry 位, 车前方 0.50m)时的肘部朝向 —— 这是
            # 用户说"双手拿水杯的过程中"最典型的姿态, 也是手臂有余量弯曲、
            # "肘往外架"能真正做出来的构型(递杯位在 0.62~0.70m, 手臂接近伸
            # 直, 肘的横向余量天然很小)。
            res['entry'] = pr.get('entry')       # P102: 入场几何(验收用)
            # P103: 倒水段(抓杯/举杯/倒水)的关节姿态, 由 pour_into_cup 采集
            res['arm'] = dict(pr.get('arm') or {})
            _elc = self._elbow_report('捧杯')
            res['arm']['捧杯'] = self._arm_state('捧杯')
            if _elc:
                res['elbows_carry'] = _elc
            # 2) 双掌前伸到递杯位(= 人身前 0.30m 的手)
            self._set_state('PRESENT')
            res.setdefault('arm', {})['前伸前'] = self._arm_state('前伸前')
            # P94: 人站偏(±0.15m/±20°)时先把"够不着"的那段用底盘补上
            res['approach'] = self._approach_if_far()
            if not res['approach'].get('ok', False):
                self._set_state('FAIL_PRESENT_APPROACH')
                self.get_logger().error(
                    'DRINK 递杯补位未确认安全可达（%s），不伸臂、不交接杯子'
                    % res['approach'])
                return False
            # P96: 人看见杯子来了, 两只手**张开**等着接。摆在这而不是放在
            # PRESENT 开头, 是因为补位会把车往前挪 0.2~0.4m —— 人该在车停稳
            # 之后再伸手, 否则手先张开再看着车拱过来, 不像"接"而像"躲"。
            ok_person_open = M.person_hands(
                self, L.PERSON_HAND_OPEN, dur=0.8, tag='张开迎杯')
            if not ok_person_open:
                self._set_state('FAIL_PERSON_OPEN')
                self.get_logger().error(
                    'DRINK 人手未确认张开，不继续把杯子递向未就绪的手')
                return False
            Tp = self._present_target()
            # P103: 递杯也走 human 分支 —— 'out' 只优化外扩量, 会把 L 侧
            # up_roll 顶到 −1.84(R4 肩内旋超限), 本身就是个违反人体规则的解。
            ok_present, present_mm = M.place_held(
                self, 'L', 'cup', Tp, tol_mm=6.0, iters=14,
                reach_tol_mm=1.5)
            ok_follow, follow_mm = M.follow_hand(
                self, 'R', 'cup', pr['off_r'])
            res['present'] = {'ok': ok_present, 'position_mm': present_mm,
                              'right_hand_ok': ok_follow,
                              'right_hand_mm': follow_mm}
            if not (ok_present and ok_follow):
                self._set_state('FAIL_PRESENT')
                self.get_logger().error(
                    'DRINK 递杯位形失败: 左手=%s(%.1fmm) 右手=%s(%.1fmm)，'
                    '不交给人、不松开杯子'
                    % (ok_present, present_mm, ok_follow, follow_mm))
                return False
            self.spin_for(0.5)
            res['hands'] = {'open_gap_mm': M.hand_gap_mm(self)}
            # 3) **人先双手合拢捧住 → 机器人才松手**。
            #    反过来的话(机器人先松、人再合), 杯子会在两只手中间悬着 0.8s,
            #    画面上是"机器人放手了而人还没接住"。
            #    P94/P96: 顺带把"递杯补位 + 双手开合"的过程数据播出去
            #    (只读验收脚本要用; 这些都是真值测量, 不是回读指令)
            ok_person_grip = M.person_hands(
                self, L.PERSON_HAND_CLOSE, dur=0.8, tag='合拢捧杯')
            if not ok_person_grip:
                self._set_state('FAIL_PERSON_GRIP')
                self.get_logger().error(
                    'DRINK 人手未确认合拢；不执行机器人交接')
                return False
            res['hands']['close_gap_mm'] = M.hand_gap_mm(self)
            # P96: **交接前**的手-杯间距是"递杯精度"(和 P94 的 d_hand 同一个
            # 东西), **交接后**的才是"两只手捧没捧在杯壁上"。混在一起量的话,
            # 实测会把 23mm 的递杯误差(机器人把杯子递偏了)当成"手没捧住"
            # 报 NG —— 而那根本不是手的问题。所以分两组名、分开判。
            _hc = M.hands_to_cup_mm(self)
            if _hc is not None:
                res['hands'].update(recv_l_mm=round(_hc[0], 1),
                                    recv_r_mm=round(_hc[1], 1))
            self._set_state('HANDOVER', {'approach': res['approach'],
                                         'hands': res['hands']})
            ok_h = self.carry.hand_to('cup', 'person', 'person_arms',
                                      L.PERSON_HAND_IN_ARM)
            if not ok_h:
                self._set_state('FAIL_HANDOVER')
                self.get_logger().error(
                    'DRINK 交接到人手失败：保持机器人当前持有/姿态，不执行张手和撤臂')
                return False
            # 交接后杯子被钉到"两手连线中点", 手到杯轴就该正好是相切值。
            # 这一项抓的是**结构性错误**: PERSON_HAND_IN_ARM 与两条手臂
            # link 的实际摆放对不上(改了 SDF 忘了改 scene_layout 才会犯)。
            _hg = M.hands_to_cup_mm(self)
            if _hg is not None:
                res['hands'].update(grip_l_mm=round(_hg[0], 1),
                                    grip_r_mm=round(_hg[1], 1),
                                    grip_dz_mm=round(_hg[2], 1))
                self.get_logger().info(
                    'DRINK 双手捧杯: 手心间距 张开 %.0f → 合拢 %.0fmm  '
                    '交接前手到杯轴 左 %.1f / 右 %.1f mm  交接后 %.1f / %.1f mm'
                    '(相切 %.1f)  高差 %.1fmm'
                    % (res['hands'].get('open_gap_mm') or 0.0,
                       res['hands']['close_gap_mm'],
                       res['hands'].get('recv_l_mm', 0.0),
                       res['hands'].get('recv_r_mm', 0.0),
                       _hg[0], _hg[1], L.CUP_GRIP_R * 1000.0, _hg[2]))
            # P101: 递杯/捧杯时刻的**肘部朝向**(验收用)。口径见
            # `arm_kin.elbow_metrics`; 老版本这里 R 侧是"肘拐进身体中线、
            # 肘尖朝下"(像端着托盘), 用户就是这么指出的。
            _elb = self._elbow_report('递杯')
            if _elb:
                res['elbows'] = _elb
            open_l, _ = M.open_hand(self, 'L')
            open_r, _ = M.open_hand(self, 'R')
            if not (open_l and open_r):
                self._set_state('FAIL_ROBOT_RELEASE_HANDS')
                self.get_logger().error(
                    'DRINK 杯已交给人手，但机器人张手未全部确认；'
                    '停止喝水阶段，避免双方仍有手部接触')
                return False
            rest_l, _ = self.arm['L'].move(K.Q_REST, timeout=10.0)
            rest_r, _ = self.arm['R'].move(K.Q_REST, timeout=10.0)
            if not (rest_l and rest_r):
                self._set_state('FAIL_ROBOT_WITHDRAW')
                self.get_logger().error(
                    'DRINK 机器人空手回到 Q_REST 未全部确认；'
                    '不向人发送 DRINK_READY')
                return False
            self.spin_for(0.6)
            self._tell_person('DRINK_READY')
            # 4) 人喝水: 举杯 → 停一会儿 → 放下
            self._set_state('PERSON_DRINK')
            self.spin_for(1.5)
            # P98: 人真的在喝 —— 仰杯期间水位按速率降到底
            self.liquid.drink_start(drink_s)
            self.bus.set(['person_arm_joint'], [-1.0])
            self.spin_for(drink_s)
            self.bus.set(['person_arm_joint'], [0.0])
            self.liquid.drink_stop()
            self.spin_for(2.0)
            # 5) 双手接回(左手主手 grab, 右手贴上去 —— §6.4 #44)
            self._set_state('TAKE_BACK')
            Tcup = M.object_pose(self, 'cup')
            if Tcup is None:
                self._set_state('FAIL_TAKE_BACK')
                self.get_logger().error(
                    'DRINK 接回阶段缺少水杯真值位姿；不下发接近动作')
                return False
            _okr, rr_r, _ = M.reach(self, 'R', P.cup_cradle_palm(Tcup, 'R'),
                                    tol_mm=5.0)
            _okl, rr_l, _ = M.reach(self, 'L', P.cup_cradle_palm(Tcup, 'L'),
                                    tol_mm=5.0)
            self.get_logger().info(
                'TAKE_BACK reach: R %.1fmm / L %.1fmm  cup=%s'
                % (rr_r, rr_l, np.round(Tcup[:3, 3], 3)))
            if not (_okr and _okl):
                self._set_state('FAIL_TAKE_BACK')
                self.get_logger().error(
                    'DRINK 接回定位失败: R=%s/%.1fmm L=%s/%.1fmm，'
                    '不抓杯、不要求人松手'
                    % (_okr, rr_r, _okl, rr_l))
                return False
            self.spin_for(0.3)
            ok_g = self.carry.grab('cup', 'L')
            if not ok_g:
                self._set_state('FAIL_TAKE_BACK')
                self.get_logger().error(
                    'DRINK 左手抓回水杯失败；人仍应保持交还姿态，不继续撤手')
                return False
            # P96: **人先双手张开 → 机器人才把杯子端走**。顺序不能反 ——
            # 手还合着就端走, 画面上是杯子从两只手里被硬拔出来; 这和松手那套
            # "张手 → 对位 → 松钉 → 撤手"是同一个道理, 只是主体换成了人。
            ok_person_open = M.person_hands(
                self, L.PERSON_HAND_OPEN, dur=0.8, tag='张开交还')
            if not ok_person_open:
                self._set_state('FAIL_PERSON_RELEASE')
                self.get_logger().error(
                    'DRINK 人手未确认张开；机器人保持持杯，不执行撤手/放回')
                return False
            _hg = M.hands_to_cup_mm(self)
            if _hg is not None:
                res['hands'].update(give_l_mm=round(_hg[0], 1),
                                    give_r_mm=round(_hg[1], 1))
                self.get_logger().info(
                    'DRINK 双手交还: 手到杯轴 左 %.1f / 右 %.1f mm '
                    '(松开前 %.1f / %.1f)'
                    % (_hg[0], _hg[1], res['hands'].get('grip_l_mm', 0.0),
                       res['hands'].get('grip_r_mm', 0.0)))
            off_r = M.hand_onto(self, 'R', 'cup',
                                lambda T: P.cup_cradle_palm(T, 'R'))
            if off_r is None:
                self._set_state('FAIL_TAKE_BACK_SUPPORT')
                self.get_logger().error(
                    'DRINK 接回后右手未能贴合杯子；保持左手持杯，不开始放回')
                return False
            # 交完就收手(空手自然垂着时是合着的), 免得人一路张着手走回去
            ok_person_close = M.person_hands(
                self, L.PERSON_HAND_CLOSE, dur=0.6, tag='交还后收手')
            if not ok_person_close:
                self._set_state('FAIL_PERSON_HAND_RESET')
                self.get_logger().error(
                    'DRINK 人手复位失败；机器人保持杯子抓持，不开始放回')
                return False
            self.spin_for(0.5)
            self._tell_person('DRINK_DONE')
            # 6) 转回桌前 → 放回原位
            self._set_state('PUT_BACK')
            ok_turn_back, turn_back_err = M.turn_base(self, 0.0)
            if not ok_turn_back:
                self._set_state('FAIL_PUT_BACK_APPROACH')
                self.get_logger().error(
                    'DRINK 放回前底盘转向失败 %.1f°，不执行桌边放杯'
                    % math.degrees(turn_back_err or 0.0))
                return False
            # P94 #59: **这里不许 place_robot 把底盘瞬移回服务位**。递杯若补过
            # 位, 底盘已经离开服务位 0.24m 了; 而 gz 的 teleport **不清零速度**
            # (§6.4 #20) —— 手里钉着的杯子跟着瞬移之后, release 那一刻会带着
            # 残留速度滑走, 实测 `put_down` 报 1.4mm、25s 后测却是 63.5mm。
            # 本来就**不需要开回去**: 补位后底盘离杯子 0.52m, 比站在服务位
            # (0.56m) 还近; `put_down` 是对**物体真值**闭环的, 与底盘在哪无关。
            # 转身造成的 3~7cm 平移漂移也由这条闭环吸收。
            Tcup_home = P.cup_on_table()
            if M.object_pose(self, 'cup') is None:
                self._set_state('FAIL_PUT_BACK')
                self.get_logger().error(
                    'DRINK 放回阶段缺少水杯真值位姿；保持抓持，不执行放杯')
                return False
            self.get_logger().info(
                'PUT_BACK 目标=%s 当前=%s (base=%s)'
                % (np.round(Tcup_home[:3, 3], 3),
                   np.round(M.object_pose(self, 'cup')[:3, 3], 3),
                   np.round(self.base, 3)))
            # P101: **两只手一起放**。原来是"右手先 open + 撤到 Q_REST, 左手
            # 再 put_down" —— 画面上杯子还悬在半空就松了一只手, 张指那一下还
            # 会蹭到杯壁(2026-10-28 用户指出)。`put_down_two` 分步下降、两只
            # 手全程都扶着, 落稳复核通过后才**同时**张开、松钉、一起撤离。
            _pb = {}
            ok_d, res_d = M.put_down_two(self, 'cup', Tcup_home, info=_pb)
            res['putback'] = _pb
            if not ok_d:
                # drop_mm 仅在 put_down_two 已经完成 release 后写入；
                # 没有它或 carry 仍登记持有，说明水杯未安全落桌，不能继续移动底盘。
                _drop = _pb.get('drop_mm')
                if ('cup' in self.carry.holding() or _drop is None
                        or abs(float(_drop)) > 5.0):
                    self._set_state('FAIL_PUT_BACK')
                    self.get_logger().error(
                        'DRINK 水杯未确认落稳/松钉后发生下落（放回残差 %.1fmm，'
                        'drop=%s），停止撤离'
                        % (res_d, _drop))
                    return False
                # 物体已安全释放，但初始 hands_off 被桌面净空门控拦下；
                # 这不是“放杯失败”，由下面的分阶段收臂路径接手完成安全撤手。
                res['putback']['hands_off_initial_ok'] = False
                res['putback']['placed_release_ok'] = True
                ok_d = True
                self.get_logger().warn(
                    'DRINK 水杯已落稳并解除附着，但初始撤手被碰撞门控拦截；'
                    '转入分阶段收臂')
            # 放回要量"落稳那一刻"(手还捧着、没抬手撤走), 见 put_down_two
            _ar = _pb.get('arm_rest') or self._arm_state('放回')
            res.setdefault('arm', {})['放回'] = _ar
            # 同步进 trace: 这条检查点是在 `put_down_two` 内部采的, 不走
            # `_arm_state`, 所以不会自己落到 arm_audit 里 —— 不补这一步,
            # [6] 全流程 trace 就少了"放回"这一站。
            self.arm_audit['放回'] = _ar
            # P103: 再加一站"撤手之后" —— 松钉→撤手这一小段以前没人量, 结果
            # 这里悄悄挑出过 `up_roll=+1.91 / 肘折 −2.36`(R4/R7 双越)的分支。
            # `hands_off` 现在改成只做局部跟踪, 这一站就是把"不许在这里拧手"
            # 钉成判据。
            res.setdefault('arm', {})['撤手'] = self._arm_state('撤手')
            # P94 #59 兜底: 松手之后杯子有时还在滑(teleport 残留速度 / 撤手
            # 带了一下)。`put_down` 内部测 0.7mm, 等撤离+交权那 25s 走完再
            # 测却偏了 59.9mm(7 组里 yaw+ 这一组)。静置 1s 复核一次, 偏了就
            # 用 `carry.place` 摆回(一次性的, 不重新钉住)。
            self.spin_for(1.0)
            Tc2 = M.object_pose(self, 'cup')
            if Tc2 is not None:
                d_slide = float(np.linalg.norm(
                    Tc2[:3, 3] - np.asarray(Tcup_home)[:3, 3])) * 1000.0
                if d_slide > 5.0:
                    self.get_logger().warn(
                        'DRINK 放回后滑了 %.1fmm, 摆回原位' % d_slide)
                    ok_restore = self.carry.place('cup', Tcup_home)
                    if not ok_restore:
                        self._set_state('FAIL_CUP_RESTORE')
                        self.get_logger().error(
                            'DRINK 放回后杯子偏移 %.1fmm，原位恢复命令无 ACK；'
                            '保持底盘停止，不继续收臂/交权' % d_slide)
                        return False
                    self.spin_for(0.5)
                    Tc2b = M.object_pose(self, 'cup')
                    restore_mm = (float(np.linalg.norm(
                        Tc2b[:3, 3] - np.asarray(Tcup_home)[:3, 3])) * 1000.0
                        if Tc2b is not None else 9e9)
                    if not math.isfinite(restore_mm) or restore_mm > 5.0:
                        self._set_state('FAIL_CUP_RESTORE')
                        self.get_logger().error(
                            'DRINK 杯子原位恢复后仍偏 %.1fmm，停止任务' % restore_mm)
                        return False
                    res_d = max(float(res_d), d_slide)
            # 7) P104: 放杯后立即开始收臂，分“桌边预回收”和“安全净空后完成”。
            # put_down_two 已经执行：落稳→双手张开→release→抬手/向两侧撤离。
            # 这里紧接着只尝试第①段（肘为支点折叠），且每个插值点都过桌面碰撞门控。
            self._set_state('RETREAT')
            _cup0 = M.object_pose(self, 'cup')
            _hp = {}
            _early_fold_ok = True
            for _s in ('R', 'L'):
                _d = {}
                _q, _v = M.hang_natural(
                    self, _s, info=_d, stages=(1,), check_table=True)
                _hp[_s] = {'early': dict(_d)}
                _early_fold_ok = _early_fold_ok and bool(_d.get('ok'))

            # 不必先退满 1.15m：只退到桌沿净距约 0.80m（臂+手全长 0.715m
            # 再留 85mm）就可完成肩部归位/伸肘。底盘位置由真实桌面矩形计算。
            ok_r, res_r = M.drive_to(
                self, L.RETREAT_FOLD_XY, v_max=0.20, tol_mm=25.0,
                tag='收臂净空位')
            _gap_fold = M.table_gap_m(self)
            _full_retreat_done = False
            if not ok_r or _gap_fold < K.ARM_REACH + 0.04:
                # 进度不足时先退到最终安全位；未达到几何净空绝不执行后两段。
                ok_r, res_r = M.drive_to(
                    self, L.SERVE_RETREAT_XY, v_max=0.22,
                    tol_mm=40.0, tag='补足收臂净空')
                _full_retreat_done = True

            # 如果桌边第①段因碰撞风险被拦截，就在净空位重新执行完整三段；
            # 若第①段已安全完成，只执行②/③，避免重新挑选/重做已经完成的折肘。
            _finish_stages = (2, 3) if _early_fold_ok else (1, 2, 3)
            _finish_ok = True
            for _s in ('R', 'L'):
                _d = {}
                _q, _v = M.hang_natural(
                    self, _s, info=_d, stages=_finish_stages,
                    check_table=True)
                _hp[_s]['finish'] = dict(_d)
                _finish_ok = _finish_ok and bool(_d.get('ok'))

            # 若 0.80m 净空位仍被真实几何检查拦截，再退至 1.15m 并重试。
            if not _finish_ok and not _full_retreat_done:
                ok_r, res_r = M.drive_to(
                    self, L.SERVE_RETREAT_XY, v_max=0.22,
                    tol_mm=40.0, tag='扩大收臂净空')
                _full_retreat_done = True
                _finish_ok = True
                for _s in ('R', 'L'):
                    _d = {}
                    _q, _v = M.hang_natural(
                        self, _s, info=_d, stages=(1, 2, 3),
                        check_table=True)
                    _hp[_s]['finish_retry'] = dict(_d)
                    _finish_ok = _finish_ok and bool(_d.get('ok'))

            ok_retract = bool(_finish_ok)
            if not _finish_ok:
                self._set_state('FAIL_ARM_CLEARANCE')
                self.get_logger().error(
                    'DRINK 收臂路径被桌面碰撞门控拦截；停止执行，不穿越桌面，不交还跟随')
                return False

            # 机械臂已悬垂后再完成剩余撤离距离，避免在臂仍伸展时长距离倒车。
            if not _full_retreat_done:
                ok_final, res_final = M.drive_to(
                    self, L.SERVE_RETREAT_XY, v_max=0.22,
                    tol_mm=40.0, tag='悬垂后撤离')
                ok_r = bool(ok_r and ok_final)
                res_r = res_final
                _full_retreat_done = True

            _cup1 = M.object_pose(self, 'cup')
            _g = M.table_gap_m(self)
            res['retreat_geom'] = {
                'gap_at_fold_m': round(_gap_fold, 3),
                'gap_m': round(_g, 3), 'reach_m': round(K.ARM_REACH, 3),
                'clear_mm': round((_g - K.ARM_REACH) * 1e3, 1),
                'early_elbow_fold': bool(_early_fold_ok),
                'cup_drive_mm': _drift(_cup0, _cup1)}
            self.get_logger().info(
                'DRINK 分阶段收臂完成: 早期折肘=%s  收臂净空 %.2fm  最终桌边净距 %.2fm '
                '(臂+手 %.2fm, 余量 %+.0fmm)'
                % (_early_fold_ok, _gap_fold, _g, K.ARM_REACH,
                   (_g - K.ARM_REACH) * 1e3))

            # 汇总 P102/P103 原验收字段：保持每侧扁平字段兼容，同时保留阶段细节。
            for _s in ('R', 'L'):
                _early = _hp[_s].get('early', {})
                _fin = _hp[_s].get('finish_retry') or _hp[_s].get('finish', {})
                _fold_src = _early if _early.get('ok') else _fin
                _hp[_s] = {
                    **_fin,
                    'elbow_drift_mm': _fold_src.get('elbow_drift_mm', 0.0),
                    'fold_deg': _fold_src.get('fold_deg', 0.0),
                    'wrist_deg': _early.get('wrist_deg', _fin.get('wrist_deg', 0.0)),
                    'wrist_step_deg': max(
                        float(_early.get('wrist_step_deg', 0.0) or 0.0),
                        float(_fin.get('wrist_step_deg', 0.0) or 0.0)),
                    'q0': _fold_src.get('q0', _fin.get('q0')),
                    'palm0_m': _fold_src.get('palm0_m', _fin.get('palm0_m')),
                    'early': _early,
                    'finish': _fin}
            res['hang_path'] = _hp
            res.setdefault('arm', {})['悬垂'] = self._arm_state('悬垂')
            res['arm_trace'] = getattr(self, 'arm_audit', {})
            res['retreat_geom']['cup_hang_mm'] = _drift(
                _cup1, M.object_pose(self, 'cup'))
            res['retreat_geom']['cup_total_mm'] = _drift(
                _cup0, M.object_pose(self, 'cup'))
            # 以真实桌沿净空作为交还跟随的硬条件；不能因 move 返回了超时
            # 就忽略机器人仍靠桌太近，也不能在臂仍伸展时交还 follower。
            if not math.isfinite(float(_g)) or _g < K.ARM_REACH + 0.08:
                self._set_state('FAIL_RETREAT_CLEARANCE')
                self.get_logger().error(
                    'DRINK 撤离净空不足 %.3fm（要求 %.3fm），保持停止，不交还跟随'
                    % (_g, K.ARM_REACH + 0.08))
                return False
            ok_r = True
            _h = {}
            for _s in ('R', 'L'):
                _names = ['%s_%s_joint' % (_s, _j[0]) for _j in K.JOINTS]
                _q = np.array([self.js.get(_n, np.nan) for _n in _names])
                _h[_s] = round(float(np.nanmax(
                    np.abs(np.degrees(_q - K.Q_HANG)))), 2)
            res['hang_deg'] = _h
            self.get_logger().info(
                'DRINK 悬垂复位: 臂最大偏差 L %.2f° / R %.2f°'
                % (_h.get('L', -1.0), _h.get('R', -1.0)))
            # P94: **撤离之后再复核一次**。滑动是**缓慢**发生的 —— put_down
            # 后静置 1s 复核时还是 0.7mm, 等撤离这 25s 走完再测就偏了 55.8mm
            # (yaw± 两组都这样)。更要命的是它会**链式污染下一组**: 杯子歪着
            # 留在桌上, 下一组 PREPARE 抓杯直接失败(实测 yaw+ 整组 FAIL)。
            # 交权之前把它摆回原位(一次性 place, 不重新钉住)。
            Tc3 = M.object_pose(self, 'cup')
            if Tc3 is not None:
                d3 = float(np.linalg.norm(
                    Tc3[:3, 3] - np.asarray(Tcup_home)[:3, 3])) * 1000.0
                if d3 > 5.0:
                    self.get_logger().warn(
                        'DRINK 撤离后杯子离原位 %.1fmm(松手时 %.1fmm), 摆回'
                        % (d3, res_d))
                    ok_restore = self.carry.place('cup', Tcup_home)
                    if not ok_restore:
                        self._set_state('FAIL_CUP_RESTORE')
                        self.get_logger().error(
                            'DRINK 撤离后杯子偏移 %.1fmm，原位恢复命令无 ACK；'
                            '不交还跟随控制权' % d3)
                        return False
                    self.spin_for(0.5)
                    Tc3b = M.object_pose(self, 'cup')
                    restore_mm = (float(np.linalg.norm(
                        Tc3b[:3, 3] - np.asarray(Tcup_home)[:3, 3])) * 1000.0
                        if Tc3b is not None else 9e9)
                    if not math.isfinite(restore_mm) or restore_mm > 5.0:
                        self._set_state('FAIL_CUP_RESTORE')
                        self.get_logger().error(
                            'DRINK 撤离后杯子恢复后仍偏 %.1fmm，不交还跟随控制权'
                            % restore_mm)
                        return False
                    res_d = max(float(res_d), d3)
            # 8) 交权: follower 恢复跟随 + cmd_mux 解 300s 喝水锁。
            #    两条都是收到 /robot/command 才动作, 一条消息两处生效。
            self._set_state('HANDBACK')
            _handback_sent_t = time.monotonic()
            self._tell_robot('FOLLOW')
            self.spin_for(1.0)
            _fs = dict(self._follower_status or {})
            handback_ok = (_fs.get('state') == 'FOLLOW'
                           and self._follower_status_t >= _handback_sent_t
                           and time.monotonic() - self._follower_status_t < 1.5)
            res['follower'] = {
                'state': _fs.get('state'),
                'track': _fs.get('track'),
                'lost': bool(_fs.get('lost', False)),
                'fresh': bool(time.monotonic() - self._follower_status_t < 1.5),
            }
            if not handback_ok:
                self._set_state('FAIL_HANDBACK')
                self.get_logger().error(
                    'DRINK 已发送 FOLLOW，但未在 1.5s 内收到 follower 的 FOLLOW 状态确认；'
                    '底盘保持停止，不报告任务成功')
                return False
            self._set_state('DONE')
            # P91: 在 DONE 上停 3s 让验收脚本抓到终态(截图/判完成),
            # 然后**必须回到 IDLE** —— 不然 5s 心跳会一直广播 DONE,
            # 下一轮端到端测试一启动就误判"已完成"(实测踩过)。
            self.spin_for(3.0)
            self._set_state('IDLE')
            # P96: 验收要读的 hands 数据是在 PRESENT/TAKE_BACK 阶段攒的,
            # 别在最后重组 res 时把它丢掉(原来 approach 就是这么没的)。
            # P101: elbows/putback/hang_deg 同理 —— 都是过程里量的, 别丢。
            _keep = {'arm_trace': getattr(self, 'arm_audit', None) or {},
                   'arm': res.get('arm'),
                   'elbows': res.get('elbows'),
                     'elbows_carry': res.get('elbows_carry'),
                     'putback': res.get('putback'),
                     'hang_deg': res.get('hang_deg'),
                     'hang_path': res.get('hang_path'),
                     'retreat_geom': res.get('retreat_geom'),
                     'entry': res.get('entry')}
            res = {'ok': bool(ok_g and ok_d and off_r is not None and ok_h
                                 and ok_present and ok_follow and ok_r
                                 and ok_retract and handback_ok
                                 and pr.get('ok', False)),
                   'cup_home_mm': res_d, 'turn_err': pr.get('turn_err'),
                   'retreat_mm': res_r, 'handback': ok_r,
                   'approach': res.get('approach'),
                   'present': res.get('present'),
                   'hands': res.get('hands'),
                   'follower': res.get('follower'), **_keep}
            self.get_logger().info(
                'DRINK ★ 完成: 接回=%s 放回=%s(%.1fmm) 撤离%.0fmm 交权=%s '
                '用时 %.0fs'
                % (ok_g, ok_d, res_d, res_r, ok_r, time.time() - t0))
            return res['ok']
        except Exception:                            # noqa: BLE001
            self._set_state('ERROR')
            # P98: %r 只有异常类型没有位置, numpy 歧义值这种错根本没法定位,
            # 打完整 traceback。
            import traceback
            self.get_logger().error('DRINK 异常:\n%s'
                                    % traceback.format_exc())
            return False
        finally:
            self._busy = False
            self.pub_state.publish(String(data=json.dumps(
                {'state': self.state, 'result': res}, ensure_ascii=False)))

    def _tick(self):
        # 一旦锁定真值来源, 就每拍把 base 跟到模型世界位姿上 —— 机器人走动、
        # 被 teleport、被 Nav2 摆位, 机械臂的正解/逆解都自动跟着走。
        # P98: 改为**每拍无条件尝试** —— 真值订阅晚于节点就绪时(发现期),
        # 期间 base 被 odom 兜底; 真值一出现立即接管锁定(原来只在
        # _truth_base=True 后才同步, 而 boot 从不调用, 锁永远是 False)。
        self.sync_base_from_truth(quiet=True)
        self.bus.publish_all()
        now = time.time()
        # P98: 道具自愈 —— 壶/杯是自由体, 速度只增不减, 漂出场外后喝水
        # 链路就死了; 长驻进程的 gz 服务调用才可靠(§2.1), 所以重建只能
        # 在这里做。任务执行中(_busy)不动场景。
        if now - getattr(self, '_last_scene_chk', 0.0) > 5.0:
            self._last_scene_chk = now
            if not self._busy and ('kettle' not in self.links
                                   or 'cup' not in self.links):
                try:
                    _liquid_mod.ensure_scene(self)
                except Exception as e:      # 自愈失败不影响主循环
                    self.get_logger().warn('LIQUID 道具自愈异常: %r' % e)
        if now - self._last_hb >= 5.0:
            self._last_hb = now
            self.pub_state.publish(String(data=json.dumps({
                'state': self.state,
                'hz': round(self.bus.hz(), 1),
                'frames': self.bus.count,
                'base': [round(v, 3) for v in self.base],
                'holding': self.carry.holding(),
                # P98: 道具在 links 里的存在性 —— 举起中途真值"消失"的
                # 排查探针(实测抓取时在、place_held 时变 None)。
                'props': [[n, round(self.links[n].position.z, 3)]
                          for n in ('kettle', 'cup', 'cup_water_disc',
                                    'stream0', 'stream2')
                          if n in self.links],
            })))

    def _carry_tick(self):
        """50Hz: 把每个已抓物体的目标位姿刷给世界内插件。"""
        if self.carry.holding():
            self.carry.update()
        self.liquid.update()

    # ------------------------------------------------------------ 动作原语
    def home(self, verbose=True):
        """上电回 home: 双 7 轴 Q_REST、十指张开、水/水流/人的肩归零、
        人的两只手合拢。

        "home" 不是随便选的一个位形 —— 它是 arm_kin.Q_REST, 也是 IK 的零空间
        参考位形。从 home 出发去做任何抓取, 关节行程最短、最不容易撞限位。
        """
        # P94: **这里不许改 self.state**。原来写 `self.state='HOME'` / 末尾
        # `'IDLE'`, 而 `prepare_present` 在 PREPARE 阶段会调 `node.home()` ——
        # 于是 5s 心跳把"正在倒水"广播成了 IDLE, 验收脚本的状态序列出现
        # `PREPARE → IDLE → PRESENT` 这种跳变。阶段由 `_set_state()` 独占
        # 管理, home() 只是个动作。`self.state` 初值在 __init__ 里给(BOOT)。
        if self.carry.items:
            self.get_logger().error(
                'HOME 被拒绝：仍抓持 %s；禁止自动 release，必须先完成安全放置/人工恢复'
                % ','.join(sorted(self.carry.holding())))
            return False
        self.sync_to_measured()
        # P96: 这几路跟左右臂无关, 原来却写在 for 里被发了两遍 —— 趁改这行
        # 把它提出来, 顺带把两只手一起归到合拢位。
        self.bus.set(['jet_joint', 'water_joint', 'person_arm_joint',
                      'person_arm_l_joint', 'person_arm_r_joint'],
                     [0.0, 0.0, 0.0,
                      L.PERSON_HAND_CLOSE, -L.PERSON_HAND_CLOSE])
        home_ok = True
        for s in ('R', 'L'):
            ok_h, err_h = self.hand[s].move(
                K.grip_values(s, K.GRIP_OPEN), timeout=5.0)
            ok_a, err_a = self.arm[s].move(K.Q_REST, timeout=10.0)
            home_ok = home_ok and bool(ok_h and ok_a)
            if verbose:
                self.get_logger().info(
                    '%s 侧 home: 臂 %s (最大残差 %.1f°)  手 %s (%.1f°)'
                    % (s, '到位' if ok_a else '超时',
                       math.degrees(np.nanmax(np.abs(err_a))),
                       '到位' if ok_h else '超时',
                       math.degrees(np.nanmax(np.abs(err_h)))))
        self.spin_for(0.3)
        self._home_motion_ok = bool(home_ok)
        all_ok = all(
            bool(self.js.get('%s_%s_joint' % (s, j[0])) is not None)
            for s in ('R', 'L') for j in K.JOINTS)
        # move() 的布尔结果是复位动作的权威确认；上面只检查状态流仍然存在。
        # 由循环收集本侧关节/手部伺服结果，任何失败都不报告 home 成功。
        return bool(all_ok and getattr(self, '_home_motion_ok', False))

    def hang(self, verbose=True):
        """P98: 悬垂待命 —— 双臂沿体侧竖直向下, 手指并拢伸直与臂同向。

        Q_HANG = 全零: 运动链在零位时沿 -z 竖直下垂(掌心在肩下 0.615m,
        掌系 -z 即正下方), 手部十关节全零 = 并拢伸直顺势朝下。抓取编排
        不受影响 —— run_drink 的 prepare_present 第一动作就是 home()。
        """
        if self.carry.items:
            self.get_logger().error(
                'HANG 被拒绝：仍抓持 %s；禁止自动 release，必须先完成安全放置/人工恢复'
                % ','.join(sorted(self.carry.items.keys())))
            return False
        self.sync_to_measured()
        hang_ok = True
        for s in ('R', 'L'):
            ok_arm, _ = self.arm[s].move(K.Q_HANG, timeout=12.0)
            hang_ok = hang_ok and bool(ok_arm)
        if not hang_ok:
            self.get_logger().error(
                'HANG 至少一侧手臂未确认悬垂；不继续下发手型归位')
            return False
        for s in ('R', 'L'):
            ok_hand, _ = self.hand[s].move(
                K.grip_values(s, K.GRIP_HANG), timeout=8.0)
            hang_ok = hang_ok and bool(ok_hand)
        self.sync_to_measured()
        if not hang_ok:
            self.get_logger().error('HANG 至少一侧灵巧手未确认悬垂手型')
            return False
        if verbose:
            for s in ('R', 'L'):
                names = ['%s_%s_joint' % (s, j[0]) for j in K.JOINTS]
                q = np.array([self.js.get(n, np.nan) for n in names])
                self.get_logger().info(
                    'HANG %s侧悬垂就位: 臂最大|Δ|=%.2f°'
                    % (s, float(np.nanmax(np.abs(np.degrees(
                        q - K.Q_HANG))))))
        return True

    def _srv_hang(self, _req, resp):
        # 与 home 同一规矩: 回调只置标志(见 _srv_home 的 #57 教训),
        # hang() 内部的 spin_for 在回调里重入 executor 会炸节点。
        self._want_hang = True
        resp.success = True
        resp.message = 'queued (async hang)'
        return resp

    def sync_to_measured(self):
        """把总线灌成当前实测值(不动的是已有的量)。

        用途: 从别的状态接管时(比如刚启动、或从 P86 的动作中途接管), 绝不能
        让总线里的旧目标在"还没下发新目标"的间隙里继续生效 —— 那等于先给关节
        一条谁都没打算发的指令。先用实测值兜住, 再写新目标。
        """
        n = 0
        # Sensor callbacks may be serviced by a different executor thread.
        measured = dict(self.js)
        names, values = [], []
        for name, value in measured.items():
            if name in self.bus.val and not math.isnan(value):
                names.append(name)
                values.append(float(value))
        self.bus.set(names, values)
        return len(names)

    def sync_base_from_truth(self, quiet=False):
        """用 gz 真值改写底盘位姿, 并锁定 base 来源(之后 odom 不再覆盖)。

        diff-drive 插件的里程计是**轮子积分**, 底盘被 `place` 直接摆到别处时
        里程计不会跟着变, 于是 FK→世界的变换会整体错位。测试里把机器人摆到
        服务位后就靠这个把 base 拉回真值。

        注意必须取 **模型** `robot` 而不是 link `chassis`:
        `/world/<w>/dynamic_pose/info` 里**模型给的是世界位姿, link 给的是
        相对所属模型的位姿**(实测机器人站在 (1.73,0.35) 时, `chassis` 报的
        是 (0.000, 0.000, 0.130))。P84/P85 的机器人一直在原点, 两者恰好相等,
        所以这个差别一直没暴露。
        """
        for name in ('robot', 'chassis', 'base_link'):
            p = self.links.get(name)
            if p is not None:
                self.base = (p.position.x, p.position.y,
                             yaw_of(p.orientation))
                self._have_odom = True
                self._truth_base = True
                if not quiet:
                    self.get_logger().info(
                        'base 取自真值 %s: (%.3f, %.3f, %.1f°)'
                        % (name, self.base[0], self.base[1],
                           math.degrees(self.base[2])))
                return True
        return False

    # ------------------------------------------------------------ 自检
    def ck_report(self):
        """三方对齐自检: 指令 vs 实测, FK vs gz 真值。返回 (文本, 是否通过)。"""
        out = []
        ok = True
        T_base = K.base_pose(*self.base)
        out.append('底盘 base = (%.3f, %.3f, yaw %.1f°)%s'
                   % (self.base[0], self.base[1],
                      math.degrees(self.base[2]),
                      '' if self._have_odom else '  [警告] 未收到 /robot/odom'))
        for s in ('R', 'L'):
            names = ['%s_%s_joint' % (s, j[0]) for j in K.JOINTS]
            miss = [n for n in names if n not in self.js]
            if miss:
                out.append('%s 侧缺关节实测: %s' % (s, miss))
                ok = False
                continue
            q = np.array([self.js[n] for n in names], float)
            q_cmd = self.bus.get_many(names)
            dq = np.degrees(q - q_cmd)
            T_fk = K.fk(s, q)                      # 本体坐标系
            T_fk_w = T_base @ T_fk                 # 世界坐标系
            tw = T_fk_w[:3, 3]
            out.append('%s 侧' % s)
            out.append('   指令-实测(°)  %s   最大 %.1f°'
                       % (' '.join('%+6.1f' % v for v in dq),
                          float(np.max(np.abs(dq)))))
            out.append('   掌心 FK(世界)  (%+.4f %+.4f %+.4f)'
                       % (tw[0], tw[1], tw[2]))
            if abs(np.max(dq)) > ARM_TOL_DEG * 2.0:
                ok = False
            key = '%s_palm' % s
            if key in self.links:
                T_gt = gz_pose_to_T(self.links[key])
                e = K.pose_err(T_fk_w, T_gt)
                dp = float(np.linalg.norm(e[:3]))
                dr = float(np.linalg.norm(e[3:]))
                out.append('   掌心 真值      (%+.4f %+.4f %+.4f)'
                           % (T_gt[0, 3], T_gt[1, 3], T_gt[2, 3]))
                out.append('   FK-真值       位置 %.1f mm  姿态 %.2f°'
                           % (dp * 1000.0, math.degrees(dr)))
                if dp > CK_POS_TOL or dr > CK_ROT_TOL:
                    ok = False
            else:
                out.append('   掌心 真值      缺失(未收到 dynamic_pose/info)')
                ok = False
        return '\n'.join(out), ok

    def _srv_ck(self, _req, resp):
        text, ok = self.ck_report()
        resp.success = bool(ok)
        resp.message = text
        self.get_logger().info('=== /drink/ck ===\n%s' % text)
        return resp

    def _srv_home(self, _req, resp):
        # home() 内部要 spin_for(再进 spin_once), 在服务回调里直接调会
        # "Executor is already spinning" 把节点炸掉(实测 2026-10-27)。
        # 与 DRINK 同一个规矩: 回调只置标志, 主循环里执行。
        self._want_home = True
        resp.success = True
        resp.message = 'queued (async home)'
        return resp


def main():
    once = '--once' in sys.argv
    rclpy.init()
    node = DrinkTask()
    try:
        node.spin_for(1.5)                    # 等第一帧 joint_state
        # P98: 开机先锁真值 base —— gz 订阅的发现期有 1~3s, 在此之前
        # _odom_cb 会用**漂移的轮子里程计**持续覆盖 base(实测 base 被带到
        # (-23.5,4.46), 离真值 25m, 所有 FK/IK 世界系全废)。轮询到真值出现
        # 为止, 之后的每拍 tick 持续跟随。
        t_sync = time.time()
        while not node.sync_base_from_truth(quiet=True) \
                and time.time() - t_sync < 15.0:
            node.spin_for(0.1)
        node.home()
        # P98: 初始状态 = 悬垂(双臂沿体侧竖直向下, 手指并拢伸直朝下)。
        # 抓取编排不受影响 —— run_drink 的 prepare_present 第一动作就是 home()。
        node.hang()
        # P94: home() 不再顺手改 self.state(它会把 PREPARE 播成 IDLE), 所以
        # "上电回 home 完成 = 待命"这一步要显式广播。
        node._set_state('IDLE')               # noqa: SLF001
        node.spin_for(0.5)
        text, ok = node.ck_report()
        print('=' * 74)
        print('P85 自检 —— 关节通道 / 静态姿态')
        print('=' * 74)
        print(text)
        print('-' * 74)
        print('指令节拍 %.1f Hz (目标 %.0f Hz), 已发 %d 帧'
              % (node.bus.hz(), PUB_HZ, node.bus.count))
        print('结论: %s' % ('通过' if ok else '不通过'))
        print('=' * 74)
        if once:
            return 0 if ok else 2
        print('DRINK 待命: /robot/command {"action":"DRINK"} 触发整套任务',
              flush=True)
        while rclpy.ok():                     # 常驻: 保持 20Hz 下发
            # P90: 任务本身要阻塞执行(`run_drink` 内部 spin_for 会再进
            # spin_once), 所以**只能在这里调**, 不能在回调里调。
            if node._want_drink and not node._busy:
                node._want_drink = False
                node.run_drink()
                continue
            if node._want_home and not node._busy:
                node._want_home = False
                node.home()
                continue
            if node._want_hang and not node._busy:
                node._want_hang = False
                node.hang()
                continue
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
