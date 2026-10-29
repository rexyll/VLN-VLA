#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cmd_mux.py —— 底盘速度复用器 (P65 / P71)

输入: /follower_cmd_vel  (自研控制器, 对正/跟踪/扫描)
      /cmd_vel_smoothed  (Nav2 velocity_smoother 输出, 搜索段)
      /nav2_active       (Bool, follower 发布的模式开关)
      /perception/proximity (JSON, 4 向深度近距 —— 最终硬闸)
输出: /robot/cmd_vel (唯一写底盘的话题)

规则:
  - nav2_active=True → 转发 Nav2 速度; False → 转发自研速度
  - 看门狗: nav 模式下 >0.6s 无 Nav2 速度 → 输出零(防控制器死亡直通)
  - 硬闸(只加强不削弱, L4): 按行进方向判定
      · 正朝前/后走且该向深度 < HARD_STOP_M(0.28) → 线速度清零
      · 任意方向深度 < BLIND_STOP_M(0.15) → 线速度清零(贴死即停)
      · 转向不做限制(留出脱困能力), 侧向近距不闸前进(否则过柱会冻住)

P71 变更:
  - 订阅话题 /nav2_cmd_vel → /cmd_vel_smoothed。Jazzy 的
    navigation_launch.py 把 controller_server 的 cmd_vel 重映射为
    cmd_vel_nav 喂给 velocity_smoother, smoother 输出 /cmd_vel_smoothed;
    /nav2_cmd_vel 在本链路上根本不存在(实测 发布0/订阅1) → GOTO 段
    看门狗 0.6s 必然触发, 速度恒为 0。
  - 硬闸从"只看 front"改为按行进方向判定: 前进看 front、后退看 back,
    另加"任意方向 < 0.15m 直接停"的贴死兜底。
"""
import json
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

HARD_STOP_M = 0.28      # 该方向深度小于此值且正朝它走 → 停线速度
BLIND_STOP_M = 0.15     # 任意方向贴死 → 无视朝向直接停
NAV_TIMEOUT_S = 0.6     # Nav2 速度断流判定
NAV_GRACE_S = 2.0       # 刚切到 Nav2 时的宽限(Nav2 要先规划再出速度)
DRINK_LOCK_S = 300.0    # 喝水锁的初始时长
DRINK_LEASE_S = 120.0   # 任务心跳续租: 见到 /drink/cmd_vel 或
                        # /drink/joint_state 就把锁推到 now+120s
LIVE_TIMEOUT_S = 2.0    # 判「任务还活着」的心跳窗口
# proximity JSON 里除 4 个主方位外还有 fl/fr/bl/br 和 age 等字段,
# 硬闸只能读下面这 4 个键 —— 否则会把 age(秒, 恒 <0.15)当成"贴死"。
DIRS = ('front', 'left', 'right', 'back')


class CmdMux(Node):

    def __init__(self):
        super().__init__('cmd_mux')
        self.pub = self.create_publisher(Twist, '/robot/cmd_vel', 20)
        self.create_subscription(Twist, '/follower_cmd_vel',
                                 self._fol_cb, 20)
        self.create_subscription(Twist, '/cmd_vel_smoothed', self._nav_cb, 20)
        self.create_subscription(Bool, '/nav2_active', self._act_cb, 10)
        self.create_subscription(String, '/perception/proximity',
                                 self._prox_cb, 10)
        # P102: 喝水任务**自己的**速度单独走一条话题, 由本节点在锁内放行。
        # 不能让它和本节点共用 /robot/cmd_vel: 两边都是 20Hz, 谁后到谁生效,
        # 实测撤离 1.15m 走成 25s(应为 5s), 甚至整段 30s 一步没走(残差 1377mm)。
        self.create_subscription(Twist, '/drink/cmd_vel', self._drk_cb, 20)
        # 喝水任务的**心跳**: 它按 20Hz 发 /drink/joint_state, 只要还在
        # 发就说明任务还活着 → 锁要一直续下去(见 _tick 的滚动续租)。
        self._live_t = 0.0
        self.create_subscription(JointState, '/drink/joint_state',
                                 self._live_cb, 5)
        self.fol = Twist()
        self.nav = Twist()
        self.drk = Twist()
        self.drk_t = 0.0
        self.nav_t = 0.0
        self.active = False
        self.prox = {}
        self._gate_t = 0.0
        # P90: 喝水任务期间底盘必须绝对静止 —— 递杯位是按真值 base 现算的,
        # 车一动整个递杯就偏(§6.4 #35/#43)。follower 收到 DRINK 已经停车让权,
        # 但 Nav2 若有残留目标仍会从 /cmd_vel_smoothed 推车, 手动遥控同理。
        # 所以在这里再上一道锁: DRINK 之后 300s 内两个输入源一律置零。
        self._drink_until = 0.0
        self.create_subscription(String, '/robot/command', self._cmd_cb, 10)
        self.create_timer(0.05, self._tick)

    def _cmd_cb(self, m):
        try:
            action = json.loads(m.data).get('action')
        except Exception:
            return
        if action == 'DRINK':
            self._drink_until = time.monotonic() + DRINK_LOCK_S
            self._live_t = time.monotonic()
            self.get_logger().info(
                'CMD_MUX ★ 喝水任务接管, 底盘输入锁定 %.0fs(任务活着就续租)'
                % DRINK_LOCK_S)
        elif action in ('STOP', 'FOLLOW'):
            if self._drink_until > 0.0:
                self.get_logger().info('CMD_MUX 解除喝水锁定')
            self._drink_until = 0.0

    def _fol_cb(self, m):
        self.fol = m

    def _nav_cb(self, m):
        self.nav = m
        self.nav_t = time.monotonic()

    def _drk_cb(self, m):
        self.drk = m
        self.drk_t = time.monotonic()

    def _live_cb(self, _m):
        self._live_t = time.monotonic()

    def _act_cb(self, m):
        if m.data != self.active:
            self.get_logger().info('CMD_MUX 模式 → %s'
                                   % ('Nav2' if m.data else '自研控制器'))
            if m.data:
                # 刚接管时 Nav2 还在规划, 不可能立刻有速度 → 给一段宽限,
                # 否则每次进 GOTO 都会先报一次"断流急停"
                self.nav_t = time.monotonic() + NAV_GRACE_S - NAV_TIMEOUT_S
        self.active = m.data

    def _prox_cb(self, m):
        try:
            self.prox = json.loads(m.data)
        except Exception:
            pass

    def _dist(self, name):
        """取某个主方位的深度(米); 缺失/非数值一律当"未知"(-1)。"""
        try:
            return float(self.prox.get(name, -1.0))
        except (TypeError, ValueError):
            return -1.0

    def _blocked_by(self, out):
        """线速度正在朝哪个方位走、该方位是否已近到必须停。

        prox 的四个方位是本体坐标系下的 (front/left/right/back), 所以
        前进只看 front、后退只看 back; 原地转向(vx=0)不做线速度闸。
        侧向近距不闸线速度 —— 否则贴着立柱经过时会直接冻住, 反而丢掉
        "跟紧"这条主需求; 侧向贴死由 _tick 里的 BLIND_STOP 兜底。
        返回 (方位名, 距离) 或 None。
        """
        vx = out.linear.x
        if vx == 0.0:
            return None
        name = 'front' if vx > 0 else 'back'
        d = self._dist(name)
        if 0 < d < HARD_STOP_M:
            return (name, d)
        return None

    def _tick(self):
        out = Twist()
        _now = time.monotonic()
        # 滚动续租: 任务还在发心跳(或速度)就把锁往后推。原来锁是**固定
        # 300s**, 而低 RTF 下整条链要 330~390s —— 锁在 RETREAT 中途到期,
        # cmd_mux 立刻回落到 follower 的零速, 任务发的 /drink/cmd_vel 被
        # 丢掉, 撤离一步没走(实测残差 1403mm、gap −0.03m)。任务自己崩了也
        # 不至于永久占锁: 心跳停了最多续 120s 就自动放开。
        if self._drink_until > 0.0 and (_now - self._live_t) < LIVE_TIMEOUT_S:
            self._drink_until = max(self._drink_until, _now + DRINK_LEASE_S)
        if self._drink_until > 0.0 and _now < self._drink_until:
            # 喝水任务期间: follower / Nav2 一律置零, 但**喝水任务自己的速度
            # 要放行**(走它自己的 /drink/cmd_vel)。这一段也不吃近距硬闸 ——
            # 倒水/撤离/驶入都是脚本化的直线, 驶入本来就要贴到桌沿 0.21m
            # (小于 HARD_STOP_M 0.28), 过闸会把整段驶入清零。
            # 断流保护: 喝水任务 0.5s 没发速度就发零(它自己崩了也不能让车跑)。
            self.pub.publish(self.drk if time.monotonic() - self.drk_t < 0.5
                             else Twist())
            return
        if self.active:
            if time.monotonic() - self.nav_t > NAV_TIMEOUT_S:
                self.get_logger().warn('CMD_MUX Nav2 速度断流 >%.1fs, 急停'
                                       % NAV_TIMEOUT_S,
                                       throttle_duration_sec=2.0)
            else:
                out = self.nav
        else:
            out = self.fol
        # --- 硬闸(两种模式都生效) ---
        blind = [n for n in DIRS if 0 < self._dist(n) < BLIND_STOP_M]
        if blind and out.linear.x != 0.0:
            self.get_logger().warn(
                'CMD_MUX 硬闸: %s 贴死(<%.2fm), 线速度清零'
                % ('/'.join(blind), BLIND_STOP_M),
                throttle_duration_sec=1.0)
            out.linear.x = 0.0
        else:
            hit = self._blocked_by(out)
            if hit is not None:
                self.get_logger().warn(
                    'CMD_MUX 硬闸: 朝 %s 行进但距离 %.2fm < %.2fm, 线速度清零'
                    % (hit[0], hit[1], HARD_STOP_M),
                    throttle_duration_sec=1.0)
                out.linear.x = 0.0
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
