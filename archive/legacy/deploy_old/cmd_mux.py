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
from std_msgs.msg import Bool, String

HARD_STOP_M = 0.28      # 该方向深度小于此值且正朝它走 → 停线速度
BLIND_STOP_M = 0.15     # 任意方向贴死 → 无视朝向直接停
NAV_TIMEOUT_S = 0.6     # Nav2 速度断流判定
NAV_GRACE_S = 2.0       # 刚切到 Nav2 时的宽限(Nav2 要先规划再出速度)
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
        self.fol = Twist()
        self.nav = Twist()
        self.nav_t = 0.0
        self.active = False
        self.prox = {}
        self._gate_t = 0.0
        self.create_timer(0.05, self._tick)

    def _fol_cb(self, m):
        self.fol = m

    def _nav_cb(self, m):
        self.nav = m
        self.nav_t = time.monotonic()

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
