#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cmd_mux.py —— 底盘速度复用器 (P65)

输入: /follower_cmd_vel (自研控制器, 对正/跟踪/扫描)
      /nav2_cmd_vel     (Nav2 速度平滑器输出, 搜索段)
      /nav2_active      (Bool, follower 发布的模式开关)
      /perception/proximity (JSON, 4 向深度近距 —— 最终硬闸)
输出: /robot/cmd_vel (唯一写底盘的话题)
规则:
  - nav2_active=True → 转发 Nav2 速度; False → 转发自研速度
  - 看门狗: nav 模式下 >0.6s 无 Nav2 速度 → 输出零(防控制器死亡直通)
  - 硬闸(只加强不削弱, L4): 任一方向深度 <0.30m 且机器人朝该方向 → 线速度清零
"""
import json
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class CmdMux(Node):

    def __init__(self):
        super().__init__('cmd_mux')
        self.pub = self.create_publisher(Twist, '/robot/cmd_vel', 20)
        self.create_subscription(Twist, '/follower_cmd_vel',
                                 self._fol_cb, 20)
        self.create_subscription(Twist, '/nav2_cmd_vel', self._nav_cb, 20)
        self.create_subscription(Bool, '/nav2_active', self._act_cb, 10)
        from std_msgs.msg import String
        self.create_subscription(String, '/perception/proximity',
                                 self._prox_cb, 10)
        self.fol = Twist()
        self.nav = Twist()
        self.nav_t = 0.0
        self.active = False
        self.prox = {}
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
        self.active = m.data

    def _prox_cb(self, m):
        try:
            self.prox = json.loads(m.data)
        except Exception:
            pass

    def _tick(self):
        out = Twist()
        if self.active:
            if time.monotonic() - self.nav_t > 0.6:
                self.get_logger().warn('CMD_MUX Nav2 速度断流 >0.6s, 急停',
                                       throttle_duration_sec=2.0)
            else:
                out = self.nav
            # 硬闸: 朝向深度 <0.30m 的方向 → 线速度清零(保留转向脱困)
            f = self.prox.get('front', -1.0)
            if 0 < f < 0.30 and out.linear.x > 0:
                out.linear.x = 0.0
        else:
            out = self.fol
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
