#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pose_demo.py —— 让双臂 + 灵巧手走一段"能看清"的动作序列

用途:
    P85 之前先给一个纯姿态演示, 用来肉眼确认
      * 7 轴臂能不能按指令到位(上下俯仰 / 左右摆 / 腕部翻转)
      * 10 路手指能不能张开、握拢、对指
      * 左右两条臂是否对称、有没有装反
    不影响任何任务逻辑, 也不碰桌上的壶/杯。

用法(需先 source install/setup.bash, 世界已在跑):
    python3 tools/pose_demo.py            # 双侧, 约 25s
    python3 tools/pose_demo.py R          # 只动右臂
    python3 tools/pose_demo.py --fast     # 快一点(每步停留减半)
"""
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from vla_follow import arm_kin as K
from vla_follow.joint_servo import JointServo

SIDES = [a for a in sys.argv[1:] if a in ('R', 'L')] or ['R', 'L']
FAST = '--fast' in sys.argv
HOLD = 0.7 if FAST else 1.4


class Demo(Node):

    def __init__(self):
        super().__init__('pose_demo')
        self.js = {}
        self.create_subscription(JointState, '/drink/joint_state',
                                 self._cb, 20)

    def _cb(self, msg):
        self.js.update(zip(msg.name, msg.position))

    def spin_for(self, s):
        t0 = time.time()
        while time.time() - t0 < s:
            rclpy.spin_once(self, timeout_sec=0.005)


def main():
    rclpy.init()
    n = Demo()
    n.spin_for(1.0)          # 等第一帧 joint_state

    arms = {s: JointServo(n, s) for s in SIDES}
    hands = {s: JointServo(n, s,
                           joint_names=['%s_joint' % j
                                        for j in K.hand_joints(s)],
                           tol_deg=3.0) for s in SIDES}

    def step(msg):
        print('  · %s' % msg)

    # ---- 1) 收回到 Q_REST(从全 0 位形开始)
    step('双臂收拢到 Q_REST')
    for s in SIDES:
        arms[s].move(K.Q_REST, timeout=8.0)
    n.spin_for(HOLD)

    # ---- 2) 手掌完全张开
    step('十指张开')
    for s in SIDES:
        hands[s].move(K.grip_values(s, K.GRIP_OPEN), timeout=4.0, tol_deg=6.0)
    n.spin_for(HOLD)

    # ---- 3) 双臂前伸(掌心朝下/朝前, 便于看清手指)
    step('双臂前伸')
    present = [0.0, -1.10, 0.0, -0.30, 0.0, 0.45, 0.0]
    for s in SIDES:
        arms[s].move(present, timeout=8.0)
    n.spin_for(HOLD * 1.5)

    # ---- 4) 握拢 → 张开, 两轮
    for i in range(2):
        step('握拢 (抓水壶力度) #%d' % (i + 1))
        for s in SIDES:
            hands[s].move(K.grip_values(s, K.GRIP_KETTLE), timeout=4.0,
                          tol_deg=6.0)
        n.spin_for(HOLD)
        step('张开 #%d' % (i + 1))
        for s in SIDES:
            hands[s].move(K.grip_values(s, K.GRIP_OPEN), timeout=4.0,
                          tol_deg=6.0)
        n.spin_for(HOLD)

    # ---- 5) 腕部翻转 + 肘部弯曲, 检查 pitch/roll 轴
    step('腕部翻转 (wrist_p / wrist_r)')
    for v in (0.9, -0.6, 0.45):
        q = list(present)
        q[5] = v
        q[6] = 0.8 * v
        for s in SIDES:
            arms[s].move(q, timeout=6.0)
        n.spin_for(HOLD * 0.8)

    # ---- 6) 肩部左右摆(pan) + 俯仰(lift)
    step('肩部左右摆 (pan ±0.6)')
    for v in (0.6, -0.6, 0.0):
        q = list(present)
        q[0] = v
        for s in SIDES:
            arms[s].move(q, timeout=6.0)
        n.spin_for(HOLD * 0.8)

    # ---- 7) 收回 Q_REST
    step('收回 Q_REST')
    for s in SIDES:
        arms[s].move(K.Q_REST, timeout=8.0)
    for s in SIDES:
        hands[s].move(K.grip_values(s, K.GRIP_OPEN), timeout=4.0, tol_deg=6.0)
    print('完成')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
