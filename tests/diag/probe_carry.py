#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""probe_carry.py —— 验证 GraspCarry 插件: 附着后物体是否刚性跟随掌心

P86 的附着方案是"世界内插件每 tick 写 WorldPoseCmd", 本探针端到端验证它:

  [1] 读初始 T_kettle / T_R_palm, 记下相对位姿 T_rel
  [2] 通过 ROS `/drink/carry` 发 "attach kettle R_palm"
  [3] 下发 R_pan_joint += 0.40 rad, 等运动结束
  [4] 掌心动了多少 / 壶动了多少 / 两者相对位姿变了多少
      —— 判据: 相对位姿变化 < 2mm / 1°(真"刚性")
  [5] 发 "detach kettle", 再动 pan -0.40 rad
      —— 判据: 壶位移 < 2mm(松开就停住, 不跟着手回去)
  [6] 复原 pan = 0

用法: python3 tests/diag/probe_carry.py
"""
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

import gz.transport13 as gz_transport
from gz.msgs10.pose_v_pb2 import Pose_V

from vla_follow import arm_kin as K

TOPIC = '/world/follow_world/dynamic_pose/info'
MODEL = 'kettle'
PALM = 'R_palm'
# 用 lift 而不是 pan: pan 是肩部**竖轴**的偏航, 而手臂垂直下垂时掌心正好在
# 这条轴上 —— 转 pan 掌心位置一点不动, 只有姿态变(第一版探针就栽在这,
# "掌心移动 0.0mm" 被判成失败, 实际插件行为完全正确)。
# lift 绕肩部 y 轴摆动, 掌心会实实在在地平移。
SWING_CMD = '/drink/cmd/R_lift_joint'
SWING = -0.45
CARRY = '/drink/carry'

link = {}


def to_T(p):
    q = p.orientation
    return K.frame(K.quat_to_mat(q.x, q.y, q.z, q.w),
                   (p.position.x, p.position.y, p.position.z))


class Probe(Node):
    def __init__(self):
        super().__init__('probe_carry')
        self.pub_carry = self.create_publisher(String, CARRY, 10)
        self.create_subscription(String, CARRY + '/ack', self._ack_cb, 10)
        self.ack = None
        self.pub_sw = self.create_publisher(Float64, SWING_CMD, 10)
        self.gzn = gz_transport.Node()
        self.gzn.subscribe(Pose_V, TOPIC, self._cb)
        self.js = {}
        self.create_subscription(JointState, '/drink/joint_state',
                                 self._js_cb, 10)
        self.sw = 0.0

    def _js_cb(self, msg):
        self.js.update(zip(msg.name, msg.position))

    def fk_palm(self):
        """第三条独立通道: 关节角 + 正解算出掌心世界位姿。

        用来裁决"话题里的 R_palm 是否新鲜": 插件按 ECM 现算, 话题走
        SceneBroadcaster 的缓存, 若两者不同, 以 FK 为准 —— FK 只依赖关节角,
        与位姿话题和 ECM 缓存都无关。
        """
        names = ['R_%s_joint' % j[0] for j in K.JOINTS]
        if any(n not in self.js for n in names):
            return None
        q = np.array([self.js[n] for n in names], float)
        return K.fk('R', q)

    def _ack_cb(self, msg):
        self.ack = msg.data

    def _cb(self, msg):
        for p in msg.pose:
            link[p.name] = p

    def spin_for(self, s):
        t0 = time.time()
        while time.time() - t0 < s:
            rclpy.spin_once(self, timeout_sec=0.01)

    def carry(self, text, timeout=4.0):
        """发指令并等 ack。为什么不是"发一条就算": 单向话题在发现完成前会
        静默丢消息(实测第一轮 attach 连发三次全丢, 5 秒后的 detach 却正常),
        必须靠 ack 确认, 否则会出现"以为抓上了其实没抓"。"""
        parts = text.split()
        op = parts[0]
        model = parts[1] if len(parts) > 1 else ''
        self.ack = None
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.pub_carry.publish(String(data=text))
            self.spin_for(0.08)
            if self.ack and self.ack.split()[0] == 'ok' \
                    and op in self.ack and (not model or model in self.ack):
                self.get_logger().info('ack: %s' % self.ack)
                self.spin_for(0.3)
                return True
        self.get_logger().error('carry 无 ack: %s (last=%r)' % (text, self.ack))
        return False

    def set_swing(self, rad, settle=3.0):
        m = Float64()
        m.data = float(rad)
        self.sw = float(rad)
        t0 = time.time()
        while time.time() - t0 < settle:
            self.pub_sw.publish(m)
            self.spin_for(0.02)

    def T(self, name):
        return to_T(link[name])


def rel_err(T_ref, Ta, Tb):
    """两段相对位姿 Ta/ Tb 之间的差异 -> (位置 mm, 姿态 °)"""
    R = (np.linalg.inv(T_ref) @ Ta, np.linalg.inv(T_ref) @ Tb)
    e = K.pose_err(R[0], R[1])
    return float(np.linalg.norm(e[:3])) * 1000.0, math.degrees(
        float(np.linalg.norm(e[3:])))


def main():
    rclpy.init()
    n = Probe()
    n.spin_for(2.5)
    if MODEL not in link or PALM not in link:
        print('[!] 缺 %s / %s, 现有: %s' % (MODEL, PALM, sorted(link)[:12]))
        return 2

    print('=' * 74)
    print('GraspCarry 附着验证')
    print('=' * 74)
    Tk0, Tp0 = n.T(MODEL), n.T(PALM)
    r0 = np.linalg.inv(Tp0) @ Tk0
    print('[1] 初始  壶 %s  掌心 %s'
          % (K.lamp(Tk0), K.lamp(Tp0)))
    print('    相对位姿 |pos| = %.4f m' % np.linalg.norm(r0[:3, 3]))

    # ---- [2] attach
    n.carry('attach %s %s' % (MODEL, PALM))
    # attach 完成、还没动: 此时的相对位姿偏差就是"静态系统性偏差"
    Tk_a, Tp_a = n.T(MODEL), n.T(PALM)
    e0p, e0r = rel_err(Tp_a, Tk_a, Tp_a @ r0)
    print('    attach 静态偏差  位置 %.1f mm  姿态 %.2f°' % (e0p, e0r))

    # ---- [3] 动 lift
    n.set_swing(SWING)
    # 手臂停稳后连采 5 次, 看相对位姿偏差是"还在收敛"还是"钉住停了"
    seq = []
    for _ in range(5):
        Tk_s, Tp_s = n.T(MODEL), n.T(PALM)
        seq.append(rel_err(Tp_s, Tk_s, Tp_s @ r0))
        n.spin_for(0.4)
    print('    停稳后逐次偏差(mm/°)  ' +
          '  '.join('%.1f/%.2f' % (p, q) for p, q in seq))
    Tk1, Tp1 = n.T(MODEL), n.T(PALM)
    dp = float(np.linalg.norm(Tp1[:3, 3] - Tp0[:3, 3]))
    dk = float(np.linalg.norm(Tk1[:3, 3] - Tk0[:3, 3]))
    ep, er = seq[-1]
    print('[2] attach 后动 lift %+.2f rad' % SWING)
    print("    掌心移动 %.1f mm   壶移动 %.1f mm" % (dp * 1000, dk * 1000))

    print('    相对位姿偏差  位置 %.1f mm  姿态 %.2f°   %s'
          % (ep, er, 'OK(刚性)' if (ep < 2.0 and er < 1.0) else 'NG(没跟上或漂移)'))
    ok2 = dp > 0.02 and ep < 2.0 and er < 1.0

    # ---- [3] 摆回原位: 判别"动态滞后"还是"几何偏差"
    # 滞后型误差与运动速度成正比, 手臂回到原姿态后应随之归零;
    # 若仍停在 0.76° 附近, 就是与位姿求解有关的几何偏差, 得换实现。
    n.set_swing(0.0)
    n.spin_for(0.5)
    Tk_b, Tp_b = n.T(MODEL), n.T(PALM)
    ebp, ebr = rel_err(Tp_b, Tk_b, Tp_b @ r0)
    Tk2 = Tk_b
    print('[3] 摆回原位后的偏差  位置 %.1f mm  姿态 %.2f°   %s'
          % (ebp, ebr, '滞后型(随运动消失)' if ebr < 0.2 else '几何偏差(需换实现)'))

    # ---- [3b] 三方对照: 话题掌心 vs FK 掌心
    Tfk = n.fk_palm()
    if Tfk is not None:
        Tk_c, Tp_c = n.T(MODEL), n.T(PALM)
        d_topic_fk = float(np.linalg.norm(Tp_c[:3, 3] - Tfk[:3, 3])) * 1000.0
        e_fk_p, e_fk_r = rel_err(Tfk, Tk_c, Tfk @ r0)
        print('[3b] 话题掌心 %s' % K.lamp(Tp_c))
        print('     FK 掌心 %s' % K.lamp(Tfk))
        print('     话题-FK 相差 %.1f mm' % d_topic_fk)
        print('     以 FK 为基准的相对位姿偏差  %.1f mm / %.2f°  %s'
              % (e_fk_p, e_fk_r,
                 'OK(说明话题在骗人)' if e_fk_p < 5.0 and e_fk_r < 0.5
                 else 'NG(插件真没跟上)'))
    else:
        print('[3b] 没有 /drink/joint_state, 跳过三方对照')

    # ---- [4] detach 再动一次
    n.carry('detach %s' % MODEL)
    n.set_swing(SWING)
    Tk3, Tp3 = n.T(MODEL), n.T(PALM)
    dk2 = float(np.linalg.norm(Tk3[:3, 3] - Tk2[:3, 3]))
    dp2 = float(np.linalg.norm(Tp3[:3, 3] - Tp_b[:3, 3]))
    print('[4] detach 后再动 lift (掌心移动 %.1f mm)' % (dp2 * 1000))
    print('    壶残留位移 %.1f mm   %s'
          % (dk2 * 1000, 'OK(松开就停住)' if dk2 < 0.002 else 'NG(还在跟着手)'))
    ok3 = dk2 < 0.002

    # 判定: 附着必须"跟得上且相对位姿稳定"; 松手必须"停住"
    rigid = ep < 2.0 and er < 1.0
    print('-' * 74)
    print('结论: %s' % ('通过 —— 附着刚性 / 松开停住' if (rigid and ok3)
                       else '不通过'))
    print('=' * 74)
    n.destroy_node()
    rclpy.shutdown()
    return 0 if (rigid and ok3) else 2


if __name__ == '__main__':
    sys.exit(main())
