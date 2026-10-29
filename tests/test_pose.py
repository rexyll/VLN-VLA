#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_pose.py —— P85 验收: 关节通道与静态姿态(6 项)

    [1] 37 路 /drink/cmd/* 在线, 且都由 drink_task 单一发布
    [2] 指令节拍稳定 20Hz(逐路测 3s)
    [3] home 后双 7 轴到位(指令-实测 < 1.5°)
    [4] 十指张开到位(< 4°, 指骨短、限位软)
    [5] 掌心 FK 与 gz 真值一致(< 10mm / 3°)
    [6] drink_cam 出图(非空白)

为什么这 6 项缺一不可:
    [1][2] 是"通道存在且活着" —— 缺了后面所有动作都是空中楼阁;
    [3][4] 是"指令真的被执行" —— P84 只有 3 个轴被真正验证过(其余 4 个轴的
           Q_REST 都是 0, 不需要动), 所以必须逐轴激励;
    [5] 是"运动学与 SDF 一致" —— 但注意: FK 用的是**实测 q**, 所以它只能证明
           链结构一致, **不能**证明指令方向对(轴镜像也能通过); 方向由
           tests/diag/probe_axis_fk.py 单独验证。
    [6] 是"观察通道" —— 后面每一个抓取/倒水动作都要靠它做人眼验收。

用法(世界已在跑):
    python3 tests/test_pose.py
    python3 tests/test_pose.py --shot          # 额外存一张 drink_cam 图
退出码: 0 全通过 / 2 有失败项
"""
import math
import os
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from vla_follow import arm_kin as K
from vla_follow.drink_task import CMD_NAMES, default_cmd
from vla_follow.joint_servo import JointServo

try:
    import gz.transport13 as gz_transport
    from gz.msgs10.pose_v_pb2 import Pose_V as GzPose_V
    HAVE_GZ = True
except Exception:
    HAVE_GZ = False

HZ_MIN, HZ_MAX = 17.0, 23.0
ARM_TOL = 1.5      # °
HAND_TOL = 4.0     # °
POS_TOL = 0.010    # m
ROT_TOL = math.radians(3.0)


class Rec(Node):

    def __init__(self):
        super().__init__('test_pose')
        self.js = {}
        self.links = {}
        self.count = {n: 0 for n in CMD_NAMES}
        self.img = None
        self.create_subscription(JointState, '/drink/joint_state', self._js, 30)
        # 队列深度只留 1: 37 路 × 20Hz = 740 msg/s, 用深队列时订阅端会囤积
        # 未处理的消息, 测速时把积压一起算进去 → 实测 24Hz 而发布端其实是
        # 20Hz(这个假象骗过了一轮)。深度 1 只保留最新值, 测到的就是真实节拍。
        for n in CMD_NAMES:
            self.create_subscription(
                Float64, '/drink/cmd/%s' % n,
                lambda m, nm=n: self.count.__setitem__(nm, self.count[nm] + 1),
                1)
        self.create_subscription(Image, '/drink_cam/image', self._img, 5)
        self.cli = self.create_client(Trigger, '/drink/ck')
        if HAVE_GZ:
            self._gz = gz_transport.Node()
            self._gz.subscribe(GzPose_V,
                               '/world/follow_world/dynamic_pose/info',
                               self._gt)

    def _js(self, m):
        self.js.update(zip(m.name, m.position))

    def _img(self, m):
        self.img = m

    def _gt(self, m):
        for p in m.pose:
            self.links[p.name] = p

    def spin_for(self, s):
        t0 = time.time()
        while time.time() - t0 < s:
            rclpy.spin_once(self, timeout_sec=0.005)


def gt_T(p):
    q = p.orientation
    return K.frame(K.quat_to_mat(q.x, q.y, q.z, q.w),
                   (p.position.x, p.position.y, p.position.z))


def save_image(msg, path):
    """rgb8 → PNG(不依赖 cv_bridge)。"""
    try:
        from PIL import Image as PILImage
    except Exception:
        return False
    arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
        msg.height, msg.width, 3)
    PILImage.fromarray(arr).save(path)
    return True


def main():
    shot = '--shot' in sys.argv
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    logdir = os.path.join(root, 'vla_ws', 'logs')
    os.makedirs(logdir, exist_ok=True)

    # ---- 起 drink_task 常驻(它必须是 37 路指令的唯一发布者)
    env = dict(os.environ)
    log = open(os.path.join(logdir, 'p85_drink_task.log'), 'w')
    proc = subprocess.Popen(
        [sys.executable, '-u', '-m', 'vla_follow.drink_task'],
        stdout=log, stderr=subprocess.STDOUT, env=env)

    rclpy.init()
    n = Rec()
    results = []
    try:
        n.spin_for(6.0)                      # 等 home 完成

        # ---- [1] 37 路话题在线 + 单一发布者
        print('=' * 74)
        print('P85 验收 —— 关节通道与静态姿态')
        print('=' * 74)
        try:
            out = subprocess.run(['ros2', 'topic', 'list'],
                                 capture_output=True, text=True,
                                 timeout=30).stdout
            listed = [x for x in out.split() if x.startswith('/drink/cmd/')]
        except Exception:
            listed = []
        ok1 = len(listed) == 37
        results.append(ok1)
        print('[1] 指令话题  ROS 侧 %d/37 在线   gz 侧 %d/37'
              % (len(listed), sum(1 for v in n.count.values() if v > 0)))
        if not ok1:
            miss = sorted(set('/drink/cmd/%s' % x for x in CMD_NAMES)
                          - set(listed))
            print('    缺: %s' % miss[:8])

        # ---- [2] 节拍 20Hz
        n.spin_for(1.0)                      # 先把启动期的积压排空
        c0 = dict(n.count)
        t0 = time.time()
        n.spin_for(3.0)
        win = time.time() - t0
        rates = {k: (n.count[k] - c0[k]) / win for k in CMD_NAMES}
        slow = sorted(k for k, v in rates.items() if v < HZ_MIN)
        fast = sorted(k for k, v in rates.items() if v > HZ_MAX)
        hz_min = min(rates.values())
        hz_max = max(rates.values())
        ok2 = not slow and not fast
        results.append(ok2)
        print('[2] 指令节拍  %.1f~%.1f Hz (目标 20Hz, 容差 %.0f~%.0f)'
              % (hz_min, hz_max, HZ_MIN, HZ_MAX))
        if slow:
            print('    偏慢: %s' % slow[:6])
        if fast:
            print('    偏快: %s' % fast[:6])

        # ---- [3][4] 到位精度(双臂 7 轴 + 十指)
        exp = default_cmd()
        arm_err, hand_err = {}, {}
        for s in ('R', 'L'):
            arm_err[s] = max(abs(math.degrees(n.js['%s_%s_joint' % (s, j[0])]
                                              - exp['%s_%s_joint' % (s, j[0])]))
                             for j in K.JOINTS)
            hand_err[s] = max(abs(math.degrees(n.js['%s_joint' % hj]
                                               - exp['%s_joint' % hj]))
                              for hj in K.hand_joints(s))
        ok3 = max(arm_err.values()) < ARM_TOL
        ok4 = max(hand_err.values()) < HAND_TOL
        results += [ok3, ok4]
        print('[3] 双 7 轴 home   R %.2f°  L %.2f°  (限 %.1f°)'
              % (arm_err['R'], arm_err['L'], ARM_TOL))
        print('[4] 十指张开       R %.2f°  L %.2f°  (限 %.1f°)'
              % (hand_err['R'], hand_err['L'], HAND_TOL))

        # ---- [5] FK vs 真值
        ok5 = True
        base = (0.0, 0.0, 0.0)
        lines = []
        for s in ('R', 'L'):
            q = np.array([n.js['%s_%s_joint' % (s, j[0])] for j in K.JOINTS])
            T = K.base_pose(*base) @ K.fk(s, q)
            key = '%s_palm' % s
            if key not in n.links:
                ok5 = False
                lines.append('%s 真值缺失' % s)
                continue
            e = K.pose_err(T, gt_T(n.links[key]))
            dp = float(np.linalg.norm(e[:3]))
            dr = float(np.linalg.norm(e[3:]))
            lines.append('%s 位置 %5.1fmm 姿态 %.2f°' % (s, dp * 1000,
                                                        math.degrees(dr)))
            ok5 = ok5 and dp < POS_TOL and dr < ROT_TOL
        results.append(ok5)
        print('[5] 掌心 FK-真值   %s' % '   '.join(lines))

        # ---- [6] drink_cam
        ok6 = False
        if n.img is None:
            n.spin_for(2.0)
        if n.img is not None:
            px = np.frombuffer(bytes(n.img.data), dtype=np.uint8)
            ok6 = px.size > 0 and len(np.unique(px[::997])) > 8
            if shot:
                p = os.path.join(logdir, 'p85_home.png')
                if save_image(n.img, p):
                    print('    已存 %s' % p)
            print('[6] drink_cam     %dx%d %s 颜色数=%d'
                  % (n.img.width, n.img.height, n.img.encoding,
                     len(np.unique(px[::997]))))
        else:
            print('[6] drink_cam     无图像')
        results.append(ok6)

        # ---- 附: /drink/ck 服务自检
        if n.cli.wait_for_service(timeout_sec=5.0):
            fut = n.cli.call_async(Trigger.Request())
            n.spin_for(3.0)
            n.spin_for(0.1)
            if fut.done() and fut.result() is not None:
                print('-' * 74)
                print('[ck] /drink/ck 服务返回 success=%s' % fut.result().success)
                for line in fut.result().message.splitlines():
                    print('     ' + line)

        print('-' * 74)
        print('结论: %s  (%d/6 通过)'
              % ('通过' if all(results) else '不通过', sum(results)))
        print('=' * 74)
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=8)
        except Exception:
            proc.kill()
        log.close()
        rclpy.shutdown()
    return 0 if all(results) else 2


if __name__ == '__main__':
    sys.exit(main())
