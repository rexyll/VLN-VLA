#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""carry.py —— 抓取附着的 ROS 侧驱动 (P86)

它干的事: 抓握期间, 每 20ms 把"物体应该在哪"算出来发给世界内的
`GraspCarry` 插件, 由插件每 1ms 钉一次。物体就在手里**刚性跟随**。

为什么位姿在 ROS 侧算
=====================
第一版让 gz 插件自己读掌心位姿再乘偏移。实测偏差只取决于手臂构型:
摆 0.45 rad 后 28.9mm/0.76°, 摆回原位变 66.6mm/1.63°, 每摆一个来回累加。
三方对照(位姿话题 / 关节角正解 / 插件读数)证实: **正解与话题一致到 0.0mm,
插件从 ECM 读到的 link 位姿是旧的**。所以掌心位姿改由本模块用 `arm_kin.fk`
从**实测关节角**算 —— 这条链 P85 验收过 FK 与 gz 真值 0.0mm / 0.00°,
且运动学仍然只有 arm_kin.py 一份真源。

为什么用话题而不是服务
=====================
`/world/<w>/set_pose` 是唯一的"外部改位姿"入口, 但 gz-transport 的
Python 请求/应答会**累计退化**(实测约 400 次后成片超时并 core dump),
而每帧跟随要 50 次/秒。改成单向下发后: 插件每 tick 重发一次目标, 丢几条
消息也不影响, 与 39 路关节指令走同一条已被 P85 验证的链路。

指令格式(与 GraspCarry.cc 对应)
=============================
    grab <model> x y z qx qy qz qw    登记目标, 回 ack
    set  <model> x y z qx qy qz qw    更新目标(高频, 不回 ack)
    place <model> x y z qx qy qz qw   一次性摆位(测试布景用)
    joint <model> <joint> <value>     运动学驱动一个关节到 value, 回 ack
    release <model> | release all     取消, 回 ack
"""
import math
import time

import numpy as np
from std_msgs.msg import String

from vla_follow import arm_kin as K

CARRY_TOPIC = '/drink/carry'
ACK_TOPIC = CARRY_TOPIC + '/ack'
STREAM_HZ = 50.0            # 目标位姿下发频率


def quat_of(T):
    """4x4 → (qx, qy, qz, qw)。用 Shuster 法, 避开 trace 在 180° 附近的病态。"""
    R = np.asarray(T)[:3, :3]
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (x, y, z, w)


class CarryBus(object):
    """抓握附着的指令通道。由 drink_task 持有, 定时器驱动 update()。"""

    def __init__(self, node):
        self.node = node
        self.pub = node.create_publisher(String, CARRY_TOPIC, 10)
        node.create_subscription(String, ACK_TOPIC, self._on_ack, 50)
        self.ack = None
        self.acks = []              # P98: ack 队列(见 _on_ack)
        self.ack_t = 0.0
        # model -> {'side': 'R'/'L', 'offset': 4x4}
        self.items = {}
        self.pub_count = 0

    # ------------------------------------------------------------ 内部
    def _on_ack(self, msg):
        # P98: ack 改为**队列** —— 原来是单槽, 液面控制器以 ~7Hz 并发下发
        # water_joint 后, jet 的 ack 会在同一个 spin 窗口里被 water 的 ack
        # 覆盖, _await 永远等不到(实测 jet ramp 全程无 ack)。
        self.acks.append(msg.data)
        del self.acks[:-64]
        self.ack = msg.data
        self.ack_t = time.time()

    def _send(self, text):
        self.pub.publish(String(data=text))
        self.pub_count += 1

    def _send_pose(self, op, model, T):
        qx, qy, qz, qw = quat_of(T)
        t = T[:3, 3]
        self._send('%s %s %.6f %.6f %.6f %.6f %.6f %.6f %.6f'
                   % (op, model, t[0], t[1], t[2], qx, qy, qz, qw))

    def _await(self, emit, ok_pred, timeout):
        """重复发一条指令直到收到匹配的 ack。

        单向话题在 gz/ROS 发现完成前会静默丢消息(实测第一轮连发三次全丢),
        所以必须靠 ack 确认; ack 到了之后再补发几针, 保证 gz 侧也收到。
        注意 emit 和 ok_pred 是两个不同的东西 —— 第一版把它们写成一个函数,
        结果"检查"分支永远返回 None, 于是每次都空等到超时。
        """
        t0 = time.time()
        self.ack = None
        self.acks = []
        while time.time() - t0 < timeout:
            emit()
            self.node.spin_for(0.08)
            hit = next((a for a in reversed(self.acks) if ok_pred(a)), None)
            if hit:
                for _ in range(3):           # 补两针, 防止 gz 侧漏收
                    emit()
                    self.node.spin_for(0.03)
                self.node.get_logger().info('ack: %s' % hit)
                return True
        return False

    # ------------------------------------------------------------ 位姿来源
    def palm_world(self, side):
        """掌心世界位姿: 实测关节角 → arm_kin.fk → 底盘位姿。"""
        names = ['%s_%s_joint' % (side, j[0]) for j in K.JOINTS]
        js = self.node.js
        if any(n not in js for n in names):
            return None
        q = np.array([js[n] for n in names], float)
        Tb = K.base_pose(*self.node.base)
        return Tb @ K.fk(side, q)

    def obj_world(self, model):
        """物体世界位姿(来自 /world/<w>/dynamic_pose/info, 只在 grab 时用一次)。"""
        p = self.node.links.get(model)
        if p is None:
            return None
        q = p.orientation
        return K.frame(K.quat_to_mat(q.x, q.y, q.z, q.w),
                       (p.position.x, p.position.y, p.position.z))

    def host_world(self, host, link=None, local_off=None):
        """"持有者"的世界位姿。host 可以是模型名, 也可以再指定一个 link 和 link 内偏移。

        踩坑提醒(P86 #22): 这个话题里**模型是世界位姿, link 是相对所属模型
        的位姿**, 所以 link 必须先乘模型位姿才是世界位姿。人的手臂
        (`person_arms`) 就是这种: 它随 `person_arm_joint` 转。

        踩坑提醒(P89): `person_arms` 的**原点在肩关节**, 而我们要钉的是
        "两手之间"那个点 —— 少了 `local_off` 这一步, 交接时人手明明在杯子
        旁边, 杯子却被钉到肩膀上(1.9m 外)。
        """
        Tm = self.obj_world(host)
        if Tm is None:
            return None
        if link is None:
            return Tm
        p = self.node.links.get(link)
        if p is None:
            return None
        q = p.orientation
        Tl = K.frame(K.quat_to_mat(q.x, q.y, q.z, q.w),
                     (p.position.x, p.position.y, p.position.z))
        T = Tm @ Tl
        if local_off is not None:
            T = T @ K.trans(local_off)
        return T

    # ------------------------------------------------------------ 对外
    def place(self, model, T, timeout=3.0):
        """一次性把 model 摆到 T(测试布景用, 不持续钉住)。"""
        ok = self._await(
            lambda: self._send_pose('place', model, T),
            lambda a: a.startswith('ok place'), timeout)
        if ok:
            self.node.get_logger().info('place %s 完成' % model)
        else:
            self.node.get_logger().error('place %s 无 ack' % model)
        return ok

    def place_fast(self, model, T):
        """高频摆位(不回 ack): 液面盘/水柱段这类逐帧驱动的视觉体 (P98)。

        与 place() 用同一个 `place` 指令 —— 插件里它本来就是"下一帧
        SetWorldPoseCmd"(place 只钉一帧), 高频重发即连续 teleport;
        等 ack 反而把 50Hz 拖成 RTT 节奏。
        """
        self._send_pose('place', model, T)
        self.pub_count += 1

    def grab(self, model, side, timeout=4.0):
        """把 model 钉到 side 侧手掌上。返回是否成功。"""
        Tp = self.palm_world(side)
        To = self.obj_world(model)
        if Tp is None or To is None:
            self.node.get_logger().error(
                'grab %s 失败: 掌心=%s 物体=%s'
                % (model, Tp is not None, To is not None))
            return False
        off = np.linalg.inv(Tp) @ To
        # 先登记(带 offset), 之后 update() 只需用当前掌心位姿乘这个常量
        self.items[model] = {'side': side, 'offset': off}

        state = {'first': True}

        def emit():
            Tp_now = self.palm_world(side)
            if Tp_now is None:
                Tp_now = Tp
            # 踩坑记录(P88): 这里原来只在第一针发 `grab`, 之后改发 `set`。
            # 但插件**只对 grab 回 ack**(set 是高频更新, 有意不回), 于是只要
            # 第一针在发现完成前丢掉, 后面发多少条 `set` 都永远等不到 ack ——
            # 表现为"grab cup 无 ack", 而且换个时刻重跑就好了, 极难复现。
            # grab 是幂等的(重新登记同一目标), 所以整段都发 grab。
            self._send_pose('grab', model, Tp_now @ off)
            state['first'] = False

        ok = self._await(emit, lambda a: a.startswith('ok grab'), timeout)
        if not ok:
            self.items.pop(model, None)
            self.node.get_logger().error('grab %s 无 ack' % model)
            return False
        d = float(np.linalg.norm(off[:3, 3]))
        self.node.get_logger().info(
            'grab %s -> %s_palm  偏移 %.3f m%s'
            % (model, side, d,
               '  [警告] 偏移过大, 抓取时掌心离物体很远' if d > 0.25 else ''))
        return True

    def joint(self, model, joint, value, timeout=2.5):
        """把一个关节**运动学**驱动到 value(米/弧度)。

        为什么不用 `/drink/cmd/<joint>` 那条速度通道
        ============================================
        P87 踩坑: jet / water 是速度模式的 `JointPositionController`, 但它们
        所属的模型(kettle / cup)会被 GraspCarry 每 1ms teleport 一次; teleport
        会把刚写进去的速度**清零**, 于是同一条命令表现为时而全动、时而完全
        不动、时而只走一部分 —— 实测 jet 只到 0.124/0.16, water 恒 0/0.012。
        这不是参数问题, 是**竞态**, 调 PID 永远调不出来。

        改成由插件在同一个 PreUpdate 里写 `JointPositionReset`, 与 teleport
        同源, 就不再跟物理抢速度了。位置是绝对量, 丢一两帧也无所谓。
        """
        # P98: 判据必须精确到关节名 —— 液面控制器会以 ~10Hz 持续下发
        # water_joint, 泛化的 startswith('ok joint') 会拿 water 的 ack 冒充
        # jet 的 ack(反之亦然)。
        ok = self._await(
            lambda: self._send('joint %s %s %.6f' % (model, joint, value)),
            lambda a: a.startswith('ok joint %s' % joint), timeout)
        if not ok:
            self.node.get_logger().warn('joint %s=%s 无 ack' % (joint, value))
        return ok

    def joint_ramp(self, model, joint, target, dur=1.0, dt=0.05):
        """分步把关节推到 target(位置指令是跳变的, 直接给终点会"瞬移")。"""
        cur = float(self.node.js.get(joint, 0.0))
        n = max(1, int(dur / max(dt, 1e-3)))
        for i in range(1, n + 1):
            v = cur + (target - cur) * float(i) / n
            if not self.joint(model, joint, v, timeout=0.8):
                return False
            self.node.spin_for(dt)
        return True

    def release(self, model=None, timeout=2.5):
        name = model or 'all'
        ok = self._await(
            lambda: self._send('release %s' % name),
            lambda a: a.startswith('ok release'), timeout)
        if model is None:
            self.items.clear()
        else:
            self.items.pop(model, None)
        if ok:
            self.node.get_logger().info('release %s' % name)
        else:
            self.node.get_logger().warn('release %s 无 ack(已按松开处理)' % name)
        return ok

    def update(self):
        """按 STREAM_HZ 调用: 把每个已抓物体的目标位姿刷一遍。

        两种持有者:
          * 自己(机械臂): 掌心位姿 = FK(实测关节角)，乘冻结偏移；
          * 别人(比如人的手): 位姿 = host 世界位姿 × 冻结偏移 —— 交接之后
            物体就跟着对方的手走，我们这边只管继续钉住。
        """
        for model, it in self.items.items():
            if it.get('host'):
                Tb = self.host_world(it['host'], it.get('link'),
                                     it.get('local_off'))
                if Tb is None:
                    continue
                self._send_pose('set', model, Tb @ it['offset'])
                continue
            Tp = self.palm_world(it['side'])
            if Tp is None:
                continue
            self._send_pose('set', model, Tp @ it['offset'])

    def hand_to(self, model, host, link=None, local_off=None):
        """"交给"对方(比如人的手): 换持有者, 但**不松钉**。

        为什么不是 release + 再 grab
        ==========================
        松钉会让物体在整个交接期间变成自由的（实测松钉瞬间会弹 2~5mm）。
        而插件本来只认"目标世界位姿", 所以只要把位姿来源从我们的掌心换成
        对方的手, 物体就无缝跟着对方走了。

        local_off 是"在 host link 坐标系里的偏置" —— 人的 `person_arms`
        原点在**肩关节**, 而我们要的是两手之间那个点, 所以必须给
        `scene_layout.PERSON_HAND_IN_ARM`。
        """
        T_host = self.host_world(host, link, local_off)
        To = self.obj_world(model)
        if T_host is None or To is None:
            self.node.get_logger().error(
                'hand_to %s <- %s 失败: host=%s obj=%s'
                % (model, host, T_host is not None, To is not None))
            return False
        self.items[model] = {'side': None, 'host': host, 'link': link,
                             'local_off': local_off,
                             'offset': np.linalg.inv(T_host) @ To}
        self.node.get_logger().info(
            'hand_to: %s 交给 %s%s' % (model, host,
                                       ('' if link is None else '/' + link)))
        return True

    def holding(self):
        return list(self.items.keys())
