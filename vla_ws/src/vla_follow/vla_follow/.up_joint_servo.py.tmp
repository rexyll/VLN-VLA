#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""joint_servo.py —— 关节伺服: 把"想去的角度"变成"真的到那个角度"

结论先行 (P84 花了整整一轮才摸清, 推翻了三版中间结论)
=====================================================
底层用 Gazebo 官方 gz-sim-joint-position-controller-system, 它有**两种模式**:

  A) 力控模式(默认): 输出 JointForceCmd = PID(actual - cmd)
  B) 速度模式(<use_velocity_commands>true</use_velocity_commands>):
     输出 JointVelocityCmd = -error/dt, 只在 |error| > cmd_max*dt 时限速

本项目**必须用 B**。A 模式在这套组合(gz-sim 8 + DART + 1ms 步长)上有三个
互相独立的毛病, 每一个都足以让"手臂到不了位":

  1. 积分项符号是反的。源码里 error = 当前角 - 目标角(P 项带负号), 而 I 项
     把误差原样累积加上去 —— 它不消除偏差, 而是往偏差方向推, 稳定在
         偏置 = i_gain / p_gain
     实测 pan(p30,i6) 偏 +0.20rad、lift(p40,i8) 偏 +0.20rad, 与公式吻合。
     症状: "把 p 从 30 提到 240, 误差一点都不降"(P 想压, I 顶着)。
     处理: i_gain 一律置 0。

  2. 即使关掉积分, 仍有 0.73 的"增益缺失": 下发 -1.5rad 只走到 -1.09。
     把整个世界重力置零, 这个比例一点不变 —— 所以**不是重力下垂**, 是力控
     本身的稳态问题。
     症状: "整条臂软绵绵, 只转 pan, 其余关节会被惯性拖着漂 0.2~0.8rad"。

  3. 大量 12g 级 link(手指)配 N·m 级力矩, 角加速度 1e5 rad/s² 量级, 1ms
     步长下数值发散, 冲量会把桌上的水壶水杯掀飞。

B 模式把速度当**硬约束**交给 DART 求解器: 没有增益、没有积分、没有稳态
误差, 下发即到位(实测 14 个关节 3 秒内到小数点后 3 位, <0.1°), 也不抖。
代价是不体现电机力矩上限 —— 本场景机械臂没有碰撞体, 完全可以接受。

于是 JointServo 退化成"下发目标 + 等实测到位", 不再需要迭代修正。保留这个
类是为了: 统一"发布/限位/等数据/超时"这些琐碎逻辑; 手部 20 路关节共用同一
接口; 若将来换回力控, 只改 move() 内部, 上层(drink_task)不用动。

另一类历史坑(仍然有效, 已做防护)
================================
* **NaN 污染**: 新进程起来时 /drink/joint_state 还没到, `_read()` 返回 NaN;
  NaN 一旦进了指令通道就把整侧关节写死, 之后怎么下发都不动 —— 看上去像机械
  故障, 实际是第一帧没数据。move() 入口先 `_wait_data()`。
* **时长与滞后**: 力控时代底层响应有 1.2s 级的限速运动, 外层按 40ms 修正会
  形成"带纯滞后的积分环"而自激。速度模式下这一点不再存在, 但仍保留超时。
"""
import time

import numpy as np
from std_msgs.msg import Float64

from vla_follow import arm_kin as K

LIM_LO = np.array([j[3][0] for j in K.JOINTS])
LIM_HI = np.array([j[3][1] for j in K.JOINTS])


class JointServo(object):
    """一侧手臂(7 轴)或一只手的关节通道封装。

    bus: 可选。传了就把目标写进一个**共享的指令总线**(drink_task.CmdBus),
         由总线统一按 20Hz 下发; 不传就自己建 publisher、自己按 50Hz 发。
         为什么要这个开关: drink_task 必须是 39 路 `/drink/cmd/*` 的**唯一
         节拍源**(20Hz 连续下发)。如果 JointServo 另建一套 publisher, 双方
         会往同一条话题交替写不同值, gz 侧按到达顺序取最后一条 → 关节在两个
         目标之间来回跳。共享总线后只有一个人写, 时序干净。
    """

    def __init__(self, node, side, joint_names=None, tol_deg=1.0, rate_hz=50,
                 bus=None):
        self.node = node
        self.side = side
        self.names = joint_names or ['%s_%s_joint' % (side, j[0])
                                     for j in K.JOINTS]
        self.tol = np.radians(tol_deg)
        self.dt = 1.0 / rate_hz
        self.bus = bus
        if bus is None:
            self.pub = [node.create_publisher(Float64,
                                              '/drink/cmd/%s' % n, 10)
                        for n in self.names]
        else:
            self.pub = None
        self.cmd = np.zeros(len(self.names))
        self.tgt = np.zeros(len(self.names))
        self.mea = np.zeros(len(self.names))

    # ------------------------------------------------------------ 内部
    def _uses_sim(self):
        """本节点是否跑在仿真时钟上(use_sim_time)。取不到就当 False。"""
        try:
            return bool(self.node.get_parameter('use_sim_time').value)
        except Exception:
            return False

    def _sim_now(self):
        """当前时间(秒): 仿真时钟可用时给仿真时间, 否则退回墙上时间。

        为什么伺服超时必须用仿真时钟(P102 踩坑)
        ========================================
        `move()` 的 timeout 原本是墙上时间。VM 上同时跑 gz 服务端 + Nav2 +
        感知 + follower, 实测 RTF 只有 0.32 —— 10s 墙上时间只等于 3.2s 仿真
        时间, 而放回那一段要从倒水位把两只手一起下到桌面(单关节摆 1.3rad,
        速度模式 0.5rad/s 上下), 仿真里就要 4~5s。于是 `move` 先按墙上超时
        返回, 手臂停在中途: 实测 place_held 收尾残差 52.3mm, put_down_two 判
        "杯子还没到位(52.3mm > 3.0), 两只手都不松", 放回直接失败。
        关节是**在仿真时间里**转的, 超时也就该按仿真时间算; 墙上时间只留
        6 倍兜底, 防止 /clock 不走(仿真暂停)时死等。
        """
        if self._uses_sim():
            try:
                s = self.node.get_clock().now().nanoseconds * 1e-9
                if s > 1.0:
                    return s
            except Exception:
                pass
        return time.time()

    def _elapsed(self, t_wall0, t_now0, cap_scale=6.0):
        """返回 (仿真/逻辑秒, 是否已到墙上兜底)。"""
        el = time.time() - t_wall0
        if t_now0 is None:
            return el, el
        return max(0.0, self._sim_now() - t_now0), el

    def _read(self):
        js = self.node.js
        out = []
        for n in self.names:
            v = js.get(n)
            out.append(np.nan if v is None else v)
        return np.array(out, float)

    def _publish(self):
        if self.bus is not None:
            self.bus.set(self.names, self.cmd)
            return
        for p, v in zip(self.pub, self.cmd):
            m = Float64()
            m.data = float(v)
            p.publish(m)

    def _limits(self):
        if len(self.names) == len(K.JOINTS):
            return LIM_LO, LIM_HI
        # 手部关节: 与 SDF 的 ±1.7 rad 一致
        return (np.full(len(self.names), -1.7),
                np.full(len(self.names), 1.7))

    def _wait_data(self, timeout=6.0):
        """等 /drink/joint_state 第一次给出本侧全部关节角(防止 NaN 写死指令)。"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            v = self._read()
            if not np.isnan(v).any():
                self.mea = v
                return True
            self.node.spin_for(self.dt)
        return False

    # ------------------------------------------------------------ 对外
    def move(self, q_target, timeout=8.0, tol_deg=None, verbose=False):
        """下发目标并等实测到位。返回 (是否达成, 最终逐轴误差 rad)。

        `timeout` 的单位是**仿真秒**(见 `_sim_now` 的说明), 墙上时间放宽 6 倍
        兜底 —— RTF 0.32 的机器上 8s 仿真时间约等于 25s 墙上时间。
        """
        tol = self.tol if tol_deg is None else np.radians(tol_deg)
        self.tgt = np.asarray(q_target, float)
        lo, hi = self._limits()
        self.cmd = np.clip(self.tgt, lo, hi)
        if not self._wait_data():
            return False, np.full(len(self.names), np.nan)
        t0 = time.time()
        s0 = self._sim_now() if self._uses_sim() else None
        last = -1.0
        err = self.tgt - self.mea
        while True:
            self._publish()
            self.node.spin_for(self.dt)
            cur = self._read()
            if np.isnan(cur).any():
                if time.time() - t0 > timeout * 6.0:
                    return False, np.full(len(self.names), np.nan)
                continue
            self.mea = cur
            err = self.tgt - cur
            el, el_wall = self._elapsed(t0, s0)
            if np.max(np.abs(err)) < tol:
                return True, err
            if el > timeout or el_wall > timeout * 6.0:
                return False, err
            if verbose and el - last >= 0.5:
                last = el
                print('    t=%4.1fs err(°)=%s' % (
                    el, ' '.join('%+6.1f' % np.degrees(e) for e in err)))

    def hold(self, seconds):
        """保持当前指令一段时间(速度模式下=让关节维持该目标)

        `seconds` 同样是**仿真秒**: 张手/落桌的"停 0.3s"在 RTF 0.32 上只等于
        0.09s 仿真时间, 手其实还在滑 —— 量出来的残差是"还在动"而不是"没到位"。
        """
        t0 = time.time()
        s0 = self._sim_now() if self._uses_sim() else None
        while True:
            self._publish()
            self.node.spin_for(self.dt)
            el, el_wall = self._elapsed(t0, s0)
            if el >= seconds or el_wall > seconds * 6.0:
                break
        self.mea = self._read()

    def set_raw(self, cmd):
        """直接下发(用于抓握张合这类不需要精确角度的动作)"""
        self.cmd = np.asarray(cmd, float)
        self.hold(0.8)
