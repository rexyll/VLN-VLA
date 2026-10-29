#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gen_drink_scene.py —— 生成"喝水任务"的 SDF / bridge / 参数片段 (P84)

只改**夹逼标记之间**的内容, 幂等可重复执行:
    work_area.sdf   <!-- DRINK:BEGIN robot_ext -->  ... <!-- DRINK:END robot_ext -->
                    <!-- DRINK:BEGIN person_ext --> ... <!-- DRINK:END person_ext -->
                    <!-- DRINK:BEGIN props -->      ... <!-- DRINK:END props -->
    config/bridge.yml <!-- DRINK:BEGIN bridge -->  ... <!-- DRINK:END bridge -->

几何全部取自 vla_follow/scene_layout.py + arm_kin.py(唯一真源),
因此"控制器算的"和"SDF 里摆的"不可能走偏。

用法:
    python3 tools/gen_drink_scene.py            # 生成
    python3 tools/gen_drink_scene.py check      # 只校验标记是否齐备
"""
import json
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'vla_ws', 'src', 'vla_follow'))

from vla_follow import arm_kin as K          # noqa: E402
from vla_follow import scene_layout as L     # noqa: E402
from vla_follow import drink_poses as P      # noqa: E402
from vla_follow import liquid                # noqa: E402  (P98 液面模型)

WORLD = os.path.join(ROOT, 'vla_ws', 'src', 'vla_follow', 'worlds',
                     'work_area.sdf')
BRIDGE = os.path.join(ROOT, 'vla_ws', 'src', 'vla_follow', 'config',
                      'bridge.yml')
CONFIG = os.path.join(ROOT, 'vla_demo', 'drink_config.json')

# 关节位置控制器(力控刚度)。手臂 link 都是 <gravity>false</gravity>, 没有重力
# 负载, 所以增益只由惯量决定 —— 而 7 个关节的惯量差了近 20 倍(肩 0.15 → 腕
# 0.005 kg·m²), 用同一组 p/d 必然让轻的那端自激振荡。
# P84 踩过的坑: 统一 p=60/d=3.2 时 lift/elbow/wrist_p 稳定停在离目标 6~17° 处,
# 把 p 提到 240 误差一点不降; 连续采样同一目标三次读到 -0.178/-0.489/-0.079
# 才发现是等幅振荡, 采样恰好落在不同相位, 伪装成"稳态误差"。
# 结论: 按惯量分档给 p (肩 30 → 腕 6), d 取 ζ≈0.8, 另加 <axis><dynamics>
# <damping> 做粘性阻尼(静止时不产生力矩 → 不引入稳态误差, 只吃高频抖动)。
# cmd_max 是**力矩**上限。原来给 40N·m, 但整条臂 2.9kg、力臂 0.5m 时重力
# 力矩就有 14N·m, 再叠加惯量与积分, 40N·m 会饱和 —— 表现是"正转跟得上、
# 反转顶不动"(重力方向与反转指令相反)。仿真里没有真实电机限制, 放到 150。
ARM_VMAX = 2.5      # 速度模式下的最大关节速度 (rad/s)
# 还必须给 i_gain: **不能给**。这一条推翻了我上一版写的结论, 留档说明原因:
# 上一版看到"稳态误差 ≈ 重力力矩/p"就认定是"纯 P 顶着重力必然留偏差", 于是
# 加了 i_gain 想把它压到 0 —— 结果误差反而更大, 而且**不随 p 缩小**(p 从 30
# 提到 240, 误差几乎不动), 这条现象当时解释不了。
# 现在对着 gz-sim 8 的源码看清楚了: JointPositionController 里
#     error = 当前关节位置 - 目标位置        // 注意是 actual - cmd
#     force = posPid.Update(error, dt)      // PID 内部 P 项带负号, I 项不带
# 于是积分项的符号是**反的**: 它不消除偏差, 而是持续往偏差方向推, 直到
#     p * 偏置 == i_gain * (积分钳位上限 1.0)
# 也就是偏置 = i_gain / p_gain。实测 pan(p30,i6) 偏 +0.20rad、lift(p40,i8)
# 偏 +0.20rad、elbow(p30,i6) 偏 +0.20rad —— 与公式吻合。
# 结论: i_gain 一律置 0, 偏差交给外层 joint_servo 的输入迭代吃掉。
ARM_GAINS = {                 # 关节 -> (p_gain, d_gain, 轴粘性阻尼, i_gain)
    # lift / elbow 是仅有的两个"要顶住整条臂自重"的轴(俯仰轴), 纯 P 会按
    # τ_重力/p 下垂: 实测 lift 在 p=40 时下垂 0.54rad(31°), elbow p=30 时
    # 下垂 0.63rad。虽然外层 joint_servo 能补, 但下垂会顺带削弱刚度, 让
    # 整条臂在运动中被惯性拖着晃(只转 pan, fore_roll 就会漂 0.23rad)。
    # 所以这两个轴单独把 p 提一个量级, 把下垂压到 4° 以内。
    # 稳定性: ωn=sqrt(p/I), lift 的 I≈0.15 → p=200 时 36rad/s, 1ms 步长下
    # 每周期 170 个采样点, 余量充足; d 取 ζ≈0.9。
    'pan':       (30.0, 1.6, 1.2, 0.0),
    'lift':      (200.0, 8.0, 1.2, 0.0),
    'up_roll':   (20.0, 1.2, 1.0, 0.0),
    'elbow':     (150.0, 6.5, 1.0, 0.0),
    'fore_roll': (10.0, 0.8, 0.8, 0.0),
    'wrist_p':   (8.0, 0.7, 0.7, 0.0),
    'wrist_r':   (6.0, 0.5, 0.6, 0.0),
}
ARM_P_GAIN, ARM_D_GAIN, ARM_I_GAIN = 30.0, 1.4, 0.0   # 兜底
# 手指 link 只有 12g, 真实惯量 ~2e-6 kg·m²。配上 N·m 级力矩 = 角加速度
# 5e5 rad/s², 1ms 步长下求解器必然发散 —— 表现是"手指疯狂振动, 冲量把桌上
# 的水壶水杯掀飞出 4m", 并连带整条臂都跟不住目标。这里把惯量放大到
# 2e-4(数值上稳), 同时把手指力矩降到 0.6N·m 量级, 等效动态行为基本不变。
FING_P_GAIN, FING_D_GAIN, FING_VMAX = 0.6, 0.03, 4.0

MAT_ARM = ('0.78 0.80 0.83', '0.78 0.80 0.83')      # 亮灰白
MAT_ARM2 = ('0.42 0.45 0.50', '0.42 0.45 0.50')     # 深灰关节
MAT_HAND = ('0.14 0.15 0.17', '0.14 0.15 0.17')     # 近黑(灵巧手)

# ---------------------------------------------------------------- 实验开关
# P84 定位"手臂一动, 桌上的壶/杯和人都跟着动"这类耦合现象需要能一键关掉某类
# 子系统。用环境变量而不是命令行参数, 因为 main() 的 argv[1] 已经被 check 占用。
#   DRINK_NO_FINGERS=1 不生成 20 个手指链节
#   DRINK_NO_FLUID=1   不生成 jet / water 两个滑动副
FLAGS = set(k for k in ('FINGERS', 'FLUID')
            if os.environ.get('DRINK_NO_' + k))

# ---------------------------------------------------------------- 抓取附着
# DetachableJoint **默认不生成**, 要显式 DRINK_DETACH=1 才加。
#
# 原因(P85 用 A/B 对照证实, 代价: 排查一整轮):
#   4 个 gz-sim-detachable-joint-system 插件(R_kettle / L_cup / R_cup /
#   person_cup)的 child_model 指向**世界级模型** kettle / cup, 而该插件只在
#   "父模型的**嵌套子模型**"里找 child_model, 于是启动时就报
#       [Wrn] [DetachableJoint.cc:308] Child Model kettle could not be found.
#   更严重的是: 这四个插件即使从未收到 attach 消息, 也会**破坏机械臂关节的
#   约束求解** —— 现象是"7 个轴里只有 lift/elbow/wrist_p 听话, pan/up_roll/
#   fore_roll/wrist_r 时不时卡在某个值不动, 而且左右两侧表现不一样"(R 侧多
#   一个 R_cup 插件, 症状更重)。指令下发 0.6 只走到 0.152 或干脆 0.000, 速度
#   反馈恒为 0。
#   去掉这 4 个插件后, 同样的指令 14 个关节全部精确到位到小数点后 3 位。
#
# 结论: 抓取附着不能用世界级 child_model。P86 之前要改成
#   ①把壶/杯作为**嵌套模型**放进 robot 模型(会跟着车走, 语义不对), 或
#   ②drink_task 用 gz.transport13 在运行时手工建固定关节, 或
#   ③拿位姿跟随(set_pose)假附着。
# 方案定稿前, 生成器不产出这些插件, 保证手臂控制是干净的。
USE_DETACH = bool(os.environ.get('DRINK_DETACH'))


# ---------------------------------------------------------------- 工具
def f(x, n=5):
    return ('%.' + str(n) + 'f') % float(x)


def pose_str(t, rpy=(0, 0, 0)):
    return '%s %s %s %s %s %s' % (f(t[0]), f(t[1]), f(t[2]),
                                  f(rpy[0], 4), f(rpy[1], 4), f(rpy[2], 4))


def mat2rpy(R):
    """3x3 → SDF rpy (R = Rz(yaw)Ry(pitch)Rx(roll))"""
    pitch = math.asin(max(-1.0, min(1.0, -R[2][0])))
    if abs(math.cos(pitch)) < 1e-6:
        yaw = 0.0
        roll = math.atan2(-R[0][1], R[1][1])
    else:
        yaw = math.atan2(R[1][0], R[0][0])
        roll = math.atan2(R[2][1], R[2][2])
    return roll, pitch, yaw


def align_z(d):
    """把 +z 转到方向 d 的 rpy(roll=0)"""
    n = math.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2)
    dz = max(-1.0, min(1.0, d[2] / n))
    pitch = math.acos(dz)
    yaw = math.atan2(d[1], d[0]) if abs(d[0]) + abs(d[1]) > 1e-9 else 0.0
    return 0.0, pitch, yaw


def inertial(mass, ixx, iyy, izz, com=(0, 0, 0), rot=(0, 0, 0)):
    return f"""        <inertial>
          <pose>{pose_str(com, rot)}</pose>
          <mass>{mass}</mass>
          <inertia><ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>"""


def visual(name, geom, mat, t=(0, 0, 0), rpy=(0, 0, 0), alpha=None):
    extra = ''
    if alpha is not None:
        extra = '\n          <transparency>%s</transparency>' % f(alpha, 3)
    return f"""        <visual name="{name}">
          <pose>{pose_str(t, rpy)}</pose>
          <geometry>{geom}</geometry>
          <material>
            <ambient>{mat[0]} 1</ambient>
            <diffuse>{mat[1]} 1</diffuse>
          </material>{extra}
        </visual>"""


def collision(name, geom, t=(0, 0, 0), rpy=(0, 0, 0), mu=0.9, mask=None):
    m = ('' if mask is None
         else '\n          <collide_bitmask>%s</collide_bitmask>' % mask)
    return f"""        <collision name="{name}">
          <pose>{pose_str(t, rpy)}</pose>
          <geometry>{geom}</geometry>{m}
          <surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction>
          </surface>
        </collision>"""


def g_cyl(r, ln):
    return '<cylinder><radius>%s</radius><length>%s</length></cylinder>' % (
        f(r, 4), f(ln, 4))


def g_sph(r):
    return '<sphere><radius>%s</radius></sphere>' % f(r, 4)


def g_box(a, b, c):
    return '<box><size>%s %s %s</size></box>' % (f(a, 4), f(b, 4), f(c, 4))


def joint(name, jtype, parent, child, t=(0, 0, 0), axis=None,
          limits=None, damping=0.02):
    s = ['      <joint name="%s" type="%s">' % (name, jtype)]
    s.append('        <parent>%s</parent>' % parent)
    s.append('        <child>%s</child>' % child)
    # P84 关键: 关节偏置**不能**写在 <joint><pose> 里。
    # 实测 gz-sim 8 建模型时用的是"子 link 自己相对父 link 的 pose"来摆放
    # link, <joint><pose> 只被当成约束锚点, 无论写什么都不影响布局 ——
    # 于是 7 个连杆全部叠在模型原点, 画面上只有一根灰柱子(排查了两轮:
    # 真值里 R_L0..R_palm 全读成 (0,0,0))。本项目原有的轮子一直好好的,
    # 正是因为它们写的是 <link><pose relative_to="chassis">。
    # 所以这里统一改成: joint 不写 pose, 偏置交给 _lpose() 挂到 link 上。
    if axis is not None:
        s.append('        <axis><xyz>%s %s %s</xyz>' % (
            f(axis[0], 4), f(axis[1], 4), f(axis[2], 4)))
        if limits:
            # effort 是关节**力矩硬限**, 它会盖掉 JPC 的 <cmd_max>。
            # 之前给 40N·m, 结果"正转跟得上、反转顶不动" —— 重力矩一旦超过
            # 40 关节就被限死, 提高 p / i / cmd_max 全都不起作用(排查绕了远路)。
            # 仿真里没有真实电机约束, 放到 400。
            s.append('          <limit><lower>%s</lower><upper>%s</upper>'
                     '<effort>400</effort><velocity>8</velocity></limit>'
                     % (f(limits[0], 4), f(limits[1], 4)))
        # P84: <dynamics> 必须挂在 <axis> 下。挂在 <joint> 下 SDFormat 不认,
        # gz sdf -k 会报 "XML Element[dynamics] ... not defined in SDF" 并静默
        # 丢弃 —— 手指这类 gravity=false 的轻 link 会因此完全没有阻尼而抖动。
        if damping:
            s.append('          <dynamics><damping>%s</damping></dynamics>'
                     % f(damping, 3))
        s.append('        </axis>')
    s.append('      </joint>')
    return '\n'.join(s)


def jpc_plugin(jname, p, d, cmax, i=0.0):
    # P84 关键: <plugin> 的 name 必须是**全限定类名**, 不是实例名。
    # 写成 name="jpc_R_pan_joint" 时 SystemLoader 报
    #   "library does not contain requested plugin"
    # 并静默跳过 —— 表面上 37 路话题都在, 实际一个关节都不动。
    #
    # P84 关键之二: 用**速度指令模式**(use_velocity_commands=true)。
    # 力控模式(PID 输出 JointForceCmd)在本机这套组合上有两个治不好的毛病:
    #   1) 误差定义是 actual-cmd, 而积分项符号与之相反 → 稳定偏置
    #      i_gain/p_gain(实测 +0.2~0.35rad, 且不随 p 缩小);
    #   2) 关掉积分后仍有 0.73 的"增益缺失"(下发 -1.5 只到 -1.09),
    #      而且把世界重力置零也不变 —— 不是重力下垂, 是力控本身的稳态问题。
    # 速度模式走的是 JointVelocityCmd, 在 DART 里是**硬约束**, 不经过任何
    # 增益: 插件按 targetVel = -error/dt 下发, 只在 |error| 超过 cmd_max*dt
    # 时才限速。于是"下发即到位", 没有偏置、没有振荡, 也不需要外层伺服。
    # 代价是不体现电机力矩上限 —— 本场景机械臂没有碰撞体, 可以接受。
    return f"""      <plugin filename="gz-sim-joint-position-controller-system"
              name="gz::sim::systems::JointPositionController">
        <joint_name>{jname}</joint_name>
        <topic>/drink/cmd/{jname}</topic>
        <use_velocity_commands>true</use_velocity_commands>
        <p_gain>{p}</p_gain>
        <i_gain>{i}</i_gain>
        <d_gain>{d}</d_gain>
        <cmd_max>{cmax}</cmd_max>
        <cmd_min>-{cmax}</cmd_min>
      </plugin>"""


# ---------------------------------------------------------------- 机器人
def _lpose(parent, t):
    """link 的初始位姿: 相对父 link(与原有 left_wheel 的写法一致)"""
    if parent is None:
        return ''
    return '        <pose relative_to="%s">%s</pose>\n' % (parent, pose_str(t))


def gen_arm(side):
    """一侧 7 轴臂 + 手掌 + 10 指节"""
    out = []
    links = []
    joints = []
    plug = []
    prev = 'torso'
    for i, (jn, off, axis, lim) in enumerate(K.JOINTS):
        name = '%s_%s' % (side, jn)
        child = '%s_L%d' % (side, i)
        # P84 修复: 肩关节的偏置在 L.SHOULDER_* 里(不在 K.JOINTS 的 off 中),
        # 原写法把 t 算出来却仍用 off 传给 joint(), 于是 pan 关节 <pose> 写成
        # 0 0 0 —— 整条臂塌在躯干原点, 7 个连杆全在同一位置。真值里
        # R_L0..R_L6 全部读成 (0,0,0) 就是这么来的。
        t = off
        if i == 0:
            t = (L.SHOULDER_X, L.SHOULDER_Y if side == 'L' else -L.SHOULDER_Y,
                 L.SHOULDER_Z)
            kind = 'shoulder'
        elif i == 2:                      # 肘点
            kind = 'elbow'
        elif i == 4:                      # 腕点
            kind = 'wrist'
        elif i == 1:
            kind = 'upper'
        elif i == 3:
            kind = 'fore'
        elif i == 5:
            kind = 'wrist2'
        else:
            kind = 'wrist3'
        links.append(_arm_link(child, kind, side, parent=prev, t=t))
        _p, _d, _damp, _i = ARM_GAINS.get(
            jn, (ARM_P_GAIN, ARM_D_GAIN, 1.0, ARM_I_GAIN))
        joints.append(joint('%s_joint' % name, 'revolute', prev, child,
                            t=t, axis=axis, limits=lim, damping=_damp))
        plug.append(jpc_plugin('%s_joint' % name, _p, _d, ARM_VMAX, _i))
        prev = child

    palm = '%s_palm' % side
    links.append(_palm_link(palm, side, parent=prev, t=(0, 0, -K.PALM_OFF)))
    joints.append(joint('%s_palm_joint' % side, 'fixed', prev, palm,
                        t=(0, 0, -K.PALM_OFF)))
    # 手指
    for k, fn in enumerate(K.FINGER_NAMES):
        base = '%s_hand_%s' % (side, fn)
        j1 = '%s_1_joint' % base
        j2 = '%s_2_joint' % base
        links.append(_finger_link(base + '_1', K.PROX_LEN, K.FINGER_W,
                                  parent=palm,
                                  t=(K.FINGER_X[k], 0, K.FINGER_BASE_Z)))
        links.append(_finger_link(base + '_2', K.DIST_LEN, K.FINGER_W * 0.86,
                                  parent=base + '_1', t=(0, 0, -K.PROX_LEN)))
        joints.append(joint(j1, 'revolute', palm, base + '_1',
                            t=(K.FINGER_X[k], 0, K.FINGER_BASE_Z),
                            axis=(1, 0, 0), limits=(-0.15, 1.65), damping=0.004))
        joints.append(joint(j2, 'revolute', base + '_1', base + '_2',
                            t=(0, 0, -K.PROX_LEN), axis=(1, 0, 0),
                            limits=(-0.15, 1.70), damping=0.003))
        plug.append(jpc_plugin(j1, FING_P_GAIN, FING_D_GAIN, FING_VMAX))
        plug.append(jpc_plugin(j2, FING_P_GAIN, FING_D_GAIN, FING_VMAX))
    # 拇指(左右镜像: 轴的 z 分量取反)
    tb = '%s_hand_thumb' % side
    j1, j2 = tb + '_1_joint', tb + '_2_joint'
    tx = K.THUMB_X[side]
    zs = 1.0 if side == 'R' else -1.0
    links.append(_finger_link(tb + '_1', 0.030, 0.017, thumb=True, parent=palm,
                             t=(tx, K.THUMB_BASE[1], K.THUMB_BASE[2])))
    links.append(_finger_link(tb + '_2', 0.024, 0.014, thumb=True,
                             parent=tb + '_1', t=(0, 0, -0.030)))
    joints.append(joint(j1, 'revolute', palm, tb + '_1',
                        t=(tx, K.THUMB_BASE[1], K.THUMB_BASE[2]),
                        axis=(0, 0, zs), limits=(-0.10, 1.10), damping=0.004))
    joints.append(joint(j2, 'revolute', tb + '_1', tb + '_2',
                        t=(0, 0, -0.030), axis=(1, 0, 0),
                        limits=(-0.10, 1.60), damping=0.003))
    plug.append(jpc_plugin(j1, FING_P_GAIN, FING_D_GAIN, FING_VMAX))
    plug.append(jpc_plugin(j2, FING_P_GAIN, FING_D_GAIN, FING_VMAX))
    out.extend(links)
    out.extend(joints)
    out.extend(plug)
    return '\n'.join(out)


def _arm_link(name, kind, side, parent=None, t=(0, 0, 0)):
    """手臂各段的视觉(无碰撞: 见 TaskPlan §2.6 —— 位形已离线校验,
    碰撞体只会带来"手臂被桌子顶住"这类坏情况与额外求解开销)"""
    if kind == 'shoulder':
        body = visual('v', g_sph(0.050), MAT_ARM2)
        mass, ii = 0.45, 0.0011
    elif kind == 'upper':
        body = visual('v', g_cyl(0.042, K.L1 - 0.03), MAT_ARM,
                      t=(0, 0, -(K.L1 - 0.03) / 2 - 0.015)) + '\n' + \
            visual('v2', g_sph(0.042), MAT_ARM2, t=(0, 0, -0.015))
        mass, ii = 0.60, 0.0062
    elif kind == 'elbow':
        body = visual('v', g_sph(0.042), MAT_ARM2)
        mass, ii = 0.35, 0.0007
    elif kind == 'fore':
        body = visual('v', g_cyl(0.034, K.L2 - 0.04), MAT_ARM,
                      t=(0, 0, -(K.L2 - 0.04) / 2)) + '\n' + \
            visual('v2', g_cyl(0.030, 0.05), MAT_ARM2,
                   t=(0, 0, -(K.L2 - 0.04)))
        mass, ii = 0.45, 0.0034
    elif kind == 'wrist':
        body = visual('v', g_cyl(0.028, 0.05), MAT_ARM2, t=(0, 0, -0.025))
        mass, ii = 0.25, 0.0003
    elif kind == 'wrist2':
        body = visual('v', g_sph(0.030), MAT_ARM2)
        mass, ii = 0.20, 0.0002
    else:
        body = visual('v', g_cyl(0.030, 0.03), MAT_ARM2, t=(0, 0, -0.015))
        mass, ii = 0.18, 0.0002
    return f"""      <link name="{name}">
{_lpose(parent, t)}        <gravity>false</gravity>
{inertial(mass, ii, ii, ii)}
{body}
      </link>"""


def _palm_link(name, side, parent=None, t=(0, 0, 0)):
    w, th, ln = K.PALM_SIZE
    cx, cy, cz = K.PALM_CENTER
    # P99: 手背侧 RGB-D 相机(rgbd_camera, 一路传感器同时出 RGB+深度)。
    #
    # 机位/朝向/分辨率**全部取自 `arm_kin.WRIST_CAM_*`** —— 那是唯一真源,
    # 验收脚本 `tests/test_p99.py` 用同一组常量把世界点投到图像上比对。这里
    # 只负责写成 SDF, 不再自己算角度。
    #
    # 为什么装在 -y(手背)侧、且必须退到 -0.040: 掌板厚 0.024(y ∈ ±0.012)、
    # 指根 knuckle 更厚(0.036, y ∈ ±0.018)、腕球 r0.03 盖到掌系 z≈+0.005。
    # P98 那版放在 y=-0.027 / z=+0.004 正好卡在腕球底面上, 朝内用了
    # pitch=1.5708 却又把相机推到 ±x 侧 —— 实测画面是"看着自己的躯干/另一条
    # 手臂", 深度全在 0.06~0.26m。
    #
    # 朝向的约定(踩出来的, 见 arm_kin.WRIST_CAM_RPY 与
    # tests/diag/diag_wrist_proj.py): gz 相机光轴是传感器系 **+x**,
    # R = Rz(yaw)·Ry(pitch)·Rx(roll), 所以 pitch=1.4202 + yaw=1.5708 得到
    # "沿手指向下 8.6°、偏向抓握面"的视线; roll 只转画面不转光轴。
    cam_y, cam_z = K.WRIST_CAM_T[1], K.WRIST_CAM_T[2]
    cam_roll, cam_pitch, cam_yaw = K.WRIST_CAM_RPY
    cam_t = K.WRIST_CAM_T
    # ---- P100: 掌侧近距相机(抓取特写) ----------------------------------
    # 机位/朝向同样只认 `arm_kin.SIDE_CAM_*`。这里唯一要小心的是**相机块不能
    # 挡住镜头**: 掌侧机位的视线斜着掠过掌缘, 所以外壳做成 18x20x26mm 的小盒
    # 贴在 x=±0.046(掌板外缘 0.0375 之外), 近裁 0.03 把盒子自身面切掉。
    s_t, s_rpy = K.side_cam_pose(side)
    s_pose = ('%.4f %.4f %.4f %.4f %.4f %.4f'
              % (s_t[0], s_t[1], s_t[2], s_rpy[0], s_rpy[1], s_rpy[2]))
    s_body = visual('side_cam_body', g_box(0.018, 0.020, 0.026), MAT_ARM2,
                    t=s_t)
    s_brk = visual('side_cam_brk', g_box(0.014, 0.018, 0.014), MAT_ARM,
                   t=(s_t[0] * 0.88, s_t[1], s_t[2]))
    s_sensor = f"""
        <sensor name="side_rgbd" type="rgbd_camera">
          <pose>{s_pose}</pose>
          <topic>/side_cam_{side}</topic>
          <update_rate>5</update_rate>
          <camera name="d">
            <horizontal_fov>{K.SIDE_CAM_HFOV:.3f}</horizontal_fov>
            <image><width>{K.SIDE_CAM_W}</width><height>{K.SIDE_CAM_H}</height>
              <format>R8G8B8</format></image>
            <clip><near>{K.SIDE_CAM_NEAR}</near><far>{K.SIDE_CAM_FAR}</far></clip>
            <depth_camera><output>depths</output></depth_camera>
          </camera>
          <always_on>1</always_on>
          <visualize>false</visualize>
        </sensor>"""
    cam_body = visual('wrist_cam_body', g_box(0.024, 0.024, 0.030), MAT_ARM2,
                      t=cam_t)
    cam_brk = visual('wrist_cam_brk', g_box(0.016, 0.028, 0.014), MAT_ARM,
                     t=(0, -0.026, -0.040))
    cam_pose = ('%.4f %.4f %.4f %.4f %.4f %.4f'
                % (cam_t[0], cam_y, cam_z, cam_roll, cam_pitch, cam_yaw))
    cam_sensor = f"""
        <sensor name="wrist_rgbd" type="rgbd_camera">
          <pose>{cam_pose}</pose>
          <topic>/wrist_cam_{side}</topic>
          <update_rate>5</update_rate>
          <camera name="d">
            <horizontal_fov>{K.WRIST_CAM_HFOV:.3f}</horizontal_fov>
            <image><width>{K.WRIST_CAM_W}</width><height>{K.WRIST_CAM_H}</height>
              <format>R8G8B8</format></image>
            <clip><near>0.05</near><far>8.0</far></clip>
            <depth_camera><output>depths</output></depth_camera>
          </camera>
          <always_on>1</always_on>
          <visualize>false</visualize>
        </sensor>"""
    # ---- 机位标定: 同一位置挂 4 个不同朝向的小相机, 一次性看清 gz 的
    #      传感器姿态约定到底怎么作用(DRINK_WRIST_DBG=1 才生成)。
    #      只挂在右手掌心原点, 128x96@2Hz, 不参与任何业务。
    dbg = ''
    if os.environ.get('DRINK_WRIST_DBG') == '1' and side == 'R':
        for tag, rot in (('id', '0 0 0'), ('pz', '0 1.5708 0'),
                         ('mz', '0 -1.5708 0'), ('pz_y90', '0 1.5708 1.5708'),
                         ('rz', '1.5708 0 0')):
            dbg += f"""
        <sensor name="dbg_{tag}" type="rgbd_camera">
          <pose>0 {cam_y} {cam_z} {rot}</pose>
          <topic>/dbg_cam_{tag}</topic>
          <update_rate>2</update_rate>
          <camera name="d">
            <horizontal_fov>1.2</horizontal_fov>
            <image><width>128</width><height>96</height>
              <format>R8G8B8</format></image>
            <clip><near>0.05</near><far>8.0</far></clip>
          </camera>
          <always_on>1</always_on>
          <visualize>false</visualize>
        </sensor>"""
    return f"""      <link name="{name}">
{_lpose(parent, t)}        <gravity>false</gravity>
{inertial(0.28, 0.004, 0.004, 0.004)}
{visual('palm', g_box(w, th, ln), MAT_HAND, t=(cx, cy, cz))}
{visual('knuckle', g_box(w * 0.92, th * 1.5, 0.012), MAT_ARM2,
        t=(0, cy, cz - ln / 2 - 0.004))}
{cam_brk}{cam_body}{cam_sensor}{s_brk}{s_body}{s_sensor}{dbg}
      </link>"""


def _finger_link(name, length, width, thumb=False, parent=None, t=(0, 0, 0)):
    return f"""      <link name="{name}">
{_lpose(parent, t)}        <gravity>false</gravity>
{inertial(0.012, 2e-4, 2e-4, 8e-5)}
{visual('v', g_box(width, width, length), MAT_HAND,
        t=(0, 0, -length / 2))}
{visual('tip', g_sph(width * 0.52), MAT_ARM2, t=(0, 0, -length))}
      </link>"""


def gen_robot_ext():
    parts = []
    # ---- 躯干立柱 + 肩梁
    parts.append(f"""      <link name="torso">
        <pose relative_to="chassis">0 0 -0.13 0 0 0</pose>
        <inertial>
          <mass>3.6</mass>
          <inertia><ixx>0.230</ixx><iyy>0.235</iyy><izz>0.035</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
{collision('c_col', g_box(0.18, 0.22, 0.78), t=(-0.08, 0, 0.55))}
{collision('c_bar', g_box(0.20, 0.42, 0.10), t=(-0.06, 0, 0.90))}
{visual('v_col', g_box(0.18, 0.22, 0.78), MAT_ARM, t=(-0.08, 0, 0.55))}
{visual('v_bar', g_box(0.20, 0.42, 0.10), MAT_ARM2, t=(-0.06, 0, 0.90))}
{visual('v_badge', g_box(0.02, 0.16, 0.10), ('0.9 0.55 0.15', '0.9 0.55 0.15'),
        t=(0.012, 0, 0.66))}
      </link>""")
    parts.append("""      <joint name="torso_joint" type="fixed">
        <parent>chassis</parent>
        <child>torso</child>
      </joint>""")
    for side in ('R', 'L'):
        parts.append(gen_arm(side))
    # ---- 抓取附着(Gazebo 官方 DetachableJoint): 掌 ↔ 物体
    # R_kettle: 右手抓壶   L_cup: 左手抓杯
    # R_cup   : 双手捧杯时右掌也参与夹持(仅在单手不稳时启用, 同时挂两条
    #           固定关节会让杯子过约束, 所以二者互斥)
    # **默认不生成**, 见文件头 "抓取附着" 一节: child_model 指向世界级模型时
    # 会破坏机械臂关节约束。要复现那个实验请 DRINK_DETACH=1。
    for tag, plink, cmodel in ([('R_kettle', 'R_palm', 'kettle'),
                                ('L_cup', 'L_palm', 'cup'),
                                ('R_cup', 'R_palm', 'cup')]
                               if USE_DETACH else []):
        parts.append(f"""      <plugin filename="gz-sim-detachable-joint-system"
              name="gz::sim::systems::DetachableJoint">
        <parent_link>{plink}</parent_link>
        <child_model>{cmodel}</child_model>
        <child_link>body</child_link>
        <attach_topic>/drink/attach_{tag}</attach_topic>
        <detach_topic>/drink/detach_{tag}</detach_topic>
      </plugin>""")
    # ---- 反馈通道(Gazebo 官方插件)
    # a) 关节状态: 每个关节的真实角度/速度。这是"下达指令 → 实际到位"的
    #    唯一可信来源, 也是 P85 手掌 FK 自检的输入(用真实 q 算 FK 再比位姿)。
    parts.append("""      <plugin filename="gz-sim-joint-state-publisher-system"
              name="gz::sim::systems::JointStatePublisher">
        <topic>/drink/joint_state</topic>
        <update_rate>50</update_rate>
      </plugin>""")
    # 链接位姿不在这里发: /world/follow_world/dynamic_pose/info 本来就把
    # "每个模型 + 每个 link" 全量广播出来(实测 8s/888 条, 含 R_palm/L_palm/
    # kettle/cup), 直接订阅即可。P84 曾试过 gz-sim-pose-publisher-system,
    # 但 gz8 的 PosePublisher 在指定 <topic> 时发的是单条 gz.msgs.Pose
    # (多实体复用同一话题), 与 ros_gz_bridge 期望的 Pose_V 类型不符,
    # 桥接侧一条都转换不出来 —— 已弃用。
    return '\n'.join(parts)


# ---------------------------------------------------------------- 人
def gen_person_ext():
    """人的双臂(肩俯仰 + 左右各一条外展自由度) + 杯子附着点。

    两条 revolute 肩关节分工:
      * `person_arm_joint`(整体, 绕 y)—— 举杯到嘴边/仰杯。手点在世界
        z=1.00、肩点在 z=1.02, 绕 y 负向转 60° 左右, 手沿圆弧上抬 ~0.15m
        且杯口自然朝脸侧倾。
      * `person_arm_l_joint` / `person_arm_r_joint`(各绕自己肩点的 z)——
        P96 新增。原来两条手臂焊在同一个刚体上, "双手接杯"只是把杯子钉到
        两手中间那个缝隙里, 手自己一动不动; 接/还全靠上层的 attach/detach,
        画面上看不出"人伸手去接"这件事。拆开后两只手可以**张开迎杯 → 合拢
        捧住 → 交还时再张开**。

    P96 的三条硬约束:
      1. **杯子钉点不变**: 杯仍然钉在 `person_arms`(= 肩连线中点那个系),
         偏置仍是 `PERSON_HAND_IN_ARM=(0.20,0,-0.02)`。手是新 link, 钉点
         不跟着手走 —— 否则手一合拢杯子就被两只手各拽一半, 而 GraspCarry
         只允许一个 host。手只负责"合拢到杯面上"这一层表演。
      2. **两侧同号同义**: 两条关节的轴都写 (0,0,1), 由上层发**反号**值
         (左 +v / 右 −v)。写在 SDF 里用 (0,0,-1) 也行, 但那样要靠 gz 真的
         尊重负轴; 上层反号是纯 Python, 一眼可验。见 §6.4 #60。
      3. **子 link 必须 gravity=false**: 手是 0.25kg 的轻 link, 一旦受重力
         位置控制器会跟重力矩较劲, 表现为手慢慢往下溜。
    """
    # skirt 原点在世界 z=0.28(裙心)。肩关节放 skirt 系 (0.10,0,0.74)
    # → 世界 (0.10,0,1.02); 手点目标世界 z=1.00 → skirt 系 0.72。
    #
    # P84 修复(用户反馈"人的双手悬浮在空中"):
    #   这个 link 的 <pose relative_to="skirt"> 已经把它自己的原点搬到了 sh,
    #   所以下面每个 visual 的 t 都必须是**相对 link 原点**的量。原来直接写了
    #   absolute 的 mid / hand —— 等于把肩偏移又加了一遍, 手臂和手被整体抬高
    #   0.74m, 看上去就是"双手悬在头顶半空、人身上没有手臂"。
    #   凡是 <link><pose> 带非零平移的地方, 其 visual 的 t 都必须减掉这个平移。
    #
    # 姿态: 双臂从躯干两侧的肩点 (y=±0.155) 向前收拢到手 (y=±0.085),
    #   即"双手在胸前合捧"——这既是人接杯/举杯的真实姿势, 也让手臂在
    #   正视和侧视下都能看出是手臂, 而不是埋在躯干里的小凸起。
    #   躯干是半径 0.17 的胶囊, 肩点 x=0.10 处该 y 的躯干表面在
    #   x = sqrt(0.17²-0.155²) = 0.070, 所以肩球(x=0.10)整颗露在外面。
    # P96: 这两个数搬到 scene_layout 去了(PERSON_SHOULDER_Y / PERSON_HAND_Y)。
    # 上层算"手心世界位姿"要用同一份(见 L.PERSON_HAND_IN_ARML/R), 摆 SDF
    # 也用这一份 —— 否则改了模型忘了改判据, 判据会去量一个不存在的手。
    SH_Y = L.PERSON_SHOULDER_Y          # 肩点左右偏移(手臂 link 原点)
    HAND_Y = L.PERSON_HAND_Y            # 手点左右偏移(向中线收拢)
    #
    # 手臂配色: 原来是和躯干一模一样的 0.18/0.44/0.82, 在软件光栅下手臂同色
    # 同亮度, 与躯干边界分不出来 —— 复核时误判成"人没有手臂"。改成偏亮的
    # 钢蓝, 肩点再加一颗球把"肩关节"画出来。
    # 注意不会影响 person_vision: 那个检测器找的是**暗**(V<=70)且低饱和的
    # 裙摆(世界 z 0.24~0.68), 而手臂在 z~1.0 且亮, 两者不重叠。
    C_ARM = ('0.42 0.66 0.95', '0.42 0.66 0.95')
    C_HAND = ('0.96 0.82 0.68', '0.96 0.82 0.68')
    ARM_R = 0.040
    HAND_R = 0.052
    SH_R = 0.055
    # 手离肩的向量(在各自的肩 link 系里): 向外收 0.070, 向前 0.20, 向下 0.02。
    # 手臂长 |d| = 0.2128。

    def arm_link(side, sgn):
        """一条手臂 = 一个绕肩点转的 link(含手臂柱 + 手掌球)。

        sgn=+1 是人的左侧(+y), -1 是右侧。link 原点就放在**肩点**上 ——
        这样 `<joint>` 不写 pose(#38: gz-sim 8 会忽略它)时, 转轴自然落在
        肩点, 手臂是绕肩外展而不是绕腰。
        """
        # 手在肩的**内侧**(|y| 更小), 所以 sgn 侧的手 y 偏移是 -sgn*(SH_Y-HAND_Y)
        d = (0.20, -sgn * (SH_Y - HAND_Y), -0.02)
        ln = math.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2)
        rpy = align_z(d)
        return """      <link name="person_arm_%s">
        <pose relative_to="person_arms">%s</pose>
        <gravity>false</gravity>
%s
%s
%s
      </link>""" % (side, pose_str((0.0, sgn * SH_Y, 0.0)),
                    inertial(0.25, 0.002, 0.002, 0.002),
                    visual('arm_%s' % side, g_cyl(ARM_R, ln), C_ARM,
                           t=(d[0] / 2.0, d[1] / 2.0, d[2] / 2.0), rpy=rpy),
                    visual('hand_%s' % side, g_sph(HAND_R), C_HAND, t=d))

    def arm_joint(side):
        # 轴统一 (0,0,1); 上层对右侧发反号(见本函数头 P96 约束 2)。
        # 行程 ±0.60rad: 上层的开合幅度是 0.42rad, 留 0.18rad 余量给
        # "合拢时再捏紧一点"。effort 必须给 400 —— 它是关节力矩硬限, 会盖掉
        # JPC 的 <cmd_max>, 给小了关节会被限死(§6.4 #17)。
        return """      <joint name="person_arm_%s_joint" type="revolute">
        <parent>person_arms</parent>
        <child>person_arm_%s</child>
        <axis><xyz>0 0 1</xyz>
          <limit><lower>-0.60</lower><upper>0.60</upper>
            <effort>400</effort><velocity>8</velocity></limit>
          <dynamics><damping>0.15</damping></dynamics>
        </axis>
      </joint>""" % (side, side)

    parts = []
    # 肩架: 只留两颗肩球。它是杯子钉点的宿主, 也是俯仰关节的载体。
    parts.append("""      <link name="person_arms">
        <pose relative_to="skirt">%s</pose>
        <gravity>false</gravity>
%s
%s
%s
      </link>""" % (pose_str((0.10, 0.0, 0.74)),
                inertial(0.6, 0.004, 0.004, 0.004),
                visual('sh_l', g_sph(SH_R), C_ARM, t=(0.0, SH_Y, 0.0)),
                visual('sh_r', g_sph(SH_R), C_ARM, t=(0.0, -SH_Y, 0.0))))
    parts.append(arm_link('l', 1))
    parts.append(arm_link('r', -1))
    parts.append(f"""      <joint name="person_arm_joint" type="revolute">
        <parent>skirt</parent>
        <child>person_arms</child>
        <axis><xyz>0 1 0</xyz>
          <limit><lower>-1.35</lower><upper>0.10</upper>
            <effort>30</effort><velocity>2</velocity></limit>
          <dynamics><damping>0.30</damping></dynamics>
        </axis>
      </joint>""")
    parts.append(arm_joint('l'))
    parts.append(arm_joint('r'))
    parts.append(jpc_plugin('person_arm_joint', 40.0, 3.0, 2.0))
    # 外展关节: 0.25kg 的轻 link + 速度模式控制器。p/d 参照肩关节下调一档
    # (惯量只有它的 1/3), 再高会在到位后自激(§6.4 #8)。
    parts.append(jpc_plugin('person_arm_l_joint', 30.0, 2.0, 2.0))
    parts.append(jpc_plugin('person_arm_r_joint', 30.0, 2.0, 2.0))
    # 人的"接杯"附着点: 与机械臂那三个同理, 默认不生成(见文件头"抓取附着")。
    if USE_DETACH:
        parts.append("""      <plugin filename="gz-sim-detachable-joint-system"
              name="gz::sim::systems::DetachableJoint">
        <parent_link>person_arms</parent_link>
        <child_model>cup</child_model>
        <child_link>body</child_link>
        <attach_topic>/drink/attach_person_cup</attach_topic>
        <detach_topic>/drink/detach_person_cup</detach_topic>
      </plugin>""")
    return '\n'.join(parts)


PERSON_HAND_Z_REL = L.PERSON_HAND_Z - 0.28


# ---------------------------------------------------------------- 桌上物件
def gen_props():
    tx, ty = L.TABLE_XY
    out = []
    # ---- 桌子(静态)
    legs = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            legs.append(collision('leg%d%d' % (sx, sy),
                                  g_box(L.TABLE_LEG, L.TABLE_LEG,
                                        L.TABLE_TOP_Z - L.TABLE_PLATE_T),
                                  t=(sx * (L.TABLE_W / 2 - 0.06),
                                     sy * (L.TABLE_D / 2 - 0.06),
                                     (L.TABLE_TOP_Z - L.TABLE_PLATE_T) / 2)))
            # P98: 四角各加一条**视觉腿**(原来只有碰撞体, GUI 里桌面悬空)。
            # 与桌面同一 model/同一 link = 刚性固定, 桌动腿必随动。
            legs.append(visual('v_leg%d%d' % (sx, sy),
                               g_box(L.TABLE_LEG, L.TABLE_LEG,
                                     L.TABLE_TOP_Z - L.TABLE_PLATE_T),
                               ('0.48 0.35 0.24', '0.52 0.38 0.26'),
                               t=(sx * (L.TABLE_W / 2 - 0.06),
                                  sy * (L.TABLE_D / 2 - 0.06),
                                  (L.TABLE_TOP_Z - L.TABLE_PLATE_T) / 2)))
    out.append(f"""    <model name="table">
      <static>true</static>
      <pose>{pose_str((tx, ty, 0.0), (0, 0, L.TABLE_YAW))}</pose>
      <link name="link">
{collision('top', g_box(L.TABLE_W, L.TABLE_D, L.TABLE_PLATE_T),
           t=(0, 0, L.TABLE_TOP_Z - L.TABLE_PLATE_T / 2), mu=1.0)}
{chr(10).join(legs)}
{visual('v_top', g_box(L.TABLE_W, L.TABLE_D, L.TABLE_PLATE_T),
        ('0.62 0.46 0.32', '0.66 0.49 0.34'),
        t=(0, 0, L.TABLE_TOP_Z - L.TABLE_PLATE_T / 2))}
{visual('v_lip', g_box(L.TABLE_W + 0.04, L.TABLE_D + 0.04, 0.018),
        ('0.48 0.35 0.24', '0.52 0.38 0.26'),
        t=(0, 0, L.TABLE_TOP_Z - L.TABLE_PLATE_T))}
      </link>
    </model>""")
    # ---- 水壶(动态)
    out.append(_kettle_sdf())
    # ---- 水杯(动态, 带水位滑动副)
    out.append(_cup_sdf())
    # ---- P98 运动学液面: 液面盘 + 水柱段(运行时由 liquid.py place 驱动)
    out.append(liquid.sdf_props())
    # ---- 观察相机
    dx = L.OBS_CAM_LOOK[0] - L.OBS_CAM_POS[0]
    dy = L.OBS_CAM_LOOK[1] - L.OBS_CAM_POS[1]
    dz = L.OBS_CAM_LOOK[2] - L.OBS_CAM_POS[2]
    n = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = -math.asin(dz / n)
    out.append(f"""    <model name="drink_cam">
      <static>true</static>
      <pose>{pose_str(L.OBS_CAM_POS, (0, pitch, yaw))}</pose>
      <link name="link">
        <sensor name="cam" type="camera">
          <topic>drink_cam/image</topic>
          <always_on>true</always_on>
          <visualize>false</visualize>
          <update_rate>5</update_rate>
          <camera>
            <horizontal_fov>{f(getattr(L, 'OBS_CAM_FOV', 1.30), 3)}</horizontal_fov>
            <image><width>640</width><height>480</height>
              <format>R8G8B8</format></image>
            <clip><near>0.2</near><far>20</far></clip>
          </camera>
        </sensor>
      </link>
    </model>""")
    return '\n'.join(out)


def _kettle_sdf():
    kx, ky = L.KETTLE_XY
    h = L.KETTLE_BODY_H
    r = L.KETTLE_BODY_R
    gx, gy, gz = L.KETTLE_GRIP_LOCAL
    sp = L.KETTLE_SPOUT_LOCAL
    spl = L.KETTLE_SPOUT_LEN
    # 壶嘴中心线方向(局部): 从 sp 沿 +x 略上扬
    tipv = (L.KETTLE_SPOUT_TIP_LOCAL[0] - sp[0], 0.0,
            L.KETTLE_SPOUT_TIP_LOCAL[2] - sp[2])
    rpy_sp = align_z(tipv)
    ln_sp = math.sqrt(sum(v * v for v in tipv))
    return f"""    <model name="kettle">
      <pose>{pose_str((kx, ky, L.KETTLE_CZ), (0, 0, L.KETTLE_YAW))}</pose>
      <link name="body">
        <!-- P86: 关重力。抓取附着由世界内的 GraspCarry 插件每 tick 写
             WorldPoseCmd 完成, 它只管位置不管速度; 若重力照常作用, 两次
             teleport 之间物体仍被加速, 松手瞬间残余速度会让它飞出去。
             关掉之后壶既不会被拽走, 也不会积累速度, 松手就停在原地 ——
             正是"放回桌面原位置"需要的语义。机械臂的 link 同理。 -->
        <gravity>false</gravity>
        <inertial>
          <pose>0 0 -0.02 0 0 0</pose>
          <mass>0.42</mass>
          <inertia><ixx>0.0026</ixx><iyy>0.0026</iyy><izz>0.0012</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
<!-- P86: 碰撞体与任何东西都不碰撞(collide_bitmask=0)。壶/杯是运动学
             驱动的(抓握期间由 GraspCarry 每 tick 钉到掌心), 碰撞体只会在钉住
             期间跟手指反复挤压、把接触冲量攒成速度, 一松钉就随机弹开 5~7mm。
             重力已关, 摆位全走 teleport, 去掉碰撞不改变可见行为。 -->
{collision('c_body', g_cyl(r, h), mu=0.7, mask='0x00')}
{visual('v_body', g_cyl(r, h), ('0.86 0.88 0.90', '0.86 0.88 0.90'))}
{visual('v_ring', g_cyl(r * 1.03, 0.014), ('0.30 0.34 0.38', '0.30 0.34 0.38'),
        t=(0, 0, h / 2 - 0.020))}
{visual('v_lid', g_cyl(r * 0.80, 0.016), ('0.20 0.22 0.25', '0.20 0.22 0.25'),
        t=(0, 0, h / 2 + 0.006))}
{visual('v_knob', g_sph(0.013), ('0.12 0.13 0.15', '0.12 0.13 0.15'),
        t=(0, 0, h / 2 + 0.022))}
{visual('v_water', g_cyl(r * 0.90, h * 0.55),
        ('0.20 0.55 0.85', '0.20 0.55 0.85'), t=(0, 0, -h * 0.18), alpha=0.30)}
{visual('v_handle_up', g_box(0.026, 0.020, 0.026),
        ('0.30 0.33 0.36', '0.30 0.33 0.36'), t=(gx + 0.012, 0, gz + 0.070))}
{visual('v_handle_dn', g_box(0.026, 0.020, 0.026),
        ('0.30 0.33 0.36', '0.30 0.33 0.36'), t=(gx + 0.012, 0, gz - 0.070))}
{collision('c_handle', g_box(0.026, L.KETTLE_GRIP_BAR_R * 2 + 0.004, 0.140),
           t=(gx, 0, gz), mu=0.9, mask='0x00')}
{visual('v_handle_bar', g_cyl(0.011, 0.140), ('0.30 0.33 0.36', '0.30 0.33 0.36'),
        t=(gx, 0, gz))}
{visual('v_spout', g_cyl(0.013, ln_sp), ('0.86 0.88 0.90', '0.86 0.88 0.90'),
        t=((sp[0] + L.KETTLE_SPOUT_TIP_LOCAL[0]) / 2, 0,
           (sp[2] + L.KETTLE_SPOUT_TIP_LOCAL[2]) / 2), rpy=rpy_sp)}
      </link>
      <link name="jet">
        <pose relative_to="body">{pose_str((L.KETTLE_SPOUT_TIP_LOCAL[0] - 0.115 * tipv[0] / ln_sp,
                         0.0,
                         L.KETTLE_SPOUT_TIP_LOCAL[2]
                         - 0.115 * tipv[2] / ln_sp), rpy_sp)}</pose>
        <gravity>false</gravity>
        <inertial><mass>0.01</mass>
          <inertia><ixx>2e-4</ixx><iyy>2e-4</iyy><izz>2e-4</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
{visual('v_jet', g_cyl(0.0055, 0.115), ('0.25 0.62 0.92', '0.25 0.62 0.92'),
        t=(0, 0, -0.0575), alpha=0.40)}
      </link>
      <joint name="jet_joint" type="prismatic">
        <parent>body</parent>
        <child>jet</child>
        <axis><xyz>0 0 1</xyz>
          <limit><lower>0.0</lower><upper>0.24</upper>
            <effort>5</effort><velocity>1</velocity></limit>
          <dynamics><damping>0.10</damping></dynamics>
        </axis>
      </joint>
{jpc_plugin('jet_joint', 6.0, 0.6, 0.12)}
    </model>"""


def _cup_sdf():
    cx, cy = L.CUP_XY
    r, h = L.CUP_R, L.CUP_H
    bw = 0.026                          # 厚底座(把"未倒水"的水体藏在里面)
    return f"""    <model name="cup">
      <pose>{pose_str((cx, cy, L.CUP_CZ), (0, 0, L.CUP_YAW))}</pose>
      <link name="body">
        <!-- P86: 关重力, 理由同水壶(_kettle_sdf 里的注释) -->
        <gravity>false</gravity>
        <inertial>
          <pose>0 0 -0.02 0 0 0</pose>
          <mass>0.16</mass>
          <inertia><ixx>0.00021</ixx><iyy>0.00021</iyy><izz>0.00012</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
{collision('c_body', g_cyl(r, h), mu=0.8, mask='0x00')}
{collision('c_base', g_cyl(r * 0.94, bw), mu=1.0,
           t=(0, 0, -h / 2 + bw / 2), mask='0x00')}
{visual('v_body', g_cyl(r, h), ('0.90 0.93 0.96', '0.90 0.93 0.96'),
        alpha=0.45)}
{visual('v_base', g_cyl(r * 0.99, bw), ('0.55 0.60 0.68', '0.55 0.60 0.68'),
        t=(0, 0, -h / 2 + bw / 2))}
{visual('v_rim', g_cyl(r * 1.02, 0.008), ('0.75 0.80 0.86', '0.75 0.80 0.86'),
        t=(0, 0, h / 2 - 0.004))}
{visual('v_h1', g_box(0.012, 0.012, 0.050), ('0.75 0.80 0.86', '0.75 0.80 0.86'),
        t=(r + 0.014, 0, 0.012))}
{visual('v_h2', g_box(0.030, 0.012, 0.012), ('0.75 0.80 0.86', '0.75 0.80 0.86'),
        t=(r + 0.014, 0, 0.037))}
{visual('v_h3', g_box(0.030, 0.012, 0.012), ('0.75 0.80 0.86', '0.75 0.80 0.86'),
        t=(r + 0.014, 0, -0.013))}
      </link>
      <link name="water">
        <pose relative_to="body">{pose_str((0, 0, -h / 2))}</pose>
        <gravity>false</gravity>
        <inertial><mass>0.01</mass>
          <inertia><ixx>2e-4</ixx><iyy>2e-4</iyy><izz>2e-4</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
{visual('v_water', g_cyl(r * 0.90, 0.026), ('0.20 0.55 0.85', '0.20 0.55 0.85'),
        t=(0, 0, 0.013), alpha=0.35)}
      </link>
      <joint name="water_joint" type="prismatic">
        <parent>body</parent>
        <child>water</child>
        <axis><xyz>0 0 1</xyz>
          <limit><lower>0.0</lower><upper>{f(L.CUP_WATER_MAX, 4)}</upper>
            <effort>5</effort><velocity>0.5</velocity></limit>
          <dynamics><damping>0.05</damping></dynamics>
        </axis>
      </joint>
{jpc_plugin('water_joint', 8.0, 0.6, 0.04)}
    </model>"""


# ---------------------------------------------------------------- 世界级插件
def gen_world_ext():
    """世界级 system 插件声明(必须是 <world> 的直接子元素)。

    目前只有 P86 的抓取附着插件 `GraspCarry`
    =====================================
    它跑在 gz 世界内部, 每 1ms 的 PreUpdate 把"被抓住"的物体钉到掌心位姿。
    为什么不做成外部服务调用, 见
        vla_ws/src/vla_follow/gz_plugins/GraspCarry.cc 顶部注释
    以及 TaskPlan §2.3 / §6.4 #19~#20。
    """
    return """    <!-- P86 抓取附着: 世界内插件, 把 attach 过的物体每 tick 钉到掌心 -->
    <plugin filename="libGraspCarry.so" name="vla::GraspCarry">
      <topic>/drink/carry</topic>
      <ack_topic>/drink/carry/ack</ack_topic>
      <debug>1</debug>
    </plugin>"""


# ---------------------------------------------------------------- bridge
def gen_bridge():
    joints = []
    for side in ('R', 'L'):
        joints += ['%s_%s_joint' % (side, j[0]) for j in K.JOINTS]
        joints += [x + '_joint' for x in K.hand_joints(side)]
    joints += ['jet_joint', 'water_joint', 'person_arm_joint']
    # P96: 人的两条手臂各自的外展关节(左右反号发, 见 gen_person_ext)
    joints += ['person_arm_l_joint', 'person_arm_r_joint']
    lines = []
    for j in joints:
        lines.append("""- ros_topic_name: "/drink/cmd/%s"
  gz_topic_name: "/drink/cmd/%s"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: ROS_TO_GZ""" % (j, j))
    # 抓取附着(attach/detach)**走 bridge**。
    # P84 曾经不敢桥接: 当时用的是 DetachableJoint 的 attach 话题, 而
    # ROS_TO_GZ 的 Empty 桥接在桥接进程一启动就会让插件判定"收到 attach
    # 请求", 于是在 t=0 把 R_palm↔kettle、L_palm↔cup、person_arms↔cup 全
    # 焊死, 手臂一动就把壶/杯/人一起拖走。
    # P86 换成 GraspCarry 后不存在这个问题 —— 它只认显式的
    # "attach <model> <link>" 字符串, 桥接进程启动不会伪造指令。走 bridge 的
    # 好处是复用已被 P85 验证过的链路(与 37 路关节指令同一条), 而且 ROS 侧
    # publish 是单向的, 不会遇到 gz 服务请求/应答那套退化问题。
    lines.append("""- ros_topic_name: "/drink/carry"
  gz_topic_name: "/drink/carry"
  ros_type_name: "std_msgs/msg/String"
  gz_type_name: "gz.msgs.StringMsg"
  direction: ROS_TO_GZ""")
    # 附着结果回报(插件 -> ROS)。为什么需要: 单向话题在发现完成前会静默丢
    # 消息(实测第一轮 attach 三条全丢, 而 5 秒后的 detach 正常到达), 上层
    # 就会"以为抓上了其实没抓"。有了 ack, drink_task 可以等到明确结果再走。
    lines.append("""- ros_topic_name: "/drink/carry/ack"
  gz_topic_name: "/drink/carry/ack"
  ros_type_name: "std_msgs/msg/String"
  gz_type_name: "gz.msgs.StringMsg"
  direction: GZ_TO_ROS""")
    # 关节真实角度(Gazebo 官方 joint_state_publisher)
    lines.append("""- ros_topic_name: "/drink/joint_state"
  gz_topic_name: "/drink/joint_state"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: GZ_TO_ROS""")
    # 位姿不走桥: drink_task 用 gz.transport13 直接订阅
    # /world/follow_world/dynamic_pose/info (与 gt_tf 同一写法, 见 §2.1)
    lines.append("""- ros_topic_name: "/drink_cam/image"
  gz_topic_name: "drink_cam/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS""")
    # P99: 腕部 RGB-D —— 每侧 **RGB + 深度** 两路(6 路 × 5Hz = 30 msg/s,
    # 256x192 的 rgb8 只有 147KB/s, 桥接开销可忽略)。深度类型是
    # sensor_msgs/msg/Image(32FC1), 不是 PointCloud。
    for side in ('R', 'L'):
        for sub in ('image', 'depth_image'):
            lines.append(f"""- ros_topic_name: "/wrist_cam_{side}/{sub}"
  gz_topic_name: "/wrist_cam_{side}/{sub}"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS""")
    # P100: 掌侧近距相机, 同样是每侧 RGB + 深度两路(192x144@5Hz)
    for side in ('R', 'L'):
        for sub in ('image', 'depth_image'):
            lines.append(f"""- ros_topic_name: "/side_cam_{side}/{sub}"
  gz_topic_name: "/side_cam_{side}/{sub}"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS""")
    return '\n'.join(lines)


# ---------------------------------------------------------------- 配置
DEFAULT_CFG = {
    "approach_speed": 0.16,
    "approach_gap": 1.05,
    "approach_tol_m": 0.03,
    "yaw_tol_deg": 2.0,
    "turn_speed": 0.45,
    "seg_slow_s": 0.55,
    "grip_close_s": 0.9,
    "grip_open_s": 0.6,
    "lift_kettle_h": 0.12,
    "kettle_tilt_deg": 68.0,
    "pour_s": 7.0,
    "pour_hold_s": 1.2,
    "jet_gap": 0.075,
    "cup_lift_z": 0.86,
    "handover_wait_s": 30.0,
    "drink_s": 5.0,
    "person_return_s": 3.2,
    "backoff_m": 0.0,
    "place_tol_m": 0.01,
}


def write_cfg():
    with open(CONFIG, 'w', encoding='utf-8') as fh:
        json.dump(DEFAULT_CFG, fh, indent=2, ensure_ascii=False)
        fh.write('\n')
    return CONFIG


# ---------------------------------------------------------------- 写文件
def markers(path, name):
    """按文件类型给出注释风格: SDF 用 <!-- -->, YAML 用 #。"""
    if path.endswith(('.yml', '.yaml')):
        return '# DRINK:BEGIN %s' % name, '# DRINK:END %s' % name
    return '<!-- DRINK:BEGIN %s -->' % name, '<!-- DRINK:END %s -->' % name


def replace_block(path, name, text):
    with open(path, 'r', encoding='utf-8') as fh:
        src = fh.read()
    b, e = markers(path, name)
    i, j = src.find(b), src.find(e)
    if i < 0 or j < 0 or j < i:
        raise SystemExit('标记缺失: %s 里找不到 %s / %s' % (path, b, e))
    new = src[:i + len(b)] + '\n' + text + '\n' + src[j:]
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(new)
    return len(new.splitlines())


def main():
    if len(sys.argv) > 1 and sys.argv[1] == 'check':
        with open(WORLD, encoding='utf-8') as fh:
            src = fh.read()
        for name in ('world_ext', 'robot_ext', 'person_ext', 'props'):
            print(name, os.path.basename(WORLD),
                  bool(re.search(r'DRINK:BEGIN %s' % name, src)),
                  bool(re.search(r'DRINK:END %s' % name, src)))
        return
    n1 = replace_block(WORLD, 'robot_ext', gen_robot_ext())
    n2 = replace_block(WORLD, 'person_ext', gen_person_ext())
    n3 = replace_block(WORLD, 'props', gen_props())
    n0 = replace_block(WORLD, 'world_ext', gen_world_ext())
    n4 = replace_block(BRIDGE, 'bridge', gen_bridge())
    print('写回 work_area.sdf: world_ext/robot_ext/person_ext/props  (%d 行)' % n0)
    print('写回 bridge.yml: bridge 段 (%d 行)' % n4)
    print('写回 drink_config.json:', write_cfg())
    print('关节总数: %d (臂 14 + 手 20 + 水/水流 2 + 人的肩 1 + 人的双臂 2)'
          % (14 + 20 + 2 + 1 + 2))


if __name__ == '__main__':
    main()
