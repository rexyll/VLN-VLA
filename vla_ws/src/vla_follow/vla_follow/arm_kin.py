#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arm_kin.py —— 7 轴机械臂 + Shadow Hand 运动学 (P84)

**唯一真源**: 本文件的链参数同时被
    ① tools/gen_drink_scene.py 生成 SDF(关节偏移/轴/限位)
    ② drink_task 运行时正解/逆解
使用, 因此 SDF 与控制器不会走偏。

链结构(每侧 7 轴, 与人臂同构):
    肩偏航 pan(z) → 肩俯仰 lift(y) → 大臂自旋 roll(z, 位于肘点)
    → 肘 pitch(y) → 小臂自旋 roll(z, 位于腕点) → 腕 pitch(y) → 腕 roll(z)
    → 掌(palm, 距腕 0.035m)
q=0 时大臂/小臂沿 -z 垂下, 与 SDF 里 visual 的方向一致。

IK 用阻尼最小二乘(DLS) + 零空间偏置, 7 自由度对 6 维位姿目标冗余 1 维,
零空间偏置把解拉向"肘朝外/自然下垂"的参考位形, 并保证逐帧连续。
"""
import math

import numpy as np

# ---------------------------------------------------------------- 链参数
L1 = 0.32                 # 肩→肘
L2 = 0.30                 # 肘→腕
PALM_OFF = 0.035          # 腕→掌原点(沿 -z)
SHOULDER = {'R': (0.02, -0.19, 0.95), 'L': (0.02, 0.19, 0.95)}

# (关节名, 相对父链的平移, 旋转轴, 限位)
# 平移在**父关节坐标系**下; 与 SDF 的 <pose relative_to="parent_link"> 一致
JOINTS = [
    ('pan',     (0.0, 0.0, 0.0),    (0.0, 0.0, 1.0), (-2.70, 2.70)),
    ('lift',    (0.0, 0.0, 0.0),    (0.0, 1.0, 0.0), (-2.40, 0.90)),
    ('up_roll', (0.0, 0.0, -L1),    (0.0, 0.0, 1.0), (-2.00, 2.00)),
    ('elbow',   (0.0, 0.0, 0.0),    (0.0, 1.0, 0.0), (-2.50, 0.15)),
    ('fore_roll', (0.0, 0.0, -L2),  (0.0, 0.0, 1.0), (-2.00, 2.00)),
    ('wrist_p', (0.0, 0.0, 0.0),    (0.0, 1.0, 0.0), (-1.50, 1.50)),
    ('wrist_r', (0.0, 0.0, 0.0),    (0.0, 0.0, 1.0), (-2.80, 2.80)),
]
NJ = len(JOINTS)
LIM_LO = np.array([j[3][0] for j in JOINTS])
LIM_HI = np.array([j[3][1] for j in JOINTS])
# 零空间参考位形: 大臂略前伸、肘微屈、腕摆平(视觉自然)
Q_REST = np.array([0.0, -0.55, 0.0, -0.85, 0.0, 0.35, 0.0])

# P98: 悬垂位形 = 全零 —— 运动链在零位时沿 -z 竖直向下(肩 (0.02,±0.19,0.95),
# 上臂 L1 + 前臂 L2 + 掌 0.035 → 掌心在肩下 0.615m, 掌系 -z 即正下方),
# 手指顺势朝下。抓取编排仍从 Q_REST 出发, 悬垂只作初始/待命态。
Q_HANG = np.zeros(NJ)

# ---------------------------------------------------------------- 手掌/手指
PALM_SIZE = (0.075, 0.024, 0.080)     # 掌心板(宽x 厚y 长z)
PALM_CENTER = (0.0, 0.0, -0.030)      # 掌心板中心(掌坐标系=腕坐标系)
FINGER_BASE_Z = -0.070                # 四指根部 z
FINGER_X = (0.030, 0.010, -0.010, -0.030)   # 食指/中指/无名/小指 的 x
FINGER_NAMES = ('index', 'middle', 'ring', 'little')
PROX_LEN, DIST_LEN = 0.034, 0.026
FINGER_W = 0.014
THUMB_X = {'R': 0.050, 'L': -0.050}   # 拇指在掌侧(x 方向), 左右镜像
THUMB_BASE = (0.0, 0.014, -0.050)     # y,z 相对掌坐标系(拇指根)

# 抓握预设: 近节/末节弯曲角(rad)。正角=朝掌心(+y_local)收拢
GRIP_OPEN = {'prox': 0.02, 'dist': 0.05, 'opp': 0.0, 'th': 0.10}
# P98: 悬垂手型 —— 十指全零 = 并拢伸直, 与臂同向(向下)
GRIP_HANG = {'prox': 0.0, 'dist': 0.0, 'opp': 0.0, 'th': 0.0}
GRIP_CUP = {'prox': 1.15, 'dist': 1.30, 'opp': 0.55, 'th': 0.95}
GRIP_KETTLE = {'prox': 1.30, 'dist': 1.40, 'opp': 0.65, 'th': 1.10}

# 物体在掌坐标系中的位置(抓稳时): (x=掌宽向, y=掌心法向, z=手指延伸向)
# 手掌板在前方 0.070 处结束, 物体中心落在这里 => 被手指与掌心夹住
OBJ_IN_PALM_CUP = (0.0, 0.036, -0.078)
OBJ_IN_PALM_BAR = (0.0, 0.030, -0.068)

# ---------------------------------------------------------------- 腕部相机
# P99: 手背侧 RGB-D 相机。**唯一真源** —— SDF 由 tools/gen_drink_scene.py
# 从这里取值生成, 验收脚本也用它把世界点投到图像上(两边不一致就谈不上"看到")。
#
# 位姿在**掌系**下: 位置 (0, -0.040, -0.040) = 掌心板背面(-y 侧)中段;
# 朝向 (roll, pitch, yaw) = (0, 1.4202, 1.5708)。
#
# 两个 gz 约定(踩过才知道, 见 tests/diag/diag_wrist_proj.py):
#   ① 相机的**光轴是传感器系 +x**(不是 -z), 所以"沿手指看下去"要 pitch≈90°
#      把 +x 转到掌系 -z;
#   ② 旋转按 R = Rz(yaw)·Ry(pitch)·Rx(roll) 作用, roll 只绕光轴转、不动光轴 ——
#      想让光轴再抬 θ 朝掌心侧(+y), 只能 `pitch = 90°-θ` 且 `yaw = 90°`:
#        forward = (cosφcosψ, cosφsinψ, −sinφ) → φ=1.4202、ψ=π/2
#        ⇒ (0, +0.15, −0.99), 即"向下 8.6°、偏向抓握面"。
#      这样视野中心落在指尖下沿: 前方 0.2~1m 的桌面能进画幅, 四指留在上沿
#      当参照(实测悬垂位指尖投到 v≈45/192)。
#   ③ roll = π/2 是**构图**需要, 不是几何需要: 画面的"上"由相机 +z 定,
#      roll=0 时 +z 落在掌系 +y 上; 而抓取位掌系 +x 才是世界竖直方向 —— 于是
#      图像横躺着(实测预抓取位地平线是竖的)。给 π/2 后 +z 对准掌系 +x:
#      抓取/前伸这类工作姿态下画面正立, 悬垂位则转 90°(悬垂只是待命态, 不管)。
WRIST_CAM_T = (0.0, -0.040, -0.040)
WRIST_CAM_RPY = (1.5708, 1.4202, 1.5708)
WRIST_CAM_W, WRIST_CAM_H = 256, 192
WRIST_CAM_HFOV = 1.20          # rad, 与 SDF 的 <horizontal_fov> 一致

# ---- P100: 掌侧近距相机("抓取特写") -------------------------------------
# P99 的手背相机有个躲不掉的盲区: 它装在掌后 4cm、视线与手指平行, 抓取瞬间
# 被抓物就在指尖那一侧 —— 被自己的四指挡死(实测预抓取位壶把投影点上读到的是
# 9cm 外的指尖深度)。想真正看见"指间发生了什么", 只能换个**侧挂**机位。
#
# 机位: 掌板 x ∈ ±0.0375、手指 x ∈ [-0.030, +0.030]、拇指在 x=+0.050(R) /
#       −0.050(L)。所以**小指侧恒为 x=−0.0375**(右); 相机贴到 x=−0.046 就在
#       整只手的轮廓之外, 不会埋进掌板或指根。
# 朝向: 不写死角度, 而是"从机位看向 SIDE_CAM_TARGET、画面'上'取 SIDE_CAM_UP"
#       反解 rpy —— 这样换机位只要改两个点, 不用再凑欧拉角。目标点选在
#       "指尖与拇指之间、抓握面外 3cm"的地方, 也就是抓壶把/杯身时物体所在的
#       位置; '上'取掌系 +x, 与手背相机同一套构图约定(实测手背相机的 up
#       正是 +x, 而抓取位掌系 +x 才是世界竖直 —— 见上面 ③)。
#
# 视角是"从侧后方斜看手指合拢", 四指在画面一侧排成一条, 抓握空档在中央 ——
# 这是手背相机永远拍不到的构图。
# 候选朝向的离线筛选见 tests/diag/side_opt.py(不必起仿真)。
SIDE_CAM_X = 0.046
SIDE_CAM_Y = 0.0
SIDE_CAM_Z = -0.036
SIDE_CAM_TARGET = (0.0, 0.028, -0.095)    # 掌系: 指尖与拇指之间的抓握点
SIDE_CAM_UP = (1.0, 0.0, 0.0)             # 掌系: 画面"上"= 掌 +x(世界竖直)
SIDE_CAM_W, SIDE_CAM_H = 192, 144
SIDE_CAM_HFOV = 1.30
SIDE_CAM_NEAR, SIDE_CAM_FAR = 0.03, 5.0


def _look_at(cam, target, up):
    """"看向 target、上贴 up" → 3x3(列 = 相机 x/y/z)。"""
    f = np.array(target, float) - np.array(cam, float)
    f = f / np.linalg.norm(f)
    u = np.array(up, float)
    u = u - f * float(u @ f)
    if np.linalg.norm(u) < 1e-6:
        raise ValueError('up 与光轴平行')
    u = u / np.linalg.norm(u)
    return np.stack([f, np.cross(u, f), u], axis=1)


def side_cam_pose(side):
    """掌侧相机的 (平移, rpy), 掌坐标系。左右镜像: 机位 x 取反(目标点不变)。"""
    cam = (SIDE_CAM_X * (-1.0 if side == 'R' else 1.0),
           SIDE_CAM_Y, SIDE_CAM_Z)
    R = _look_at(cam, SIDE_CAM_TARGET, SIDE_CAM_UP)
    p = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    y = math.atan2(R[1, 0], R[0, 0])
    r = math.atan2(R[2, 1], R[2, 2])
    return cam, (r, p, y)


def rot_rpy(rpy):
    """(roll, pitch, yaw) → 3x3, 与 gz 的 <pose> 同一套约定(Rz·Ry·Rx)。"""
    r, p, y = rpy
    Rx = np.array([[1.0, 0.0, 0.0],
                   [0.0, math.cos(r), -math.sin(r)],
                   [0.0, math.sin(r), math.cos(r)]])
    Ry = np.array([[math.cos(p), 0.0, math.sin(p)],
                   [0.0, 1.0, 0.0],
                   [-math.sin(p), 0.0, math.cos(p)]])
    Rz = np.array([[math.cos(y), -math.sin(y), 0.0],
                   [math.sin(y), math.cos(y), 0.0],
                   [0.0, 0.0, 1.0]])
    return Rz @ Ry @ Rx


def wrist_cam_world(side, q, base):
    """腕部相机的世界位姿(4x4): FK 掌位姿 @ 相机掌系偏置。"""
    T = fk_world(side, q, base) @ frame(t=WRIST_CAM_T, R=rot_rpy(WRIST_CAM_RPY))
    return T


def wrist_cam_project(T_cam, p_world):
    """世界点 → (u, v, 轴向距离 m); 在相机背后返回 (None, None, d)。"""
    return _project(T_cam, p_world, WRIST_CAM_W, WRIST_CAM_H, WRIST_CAM_HFOV)


def side_cam_world(side, q, base):
    """掌侧近距相机的世界位姿(4x4)。"""
    t, rpy = side_cam_pose(side)
    return fk_world(side, q, base) @ frame(t=t, R=rot_rpy(rpy))


def side_cam_project(T_cam, p_world):
    """世界点 → (u, v, 轴向距离 m), 掌侧相机(192x144, HFOV 1.30)。"""
    return _project(T_cam, p_world, SIDE_CAM_W, SIDE_CAM_H, SIDE_CAM_HFOV)


def _project(T_cam, p_world, w, h, hfov):
    """针孔投影。像素约定: 光轴 +x, 相机 +y 朝左(u 减小), +z 朝上(v 减小)。

    这套约定是从 gz 实测反推的(见 tests/diag/diag_wrist_proj.py): 用 FK 出
    指尖世界点投到图上, 与深度图"最近 3cm 像素质心"差 20px 才算过。换相机
    只改 (w, h, hfov), 约定不变 —— 这也是把它抽出来的原因。
    """
    d = np.asarray(p_world, float) - T_cam[:3, 3]
    xc = float(d @ T_cam[:3, 0])
    yc = float(d @ T_cam[:3, 1])
    zc = float(d @ T_cam[:3, 2])
    if xc <= 0.02:
        return None, None, xc
    fx = (w / 2.0) / math.tan(hfov / 2.0)
    return (w / 2.0 - fx * yc / xc, h / 2.0 - fx * zc / xc, xc)


def hand_joints(side):
    """该侧手部 10 个关节名(与 SDF 顺序一致)"""
    out = []
    for fn in FINGER_NAMES:
        out.append('%s_hand_%s_1' % (side, fn))
        out.append('%s_hand_%s_2' % (side, fn))
    out.append('%s_hand_thumb_1' % side)
    out.append('%s_hand_thumb_2' % side)
    return out


def arm_joints(side):
    return ['%s_%s' % (side, j[0]) for j in JOINTS]


def all_joints(side):
    return arm_joints(side) + hand_joints(side)


def grip_values(side, preset):
    """把预设展开成 10 个手部关节角(顺序同 hand_joints)"""
    vals = []
    for _ in FINGER_NAMES:
        vals.append(preset['prox'])
        vals.append(preset['dist'])
    # 拇指: 先对指(opp)再屈曲; 左右镜像由 SDF 的轴方向处理, 这里给同一标量
    vals.append(preset['opp'])
    vals.append(preset['th'])
    return vals


# ---------------------------------------------------------------- 基础矩阵
def rot(axis, q):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    x, y, z = a
    c, s = math.cos(q), math.sin(q)
    C = 1.0 - c
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def trans(t):
    T = np.eye(4)
    T[:3, 3] = t
    return T


def frame(R=None, t=(0, 0, 0)):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    T[:3, 3] = t
    return T


def rot_x(q):
    return rot((1, 0, 0), q)


def rot_y(q):
    return rot((0, 1, 0), q)


def rot_z(q):
    return rot((0, 0, 1), q)


def quat_to_mat(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def mat_to_rpy(R):
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(math.cos(pitch)) < 1e-6:
        yaw = 0.0
        roll = math.atan2(-R[0, 1], R[1, 1])
    else:
        yaw = math.atan2(R[1, 0], R[0, 0])
        roll = math.atan2(R[2, 1], R[2, 2])
    return roll, pitch, yaw


# ---------------------------------------------------------------- 正解
def fk(side, q):
    """返回手掌(palm)坐标系的世界/本体位姿 4x4。q 为 7 个臂关节角。"""
    T = frame(t=SHOULDER[side])
    for i, (_n, off, axis, _lim) in enumerate(JOINTS):
        T = T @ trans(off) @ frame(rot(axis, q[i]))
    return T @ trans((0.0, 0.0, -PALM_OFF))


def fk_joint_frames(side, q):
    """每个关节的坐标系(用于几何雅可比)"""
    T = frame(t=SHOULDER[side])
    frames = []
    for i, (_n, off, axis, _lim) in enumerate(JOINTS):
        T = T @ trans(off)
        frames.append((T[:3, 3].copy(), T[:3, :3] @ np.array(axis, float)))
        T = T @ frame(rot(axis, q[i]))
    return frames


def pose_err(T_cur, T_tgt):
    """6 维位姿误差: [位置差, 旋转差(世界系轴角)]"""
    ep = T_tgt[:3, 3] - T_cur[:3, 3]
    Rd = T_tgt[:3, :3] @ T_cur[:3, :3].T
    # 轴角 (罗德里格斯)
    c = max(-1.0, min(1.0, (np.trace(Rd) - 1.0) / 2.0))
    ang = math.acos(c)
    if ang < 1e-7:
        er = np.zeros(3)
    elif abs(math.pi - ang) < 1e-4:
        # 近 180°: 取对角最大列
        A = (Rd + np.eye(3)) / 2.0
        v = np.sqrt(np.maximum(np.diag(A), 0.0))
        k = int(np.argmax(v))
        axis = A[:, k] / max(1e-9, np.linalg.norm(A[:, k]))
        er = axis * ang
    else:
        axis = np.array([Rd[2, 1] - Rd[1, 2], Rd[0, 2] - Rd[2, 0],
                         Rd[1, 0] - Rd[0, 1]]) / (2.0 * math.sin(ang))
        er = axis * ang
    return np.concatenate([ep, er])


def ik(side, T_tgt, seed=None, iters=120, tol=2e-3, lam=0.05,
       w_rot=1.0, null_gain=0.06):
    """阻尼最小二乘 IK。返回 (q, 误差范数)。失败时返回最优解。"""
    q = np.array(Q_REST if seed is None else seed, float)
    q = np.clip(q, LIM_LO, LIM_HI)
    best_q, best_e = q.copy(), 1e9
    for _ in range(iters):
        Tc = fk(side, q)
        e = pose_err(Tc, T_tgt)
        e[3:] *= w_rot
        en = float(np.linalg.norm(e))
        if en < best_e:
            best_e, best_q = en, q.copy()
        if en < tol:
            break
        J = np.zeros((6, NJ))
        pe = Tc[:3, 3]
        for i, (pi, zi) in enumerate(fk_joint_frames(side, q)):
            J[:3, i] = np.cross(zi, pe - pi)
            J[3:, i] = zi * w_rot
        JJt = J @ J.T + (lam ** 2) * np.eye(6)
        dq = J.T @ np.linalg.solve(JJt, e)
        # 零空间: 朝参考位形微调(只在不影响末端时生效)
        Jpinv = J.T @ np.linalg.inv(JJt)
        N = np.eye(NJ) - Jpinv @ J
        dq += N @ (null_gain * (np.clip(Q_REST, LIM_LO, LIM_HI) - q))
        m = float(np.max(np.abs(dq)))
        if m > 0.25:
            dq *= 0.25 / m
        q = np.clip(q + dq, LIM_LO, LIM_HI)
    return best_q, best_e


def reach_ok(side, T_tgt, seed=None):
    q, e = ik(side, T_tgt, seed)
    return e < 0.02, e, q


# ---------------------------------------------------------------- KDL 逆解
# 开源优先: KDL(Orocos) 的 LMA / NR_JL 是工业界用了十几年的 6/7 轴逆解实现,
# 本机 `python3-pykdl` 已预装。自研 DLS(上面的 ik()) 只在 PyKDL 缺失时兜底,
# 保证节点不会因为缺一个包就整体跑不起来。
try:
    import PyKDL
    HAVE_KDL = True
except Exception:                                    # pragma: no cover
    HAVE_KDL = False

_KDL_CACHE = {}


def _kdl_chain(side):
    if side in _KDL_CACHE:
        return _KDL_CACHE[side]
    ch = PyKDL.Chain()
    for name, off, axis, _lim in JOINTS:
        if abs(axis[2]) > 0.5:
            jt = PyKDL.Joint.RotZ
        elif abs(axis[1]) > 0.5:
            jt = PyKDL.Joint.RotY
        else:
            jt = PyKDL.Joint.RotX
        ch.addSegment(PyKDL.Segment(
            name, PyKDL.Joint(jt), PyKDL.Frame(PyKDL.Vector(*off))))
    ch.addSegment(PyKDL.Segment(
        'palm', PyKDL.Joint(PyKDL.Joint.Fixed),
        PyKDL.Frame(PyKDL.Vector(0.0, 0.0, -PALM_OFF))))
    _KDL_CACHE[side] = ch
    return ch


def _to_kdl_frame(T):
    R, t = T[:3, :3], T[:3, 3]
    return PyKDL.Frame(
        PyKDL.Rotation(R[0, 0], R[0, 1], R[0, 2],
                       R[1, 0], R[1, 1], R[1, 2],
                       R[2, 0], R[2, 1], R[2, 2]),
        PyKDL.Vector(t[0], t[1], t[2]))


def _kdl_solver(side, kind):
    key = (side, kind)
    if key in _KDL_CACHE:
        return _KDL_CACHE[key]
    ch = _kdl_chain(side)
    if kind == 'lma':
        try:
            s = PyKDL.ChainIkSolverPos_LMA(ch)
        except Exception:                              # 旧绑定少一个重载
            s = PyKDL.ChainIkSolverPos_LMA(ch, 1e-5, 300)
    else:
        lo = PyKDL.JntArray(NJ)
        hi = PyKDL.JntArray(NJ)
        for i in range(NJ):
            lo[i] = float(LIM_LO[i])
            hi[i] = float(LIM_HI[i])
        # 踩坑记录(P86): NR_JL 只保存 fk/vel 求解器的**裸指针**, 不持有引用。
        # 写成 `ChainIkSolverPos_NR_JL(ch, lo, hi, ChainFkSolverPos_recursive(ch),
        # ChainIkSolverVel_pinv(ch), ...)` 会让两个临时对象在语句结束就被 GC,
        # 之后每次 CartToJnt 都在悬垂指针上跑 —— 实测直接 Segfault (core dump),
        # 且只在 ik_best 的"越限→nr_jl 复算"分支触发, 平时看不出来。
        # 这里显式持有, 与 solver 同生命周期挂在 _KDL_CACHE 里。
        fk_s = PyKDL.ChainFkSolverPos_recursive(ch)
        ikv = PyKDL.ChainIkSolverVel_pinv(ch)
        _KDL_CACHE[key + ('keep',)] = (fk_s, ikv)
        s = PyKDL.ChainIkSolverPos_NR_JL(ch, lo, hi, fk_s, ikv, 300, 1e-5)
    _KDL_CACHE[key] = s
    return s


def ik_kdl(side, T_tgt, seed=None, kind='lma'):
    """KDL 逆解一次。返回 (q, 位姿误差范数)。"""
    if not HAVE_KDL:
        return None, None
    Tb = frame(t=SHOULDER[side])
    Tloc = _to_kdl_frame(np.linalg.inv(Tb) @ T_tgt)
    q0 = PyKDL.JntArray(NJ)
    base = np.clip(np.array(Q_REST if seed is None else seed, float),
                   LIM_LO, LIM_HI)
    for i in range(NJ):
        q0[i] = float(base[i])
    qout = PyKDL.JntArray(NJ)
    solver = _kdl_solver(side, kind)
    try:
        solver.CartToJnt(q0, Tloc, qout)
    except Exception:
        return None, None
    q = np.array([qout[i] for i in range(NJ)], float)
    return q, float(np.linalg.norm(pose_err(fk(side, q), T_tgt)))


def ik_best(side, T_tgt, seed=None):
    """生产用逆解入口: KDL 优先, 越限夹紧 → NR_JL(带限位) 复算 → DLS 兜底。

    返回 (q, err, 来源标签)。err 为 [位置(m) + 旋转(rad)] 的范数。
    """
    cands = []
    if HAVE_KDL:
        q, e = ik_kdl(side, T_tgt, seed, 'lma')
        if q is not None:
            cands.append((q, e, 'kdl-lma'))
            if np.any(q < LIM_LO - 1e-6) or np.any(q > LIM_HI + 1e-6):
                q2, e2 = ik_kdl(side, T_tgt, np.clip(q, LIM_LO, LIM_HI),
                                'nr_jl')
                if q2 is not None:
                    cands.append((np.clip(q2, LIM_LO, LIM_HI), e2, 'kdl-nrjl'))
            else:
                e2 = float(np.linalg.norm(pose_err(fk(side, q), T_tgt)))
                cands.append((np.clip(q, LIM_LO, LIM_HI), e2, 'kdl-lma-inlim'))
    q, e = ik(side, T_tgt, seed)
    cands.append((q, float(np.linalg.norm(pose_err(fk(side, q), T_tgt))),
                  'dls'))
    # **误差必须按"夹到限位之后"的 q 重算**(P103 踩坑)。原来 KDL 那一路报的是
    # 未夹紧解的残差: 解出来 wrist_p=1.70(限位 1.50)时残差报 0.3mm, 夹紧后
    # 真值是 200mm+ —— 上层据此以为"解到了", 伺服却把指令夹住, 手臂停在
    # 半路, `reach` 卡在 244mm 不动。所有候选一律夹紧后重算再比。
    fixed = []
    for _q, _e, _tag in cands:
        _q = np.clip(_q, LIM_LO, LIM_HI)
        fixed.append((_q, float(np.linalg.norm(pose_err(fk(side, _q), T_tgt))),
                      _tag))
    q, e, tag = min(fixed, key=lambda c: c[1])
    return q, e, tag


# ---- 关节居中优先的逆解(过渡位/预备位用) --------------------------------
# `ik_best` 只按"误差最小"挑解, 而 7 轴对 6 维位姿有 1 维冗余 —— 同一目标不
# 同种子会给出 fore_roll 贴 +2.00、或 wrist_r 贴 −2.80 的解。位形本身对, 但
# 下一步的闭环 IK 就**动不了**了: 2026-10-28 实测预备位落到 wrist_r=−2.69 的
# 分支上, 举杯那一步 IK 直接发散 1166mm, 整条 PREPARE 挂成 FAIL_PREPARE。
#
# 左右镜像的换算是: S = diag(1,−1,1) 是非法旋转, S·rot(a,θ)·S = rot(Sa,−θ),
# 于是绕 ±y 的关节(lift/elbow/wrist_p)角度**不变**, 绕 z 的(pan/up_roll/
# fore_roll/wrist_r)要取反。
MIRROR_SIGN = np.array([-1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0])


def mirror_q(q):
    """右臂关节角 → 左臂关节角(左右对称位姿用)。"""
    return np.asarray(q, float) * MIRROR_SIGN


def joint_stress(q):
    """关节"贴限位"的程度: 0 = 全在行程中点, 1 = 有轴顶到限位。"""
    mid = 0.5 * (LIM_LO + LIM_HI)
    half = 0.5 * (LIM_HI - LIM_LO)
    return float(np.max(np.abs((np.asarray(q, float) - mid) / half)))


def ik_natural(side, T_tgt, seed=None, tol=2e-3,
               up_rolls=(-0.9, -0.6, -0.3, 0.0, 0.3, 0.6, 0.9)):
    """在合格解里挑**关节最居中**的那个(过渡位/预备位专用)。

    优先级: ①种子本身就已经解出这个位姿 → 原样返回(镜像种子就是靠这条生效);
    ②否则扫 up_roll 种子, 取 stress 最小的合格解。
    """
    q0 = np.asarray(seed if seed is not None else Q_REST, float)
    if np.linalg.norm(pose_err(fk(side, q0), T_tgt)) < tol * 0.5:
        return q0.copy(), float(np.linalg.norm(pose_err(fk(side, q0), T_tgt))), \
            'seed'
    best = None
    for v in up_rolls:
        s = q0.copy()
        s[2] = v
        q, err, how = ik_best(side, T_tgt, s)
        if err > tol:
            continue
        st = joint_stress(q)
        if best is None or st < best[0]:
            best = (st, q, err, how)
    if best is None:
        return ik_best(side, T_tgt, seed)
    return best[1], best[2], best[3] + '/nat'


# ---- 肘部朝向(冗余的那 1 维怎么用) -------------------------------------
# 7 轴对 6 维位姿冗余的一维, 物理上就是"大臂绕自己轴转多少"(up_roll), 它决定
# 肘往哪个方向拐。KDL 的 LM 只按种子收敛, 给它 home 的种子就得到"肘朝下"的解
# —— 端杯子时看着像端托盘, 不像人抱着杯子(P88 起一直如此, 2026-10-28 用户指出)。
#
# 做法: 换一组跨越 up_roll 的种子各解一次, 在**残差合格**的解里按
#   score = 横向外扩 − 1.2 × max(0, 肘比手还高)
# 挑。横向外扩 = sign·(肘y − 肩y), L 侧 sign=+1(外=+y)、R 侧 −1(外=−y)。
# 离线验证见 tests/diag/elbow_study.py: L 选中 up_roll≈−1.56(肘在肩外 135mm、
# 与手齐高), R 选中 +0.90(肘在肩外 95mm、比手低 190mm)。R 抬不起来不是选错解
# —— 右手要伸到身前 0.5m 处, 0.615m 的臂接近伸直, 肘本来就在下面。
ELBOW_OUT_SEEDS = (-2.0, -1.7, -1.4, -0.9, 0.9, 1.4, 1.7, 2.0)


def elbow_pos(side, q):
    """肘点(本体坐标系)。up_roll 绕大臂轴转、不动自己的原点, 所以它就是肘。"""
    T = frame(t=SHOULDER[side])
    for i, (_n, off, axis, _lim) in enumerate(JOINTS):
        T = T @ trans(off)
        if i == 2:
            return T[:3, 3].copy()
        T = T @ frame(rot(axis, q[i]))
    return T[:3, 3].copy()


def elbow_metrics(side, q):
    """肘部朝向的度量(本体坐标系), 验收与调参共用同一套口径。

    为什么要两个"外扩"口径
    ======================
    `out_base`(肘y − 肩y) 是**相对车身**的: 杯子正前方 0.50m 时它很好看
    (L +135 / R +95mm), 但杯子被推到臂长极限(0.70m)时手臂几乎伸直, 肘
    本来就落在肩-腕连线上 —— 实测掉到 +12 / +22mm。这时"肘不朝外"是**几
    何必然**(直臂没有肘), 拿它当判据会把"够不着"误判成"姿态不对"。

    `out_line` 把肘点投到**肩→腕轴的垂线**上, 再取"外侧"分量: 它量的是
    "肘往体侧架出去多少", 与手臂伸出多远无关。同一组实车姿态下的对照:

        姿态            out_base     out_line    droop
        R 用 Q_REST 解    -79mm       -14mm      +238mm   ← 老版本(肘垂在体侧)
        R 用肘外解        +95mm      +164mm      +167mm
        L 极限伸出(+12)   +12mm       +91mm       -10mm

    `droop` = 肘点低于肩→腕轴线多少(正=更低)。"端着托盘"那种姿势 droop
    很大(200mm 级), 肘架到体侧时接近 0。R 侧托杯时手掌在杯子外侧下方,
    droop 天然有 60~170mm, 所以它只记录、不当硬判据。

    外侧方向: 把肩→腕的水平方向 u 转 90°, L 取 (-u_y, u_x)、R 取
    (u_y, -u_x) —— 手臂朝 +x 时分别就是 +y 和 −y, 与 out_base 同号。
    """
    sh = np.asarray(SHOULDER[side], float)
    el = elbow_pos(side, q)
    palm = fk(side, q)[:3, 3]
    d = palm - sh
    n_d = float(np.linalg.norm(d))
    sign = 1.0 if side == 'L' else -1.0
    out_base = sign * (el[1] - sh[1])
    if n_d < 1e-6:
        return {'out_base_mm': round(out_base * 1e3, 1),
                'out_line_mm': 0.0, 'droop_mm': 0.0, 'reach_m': 0.0,
                'up_roll': round(float(q[2]), 2)}
    u = d / n_d
    e = el - sh
    perp = e - float(e @ u) * u
    n = np.array([-u[1], u[0], 0.0]) if side == 'L' else np.array([u[1], -u[0], 0.0])
    return {'out_base_mm': round(out_base * 1e3, 1),
            'out_line_mm': round(float(perp @ n) * 1e3, 1),
            'droop_mm': round(float(-perp[2]) * 1e3, 1),
            'reach_m': round(n_d, 3),
            'up_roll': round(float(q[2]), 2)}


def ik_best_out(side, T_tgt, seed=None, tol=2e-3):
    """残差合格的解里挑"肘最朝外"的那个。全都不合格就退回 ik_best。

    打分口径 = **`elbow_metrics` 的 `out_line`**(肘点在肩→腕轴垂线上的外侧
    分量), 而不是"肘y − 肩y"。

    为什么换(2026-10-28 实测): 7 轴对 6 维位姿的冗余维就是 up_roll, 不同种子
    会收敛到"肘外架"或"肘拐进中线"两类解。原来按车身横向打分, 在**手臂几乎
    伸直**或**杯子不在正前方**时两侧的分数都趋近 0, 于是同一套布景连跑三次
    会时好时坏(L 侧线口径 +131 / +147 / **−3.5**mm)。而验收判的正是线口径
    —— 用同一个量做优化目标, 抖动就没了。`droop`(肘低于轴线多少)给一个软
    惩罚: "端着托盘"那种肘朝下的解不该因为横向大一点就赢。
    """
    q0 = np.asarray(seed if seed is not None else Q_REST, float)
    best = None
    for v in ELBOW_OUT_SEEDS:
        s = q0.copy()
        s[2] = v
        q, err, how = ik_best(side, T_tgt, s)
        if err > tol:
            continue
        m = elbow_metrics(side, q)
        score = m['out_line_mm'] - 0.6 * max(0.0, m['droop_mm'])
        if best is None or score > best[0]:
            best = (score, q, err, how)
    if best is None:
        return ik_best(side, T_tgt, seed)
    return best[1], best[2], best[3] + '/out'


# ---- 人体关节规则(验收/优化共用口径) ------------------------------------
# 用户 2026-10-28: "倒水的时候机器人左手臂肘部弯向内侧, 违背人体骨骼运动规律;
# 另外检查其他运动关节有没有违背人体肢体正常运动轨迹的现象"。
#
# 实测原因是**逆解落到了极端的等价解**上: 抓杯成功那一刻的左臂是
#     q = [−0.34, +0.44, −0.13, −2.39, −0.71, +0.46, −2.54]
# —— 肘折到 137°(限位 −2.50 就在旁边)、前臂扭转 −3.25rad(186°, 人的旋前+
# 旋后加起来才 180°)。这个位形**满足掌心位姿**, 所以闭环报"到位 9.3mm", 但
# 画面上就是"肘拐进怀里、手腕反向拧了一圈"。接着举杯那一步 IK 从这条分支上
# 出不去, 直接顶到 lift=+0.90 / elbow=−2.50 两个限位上, 残差 119mm。
#
# 7 轴对 6 维位姿冗余 1 维, 同一目标有整族等价解, 而 KDL 只按种子收敛 ——
# 选哪一条是**策略问题**, 不是精度问题。下面这套就是"策略":
#   ① 只用残差合格的解;  ② 按"人体代价"挑最小;  ③ 平价时取离种子最近的,
#   保证逐帧连续、不会两条分支来回跳。
HUMAN_UPROLL = 2.00        # 肩内旋上限(= 机械限位)
# 为什么不是 1.75/1.90/1.95: "掌心下扣 + 桌面高度 + 贴近身体"的**放回**位形,
# 它的可行分支实测集中在 up_roll −1.79~−1.95, 而且最好的那几条本身就在 2.0
# 附近 —— 阈值定在 1.90 或 1.95 都是从这群候选中间切过去, 每一轮挑到哪条全看
# 种子运气, 验收在阈值两侧跳(实测同一版代码 −1.84 过、>1.95 挂)。
# 定到 2.00(= 机械限位)就落在整个族群之外, "别贴着限位走"交给贴限代价项
# (0.85×限位起算, 系数 1.2)去做 —— 和 R6 用机械限位 1.50 是同一个口径。
                           # 原定 1.75(100°)是拍的。P103 用实测位形反查分支
                           # (`tests/diag/_p103_live_sweep.py`): "掌下扣、桌面
                           # 高度把杯子放下"那个掌心位姿, 稠密撒开 pan/lift/
                           # up_roll/elbow 解出 95 条独立分支, **0 条**能把
                           # |up_roll| 压到 1.75 以内(最低 1.79, 集中在 1.79~
                           # 1.92) —— 这个位形本身就压在这个角上, 说明 1.75 是
                           # 门槛定低了, 不是姿态跑偏了。人体"肱骨内旋 ~90° +
                           # 肩胛上旋"在桌面高度前伸时约 105~110°, 取 1.90 既
                           # 覆盖该位形, 又离机械限位(2.00)留 0.10rad, 仍然能
                           # 拦住"拧到限位"的那种(老 elbow='out' 策略实测顶到
                           # 1.84 且肘折 −2.39, 现在由 R7/R5 一并兜住)。
HUMAN_TWIST = 2.00         # 前臂旋前+旋后合计上限(115°; 人的极限约 180°,
                           # 但那是"特意去拧", 端杯子不需要)
HUMAN_ELBOW = 2.25         # 肘折叠上限(129°)
# 为什么不是 2.10(120°): 桌面高度(0.72)比肩(0.95)低 180mm, 又要掌心朝下捧杯,
# 肘本来就得折 100°+; 实测"放回"那一站的右肘在 −1.85~−1.94, 再抬 120mm 就是
# −1.97~−2.20 —— 2.10 会从这群位形中间切过去。人的肘屈曲上限 145°(2.53),
# 129° 仍在人体行程内; 而当年那个"折太深把行程吃光"的病态位形是 **−2.39
# (137°)**(抓杯后紧接着举杯差 103mm), 2.25 照样拦得住。
HUMAN_WRIST_P = 1.50       # 腕屈伸上限(86°): 人的掌屈能到 80~90°, 而本臂的
                           # 机械限位正好是 1.50 —— 这条实际判的是"顶到限位",
                           # 而不是"屈伸看着别扭"(实测捧杯 1.41 是正常的举杯姿势)
HUMAN_OUT_MM = 40.0        # 肘相对肩至少要外扩这么多(否则算"内收")
HUMAN_HOLD = 0.25          # "够好, 不必换分支"的代价闸(迟滞用)。参考: 合规
                           # 位形多在 0.05~0.25; 倒水时"肘贴肋"那条(外扩 40mm)
                           # 0.42、捧杯右臂内收那条 0.39 —— 都在闸外, 该重挑。
                           # 注意闸门必须**低于** HUMAN_HARD_W: 越限的解一律
                           # 重挑, 不能靠迟滞"沿用"下来。
HUMAN_EDGE = 0.85          # 进入限位的最后 15% 算"贴着限位走", 开始计税
HUMAN_SOFT_W = 1.2         # 贴限项的系数
HUMAN_HARD_W = 0.60        # 越过任何一条硬规则(哪怕 0.01)的起步代价 —— 必须
                           # 压过所有软项之和(单项最大约 0.4), 否则优化器会用
                           # "硬越限一点点"去换别的指标, 见 human_penalty ①
HUMAN_HARD_K = 6.0         # 越限的边际代价


def human_penalty(side, q):
    """人体规则代价(rad 当量): 0 = 完全像人, 越大越别扭。

    三层结构(层次必须分清, 否则会互相串):
      ① **越限**(hard): 越过 R4~R7 的硬界, **一起扣 0.60** 再按超出量加罚
         (`HUMAN_HARD_W + HUMAN_HARD_K·e`)。为什么要有这个"起步价": 只按超出量
         罚的话, 越限 0.01 只值 0.01 —— 比"把另一个关节多含 0.1rad"的软代价
         (0.12)还便宜, 于是优化器宁可**硬越限**去换别的指标。实测放回位形:
         `up_roll=−1.91`(越限 0.01, 代价 0.452) 打败了完全合规的
         `(−1.85, twist −1.81)`(0.492) —— 就差这 0.04。加了起步价后合规解
         一定赢, 而且会落在**两边都留余量**的地方, 不再贴着某个硬界来回跳。
      ② **贴限**(soft): 进入限位最后 15% 就开始计, 系数 1.2。
         只有 ① 时, 代价在边界上是**平的**, "刚好 1.99"和"1.74"一样好, 系统就
         一路贴着限位走, 验收在阈值两侧来回跳(实测抓杯 L 第一轮 −1.98 过、
         第二轮 −2.01 挂, 其实是同一条分支)。加了这个斜坡, 解会主动往限位中段
         靠: 同一个抓杯位形 100 条分支里合规 11 条, 扭转从 −1.98 排到 −1.74,
         现在会选 −1.74 那条(代价 0.048 vs 0.336), 外扩反而更好(+199 vs +193mm)。
      ③ **构型偏好**: 内收(肘过体侧中线/贴肋)、关节应力、以及"在合格解里偏爱
         把肘架出去"。
    """
    q = np.asarray(q, float)
    m = elbow_metrics(side, q)
    p = 0.0

    def lim_term(x, lim):
        e = max(0.0, abs(float(x)) - lim)
        s = max(0.0, abs(float(x)) - HUMAN_EDGE * lim)
        out = HUMAN_SOFT_W * s
        if e > 0.0:
            out += HUMAN_HARD_W + HUMAN_HARD_K * e
        return out

    p += lim_term(q[2], HUMAN_UPROLL)
    p += lim_term(q[4] + q[6], HUMAN_TWIST)
    p += lim_term(q[3], HUMAN_ELBOW)
    p += lim_term(q[5], HUMAN_WRIST_P)
    # 肘穿过身体中线 = 内收, 是最刺眼的那个(用户指的就是这个)
    p += 1.5 * max(0.0, -m['out_base_mm'] / 1000.0)
    p += 0.5 * max(0.0, (HUMAN_OUT_MM - m['out_line_mm']) / 1000.0)
    p += 0.30 * joint_stress(q)
    # 在合格的解里**偏好肘架出去**(捧杯/端碗时肘贴着身体是"托盘", 不是抱)。
    # 这一项只在几个等价解之间做选择, 上面那些违规项优先。
    p -= 1.0 * min(m['out_line_mm'], 250.0) / 1000.0
    return p


def violation_list(side, q):
    """逐条列出违反的人体规则(日志/验收用)。

    **没有"手过中线"这一条**(P103 第六轮删掉): 人体规则里"手不许过身体中线"
    只在物体摆在正前方时才有意义。实测杯子会落在基座中线**左右各 ~110mm**
    (进站侧向误差), 这时左手去捧杯必然跨过中线 —— 掌相对**同侧肩**的横向
    偏移实测 −5 ~ −224mm 全都是正常的捧杯位形, 拿绝对中线判会误报。
    "手拐进怀里"这件事由 R2(肘) + `out_line_mm`(肘离肩→腕连线的横向偏移)
    两条兜住, 它们都是**相对自己的肩**量的, 与物体摆在哪无关。
    """
    q = np.asarray(q, float)
    m = elbow_metrics(side, q)
    sgn = 1.0 if side == 'L' else -1.0
    sh = np.asarray(SHOULDER[side], float)
    el = elbow_pos(side, q)
    bad = []
    if q[3] > 0.02:
        bad.append('R1肘反折')
    if sgn * el[1] < -0.005:
        bad.append('R2肘过中线')
    if abs(q[2]) > HUMAN_UPROLL:
        bad.append('R4肩内旋')
    if abs(q[4] + q[6]) > HUMAN_TWIST:
        bad.append('R5前臂扭转')
    if abs(q[5]) > HUMAN_WRIST_P:
        bad.append('R6腕反折')
    if abs(q[3]) > HUMAN_ELBOW:
        # R7 肘过度折叠: 人的肘能屈到 ~145°, 但**端着东西**时肘尖不会贴到
        # 自己肩上。实测抓杯那一步的"等价解"里肘折到 137°(q=−2.39), 紧接着
        # 举杯就差 103mm 够不到 —— 折叠把剩下的行程全吃掉了。
        bad.append('R7肘过折')
    return bad


def ik_human(side, T_tgt, seed=None, tol=2e-3, seeds=None):
    """在残差合格的解里挑**最像人**的那个(整个人形动作链的默认策略)。

    候选 = 种子本身 + 一组跨 pan/lift/up_roll/elbow 的种子。7 轴对 6 维位姿
    冗余 1 维, 同一目标往往有 20~100 条等价解, 只有少数几条落在人的行程里。

    为什么四维都要撒(P103 第二轮实测)
    ---------------------------------
    原来只在 up_roll × elbow 上撒(pan/lift 钉死 Q_REST 的 0/−0.55), 结果
    倒水时右手拿到的解外扩线只有 +40mm。反查发现: 同一掌心位姿解出 100 条
    分支, **87 条合规, 最好的外扩 +173mm**, 但那条分支的初值在 pan≈−0.26、
    lift≈−1.95 —— 种子撒不到的地方。凡是"手里拿着东西、胳膊要抬高/侧开"
    的位形, 好分支都在 pan/lift 上岔开, 只在 up_roll/elbow 上撒等于只看
    "原地转肩"这一小片。
    """
    q0 = np.asarray(seed if seed is not None else Q_REST, float)
    # **迟滞**: 种子解本身"既合规又不太别扭"就沿用它, 不换分支。
    # 这条比"每轮都重挑最优"重要得多: 闭环补位时种子就是"刚才那一步",
    # 每轮重挑会让手臂在两个等价分支之间来回挪 —— 实测举杯那 8 轮里
    # up_roll 从 −1.56 一路被推到 −2.00(限位)、肘折到 −2.50(限位),
    # 掌心还在原地, 最后残差 74mm。分支是"姿态策略", 只该在**进入一个动作
    # 时定一次**; 进了同一条分支就别再挑了。
    #
    # 但"合规"不够, 得加代价闸: 倒水时右臂那条分支 R1~R7 全过、代价却高达
    # 0.29(肘贴在体侧, 外扩只有 40mm), 于是每次都被"沿用"下来, 永远换不到
    # 那个 0.07/外扩 173mm 的分支。闸门取 HUMAN_HOLD: 明显别扭的解不算"够好",
    # 该重挑就重挑。
    qf, ef, hf = ik_best(side, T_tgt, q0)
    if ef <= tol and not violation_list(side, qf) \
            and human_penalty(side, qf) <= HUMAN_HOLD:
        return qf, ef, hf + '/human-hold'
    # 候选种子用**中立体形 Q_REST** 起解。
    # 不能拿 q0 当底: 一旦当前位形已经落在坏分支上(实测左肘 y=−52mm, 越过
    # 身体中线), 以它为底的种子全都收敛回那条分支的邻域, 扫一圈也没有干净解,
    # 于是"没有更好的" —— 越陷越深。中立体形起解才能把整族等价解都摆出来。
    base_q = Q_REST.copy()
    cands = []
    # 两档残差闸: 先按 tol 找; 一个合规解都没有就放宽到 4mm 再找一遍。
    # 为什么要放宽 —— 收敛了但差几毫米的解**照样能执行**(伺服会把剩下的毫米
    # 吃掉), 而"宁可要一个越限的精确解"毫无道理: 实测张开手指后那次压实,
    # 按 2mm 闸一个合规解都没进来, 于是回退成种子原解, 左臂被挑到
    # `q=[−1.53,−1.30,+1.90,−2.42,…]`(up_roll 顶 R4、肘折 2.42 越 R7)。
    for gate in (tol, max(tol, 4e-3)):
        for v in (seeds or HUMAN_SEEDS):
            s = base_q.copy()
            s[0], s[1], s[2], s[3] = v[0], v[1], v[2], v[3]
            q, err, how = ik_best(side, T_tgt, s)
            if err > gate:
                continue
            q = np.clip(q, LIM_LO, LIM_HI)
            cands.append((q, err, how))
        if any(not violation_list(side, q) for q, _e, _h in cands):
            break
    if not cands:
        return ik_best(side, T_tgt, seed)

    def cost(c):
        q, e, _h = c
        # 违规代价优先(量级压过其它项); 连续性只当**同等优劣时的偏好**,
        # 所以权重只给 0.12 —— 原来 0.35 会让"离当前位形近"盖过"外扩差
        # 120mm"这种量级差: 倒水右手就是这么被钉在内收分支上的
        # (近分支代价 0.29+0.35×0.02, 好分支 0.07+0.35×0.94)。
        # 残差项(2.0/m)只在代价打平时起作用, 让"更准的那个"胜出。
        return (human_penalty(side, q) + 2.0 * float(e)
                + 0.12 * float(np.max(np.abs(q - q0))))
    q, err, how = min(cands, key=cost)
    return q, err, how + '/human'


# 候选种子: pan × lift × up_roll × elbow(3×3×9×4 = 324 组, 单次 ik_best 是
# 2~3ms, 一轮约 0.9s —— 只在**进入动作**时定一次分支, 花得起)。
# pan 给"肩往左/正/右开", lift 给"垂/半抬/抬", 这两维是原来缺的:
# 手持物抬高/放下时, 好分支就是靠 pan 偏 0.2~0.3rad、lift 到 −1.9 才岔出来的。
HUMAN_SEEDS = [(p, lf, v, e)
               for p in (-0.9, 0.0, 0.9)
               for lf in (-1.4, -0.55, 0.1)
               for v in (-1.6, -1.2, -0.8, -0.4, 0.0, 0.4, 0.8, 1.2, 1.6)
               for e in (-0.5, -1.1, -1.7)]


# ---------------------------------------------------------------- 坐标系
def base_pose(x, y, yaw, z=0.0):
    """底盘位姿 → 4x4 (base_link 在轮轴中点正下方的地面)"""
    return frame(rot_z(yaw), (x, y, z))


def ik_world(side, T_world, base, seed=None, elbow=None):
    """世界系目标 → 转成本体坐标系再解。base 可为 (x,y,yaw) 或 4x4。

    elbow='out' 时在合格解里挑"肘朝外"的那个(见 ik_best_out) —— 双手捧杯/
    递杯这类姿态要用, 否则 KDL 会给出肘朝下的解。
    elbow='human' 时按 `ik_human` 挑"最像人"的解(肘不内收、前臂不反拧、不顶
    限位) —— 抓杯/举杯/倒水这类长链动作用这个, 单看"肘朝外"不够: 实测抓杯
    那一步的等价解里肘是朝外的, 但肘折到 137°、手腕拧了 186°, 画面上一眼假。
    """
    Tb = base if isinstance(base, np.ndarray) else base_pose(*base)
    T_loc = np.linalg.inv(Tb) @ T_world
    if elbow == 'human':
        return ik_human(side, T_loc, seed)
    if elbow == 'out':
        return ik_best_out(side, T_loc, seed)
    return ik_best(side, T_loc, seed)


def fk_world(side, q, base):
    Tb = base if isinstance(base, np.ndarray) else base_pose(*base)
    return Tb @ fk(side, q)


# ------------------------------------------------- 人体式收臂 / 抬臂(关节路点)
# 用户 2026-10-28: "放回水杯后机器人略微往后退, 使车与桌子的距离大于
# 机械臂+灵巧手整体长度, 然后机械臂和灵巧手**从前方自然往下放**, 前臂以
# **肘部为支点**回收、回到与上臂平行的状态; 不能让机械臂做翻转等违背人体
# 骨骼运动规律的动作。"
#
# 为什么不能直接 `arm.move(Q_HANG)`: 关节空间的直线让 7 个关节同时走,
# 而 carry 位形里 wrist_p/wrist_r 各有 1.2~1.5rad —— 前臂还没转到竖直,
# 掌心就先翻过去了, 画面上是"手腕拧一圈再甩下来"。
#
# 人的顺序(本函数给出的三段路点, 也是唯一被允许的执行顺序):
#   ① 折肘 —— 只动 elbow, 腕部归零; pan/lift/up_roll 一律不动
#              ⇒ 肘的世界位置**严格不变**, 前臂绕肘尖回收;
#   ② 摆肩 —— pan/lift/up_roll → 0, 折着的前臂跟着大臂垂下来;
#   ③ 伸肘 —— elbow → 0, 前臂展开成与上臂同向 ⇒ 整条臂竖直悬垂。
# 抬臂(去倒水前的预备位)是这条路径的逆序: 肩往上抬、同时以肘为支点弯曲。
ARM_REACH = L1 + L2 + PALM_OFF + 0.060     # 肩→指尖 = 0.715m
HANG_FOLD = -1.75                          # 折肘角(限位 -2.50, 还留 43°)
# 抬臂预备位: 掌心在车前 0.38m、高 0.82m(桌面 0.72 + 壶身握把)、左右 0.105。
# 0.38 是扫出来的: 0.30/0.34 时手臂折得太深(肘 ≥2.1rad), 0.38 时肘 ~1.9rad
# 且左右两臂的关节应力都落在 0.55(行程中部), 后面抓杯/抓壶都还动得开。
READY_R, READY_Z, READY_Y = 0.38, 0.82, 0.105


def _waypoints(qa, qb, step):
    """从 qa 到 qb 的直线路点(含 qb, 不含 qa), 每步最大关节增量 step。"""
    qa = np.asarray(qa, float)
    qb = np.asarray(qb, float)
    d = float(np.max(np.abs(qb - qa)))
    n = max(1, int(math.ceil(d / step)))
    return [qa + (qb - qa) * ((i + 1) / float(n)) for i in range(n)]


def hang_path(q0, fold=HANG_FOLD, step=0.30):
    """q0 → 悬垂(Q_HANG) 的人体顺序路点表(见上面的三段)。"""
    q0 = np.asarray(q0, float)
    q1 = q0.copy()
    q1[3] = min(q0[3], fold)      # 本来就折得更深就别往外翻
    q1[4] = 0.0                   # 前臂自转归零(纯绕前臂轴, 不改变指向)
    q1[5] = 0.0
    q1[6] = 0.0
    q2 = q1.copy()
    q2[0] = 0.0
    q2[1] = 0.0
    q2[2] = 0.0
    q3 = np.zeros(NJ)
    out = []
    for qa, qb in ((q0, q1), (q1, q2), (q2, q3)):
        out.extend(_waypoints(qa, qb, step))
    return out


def raise_path(q0, q1, step=0.30):
    """悬垂 → 抬臂预备位: 肩抬与屈肘**同时**进行(用户要求"往上抬, 同时以
    肘为支点弯曲")。两端都是小角度, 关节空间直线不会出现翻转, 只做限步平滑。
    """
    return _waypoints(q0, q1, step)


# ---------------------------------------------------------------- 抓取位姿
def palm_target(obj_R, obj_t, approach_local, curl_local, offset_local):
    """由"物体位姿 + 抓取几何"反算手掌目标位姿。

    obj_R/obj_t: 物体(或被抓住的那个特征点)的位姿
    approach_local / curl_local: 在**物体坐标系**下的手指朝向 / 掌心法向
    offset_local: 物体特征点在手坐标系中的位置
    """
    a = obj_R @ np.asarray(approach_local, float)
    c = obj_R @ np.asarray(curl_local, float)
    a = a / np.linalg.norm(a)
    c = c / np.linalg.norm(c)
    z = -a                                   # 手指朝 -z
    y = c - np.dot(c, z) * z
    y = y / np.linalg.norm(y)
    x = np.cross(y, z)
    R = np.column_stack([x, y, z])
    t = np.asarray(obj_t, float) - R @ np.asarray(offset_local, float)
    return frame(R, t)


def lamp(name):
    """调试用: 打印位姿"""
    R, t = name[:3, :3], name[:3, 3]
    rpy = mat_to_rpy(R)
    return ('t=(%+.3f %+.3f %+.3f) rpy=(%+.0f %+.0f %+.0f)'
            % (t[0], t[1], t[2], math.degrees(rpy[0]),
               math.degrees(rpy[1]), math.degrees(rpy[2])))
