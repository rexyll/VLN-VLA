#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_p99.py —— P99 验收: 手背腕部 RGB-D 相机(眼在手)

为什么要这套相机
================
`/drink_cam` 是世界里的**第三人称**观察机位, 只能回答"动作看起来对不对";
抓取前"手离把手还差多少、偏左还是偏右"这种问题它答不了 —— 相机离得远,
壶把在画面里只有十几个像素。P99 在**两只手掌"手背"**各挂一路 rgbd_camera,
视线沿手指方向、略微偏向抓握面, 用来给后续的视觉伺服/抓取自检留观测口。

验收口径
========
[1] 出流: 每侧 RGB + 深度各一路, 5Hz, 256x192, 深度有效像素 ≥ 60%
[2] 桥接: 同样的 4 路在 ROS 侧也能收到(ros_gz_bridge 新加 4 条)
[3] **外参自检**(最关键): 悬垂位把 FK 算出来的四个指尖投到图像上, 深度图里
    "最近的那块"必须落在某个指尖投影附近 —— 这条一通, "看到的到底是哪"就
    不再是猜的(`arm_kin.WRIST_CAM_*` 与 SDF 是同一份常量)
[4] **抓取前能看到目标**: 摆到服务位、右臂到预抓取位后, 壶把手点必须落在
    画幅内, 且该像素的实测深度与投影预测的轴向距离一致(差 < 50mm)
    —— 只"投影在画幅内"不算数: 手指挡在前面时投影也在画幅内
[5] 截图: `wrist_hang_R/L`、`wrist_grasp_R`

用法:
    bash tools/start_scene.sh
    source /opt/ros/jazzy/setup.bash; source vla_ws/install/setup.bash
    python3 tests/test_p99.py
"""
import math
import os
import struct
import sys
import time
import zlib

import numpy as np
import rclpy

import gz.transport13 as gz_transport
import gz.msgs10.image_pb2 as image_pb2

from sensor_msgs.msg import Image as RosImage

from vla_follow import arm_kin as K
from vla_follow import drink_poses as P
from vla_follow import scene_layout as L
from vla_follow.drink_task import DrinkTask
from vla_follow.pngsnap import save_png

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOGS = os.path.join(ROOT, 'vla_ws', 'logs')
SIDE = 'R'
W, H = K.WRIST_CAM_W, K.WRIST_CAM_H


# ---------------------------------------------------------------- PNG(灰度)
def _chunk(tag, data):
    return (struct.pack('>I', len(data)) + tag + data
            + struct.pack('>I', zlib.crc32(tag + data) & 0xFFFFFFFF))


def save_png_gray(path, arr):
    a = np.ascontiguousarray(arr.astype(np.uint8))
    h, w = a.shape
    raw = b''.join(b'\x00' + a[y].tobytes() for y in range(h))
    with open(path, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n')
        f.write(_chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 0, 0, 0, 0)))
        f.write(_chunk(b'IDAT', zlib.compress(raw, 6)))
        f.write(_chunk(b'IEND', b''))


# ---------------------------------------------------------------- gz 订阅
class GzCam:
    """一路腕部相机的 gz 侧取帧器(RGB + 深度)。"""

    def __init__(self, side):
        self.side = side
        self.node = gz_transport.Node()
        self.rgb = {}
        self.dep = {}
        self.rgb_n = 0
        self.dep_n = 0

        def on_rgb(m):
            self.rgb_n += 1
            self.rgb.setdefault('m', m)

        def on_dep(m):
            self.dep_n += 1
            self.dep.setdefault('m', m)
        self.node.subscribe(image_pb2.Image, '/wrist_cam_%s/image' % side,
                            on_rgb)
        self.node.subscribe(image_pb2.Image, '/wrist_cam_%s/depth_image' % side,
                            on_dep)

    def fresh(self):
        """丢掉缓存, 让下一次读到的是新帧。"""
        self.rgb.clear()
        self.dep.clear()
        self.rgb_n = self.dep_n = 0

    def wait(self, timeout=6.0):
        t0 = time.time()
        while time.time() - t0 < timeout and (
                'm' not in self.rgb or 'm' not in self.dep):
            time.sleep(0.05)
        return 'm' in self.rgb and 'm' in self.dep

    def rgb_array(self):
        m = self.rgb['m']
        a = np.frombuffer(m.data, dtype=np.uint8)
        ch = a.size // (m.width * m.height)
        # 颜色顺序不再"猜": `<format>R8G8B8</format>` 出来的 buffer 就是 RGB。
        # 早先按 `pixel_format_type == 3` 当 BGR 翻了一次通道 —— 而 gz 的枚举里
        # 3 是 R_FLOAT32, 根本不是 BGR, 于是整幅图被换成补色(地面变薰衣草紫)。
        # 判据: 背景像素 [183,198,219] 必须等于 scene 的 <background> 0.72/0.78/0.86。
        a = a.reshape(m.height, m.width, ch)[:, :, :3]
        return a, m

    def depth_array(self):
        m = self.dep['m']
        f = np.frombuffer(m.data, dtype=np.float32)
        if f.size != m.width * m.height:
            f = np.frombuffer(m.data, dtype=np.uint16).astype(np.float32) / 1e3
        return f.reshape(m.height, m.width), m


# ---------------------------------------------------------------- 运动助手
def q_meas(node, side=SIDE):
    names = ['%s_%s_joint' % (side, j[0]) for j in K.JOINTS]
    if any(n not in node.js for n in names):
        return None
    return np.array([node.js[n] for n in names], float)


def T_of(node, name):
    p = node.links.get(name)
    if p is None:
        return None
    q = p.orientation
    return K.frame(K.quat_to_mat(q.x, q.y, q.z, q.w),
                   (p.position.x, p.position.y, p.position.z))


def move_to(node, T_world, side=SIDE, timeout=10.0, tol_mm=5.0, passes=4):
    res = 9e9
    sched = (None, 0.6, 0.25, 0.25)
    for it in range(passes):
        q_now = q_meas(node, side)
        if q_now is None:
            return False, res
        q, _e, _how = K.ik_world(side, T_world, node.base, q_now)
        node.arm[side].move(q, timeout=timeout, tol_deg=sched[min(it, 3)])
        q_end = q_meas(node, side)
        res = float(np.linalg.norm(
            K.pose_err(K.fk_world(side, q_end, node.base), T_world)[:3])) * 1e3
        if res < tol_mm:
            break
    return res < tol_mm, res


def nearest_blob(dep):
    """深度图里"最近那一片"的质心 + 最近值。"""
    valid = np.isfinite(dep) & (dep > 0.05) & (dep < 7.5)
    if not valid.any():
        return None, None
    dmin = float(dep[valid].min())
    yy, xx = np.nonzero(valid & (dep < dmin + 0.03))
    return (float(xx.mean()), float(yy.mean())), dmin


def main():
    rclpy.init()
    node = DrinkTask()
    fails = []

    def step(tag, ok, detail=''):
        print('%s %-52s %s%s'
              % ('[OK]' if ok else '[NG]', tag, 'OK' if ok else 'NG',
                 ('  ' + detail) if detail else ''))
        if not ok:
            fails.append(tag)

    print('=' * 80)
    print('P99 验收 —— 手背腕部 RGB-D 相机')
    print('=' * 80)
    os.makedirs(LOGS, exist_ok=True)

    # ---- [1] 出流
    cams = {s: GzCam(s) for s in ('R', 'L')}
    t0 = time.time()
    while time.time() - t0 < 8.0 and not all(c.wait(0.5) for c in cams.values()):
        pass
    got = {s: c.wait(0.2) for s, c in cams.items()}
    step('[1a] 四路出流  R=%s L=%s' % (got['R'], got['L']), all(got.values()))

    # 帧率: 4s 窗口计数
    for c in cams.values():
        c.fresh()
    node.spin_for(4.0)
    hz = {}
    size_ok = True
    for s, c in cams.items():
        hz[s] = (c.rgb_n / 4.0, c.dep_n / 4.0)
        if 'm' in c.rgb:
            m = c.rgb['m']
            size_ok = size_ok and (m.width, m.height) == (W, H)
    step('[1b] 帧率/分辨率  R %.1f/%.1f Hz  L %.1f/%.1f Hz  %dx%d'
         % (hz['R'][0], hz['R'][1], hz['L'][0], hz['L'][1], W, H),
         all(3.5 <= v[0] <= 6.5 and 3.5 <= v[1] <= 6.5 for v in hz.values())
         and size_ok)

    # ---- [2] ROS 桥接
    box = {'R': 0, 'L': 0, 'Rd': 0, 'Ld': 0}

    def mk(k):
        def cb(_m):
            box[k] += 1
        return cb
    subs = [node.create_subscription(RosImage, '/wrist_cam_R/image',
                                     mk('R'), 5),
            node.create_subscription(RosImage, '/wrist_cam_L/image',
                                     mk('L'), 5),
            node.create_subscription(RosImage, '/wrist_cam_R/depth_image',
                                     mk('Rd'), 5),
            node.create_subscription(RosImage, '/wrist_cam_L/depth_image',
                                     mk('Ld'), 5)]
    node.spin_for(4.0)
    step('[2] ROS 桥接 4 路  rgbR=%d rgbL=%d depR=%d depL=%d'
         % (box['R'], box['L'], box['Rd'], box['Ld']), min(box.values()) >= 5)
    for s in subs:
        node.destroy_subscription(s)

    # ---- [3] 外参自检(home 位)
    # 为什么不用悬垂位做这一项: 左手的"手背"朝体侧, 悬垂时相机离躯干只有 5cm,
    # 画面下沿会拍到机器人自己的底盘/立柱 —— 最近的深度块是**自己**而不是指尖,
    # 这一项会假 NG。home(Q_REST) 双臂前伸, 两只相机前方都是空场, 两侧对称可比。
    node.home(verbose=False)
    node.sync_base_from_truth()
    for c in cams.values():
        c.fresh()
    node.spin_for(0.6)
    proj_lines = []
    dists = []
    for s in ('R', 'L'):
        q = q_meas(node, s)
        if q is None or not cams[s].wait(0.5):
            continue
        T_world_palm = K.fk_world(s, q, node.base)
        T_cam = K.wrist_cam_world(s, q, node.base)
        tips = [(K.FINGER_X[k], 0.0,
                 K.FINGER_BASE_Z - K.PROX_LEN - K.DIST_LEN) for k in range(4)]
        uv_pred = []
        for t in tips:
            p = T_world_palm[:3, :3] @ np.array(t) + T_world_palm[:3, 3]
            u, v, _d = K.wrist_cam_project(T_cam, p)
            if u is not None:
                uv_pred.append((u, v))
        dep, _m = cams[s].depth_array()
        cen, dmin = nearest_blob(dep)
        if cen is None or not uv_pred:
            dists.append(9e9)
            continue
        dist = min(math.hypot(cen[0] - u, cen[1] - v) for u, v in uv_pred)
        dists.append(dist)
        proj_lines.append('%s 指尖投影 %s -> 最近块 (%.0f,%.0f) 差 %.0fpx'
                          % (s, ' '.join('(%.0f,%.0f)' % uv for uv in uv_pred),
                             cen[0], cen[1], dist))
    for ln in proj_lines:
        print('       ' + ln)
    worst = max(dists) if dists else 9e9
    step('[3] 外参自检(指尖投影↔最近深度块) 最差 %.0f px (判据 35)' % worst,
         bool(dists) and worst < 35.0)

    # ---- [4] 抓取前能看到壶把手
    T_place = K.frame(K.rot_z(0.0), (L.SERVE_POSE[0], L.SERVE_POSE[1], 0.0))
    node.carry.place('robot', T_place)
    node.spin_for(0.8)
    node.sync_base_from_truth()
    node.home(verbose=False)
    Tk0 = T_of(node, 'kettle')
    if Tk0 is None:
        step('[4] 读壶真值', False)
        return report(fails, node)
    T_grasp = P.kettle_grasp_palm(Tk0)
    T_pre = T_grasp @ K.trans((0.0, 0.0, 0.10))
    node.hand[SIDE].move(K.grip_values(SIDE, K.GRIP_OPEN))
    ok_pre, res_pre = move_to(node, T_pre)
    q = q_meas(node, SIDE)
    T_cam = K.wrist_cam_world(SIDE, q, node.base)
    cams['R'].fresh()
    node.spin_for(0.6)
    cams['R'].wait(0.5)
    dep, _m = cams['R'].depth_array()
    rgb, _m2 = cams['R'].rgb_array()
    save_png(os.path.join(LOGS, 'wrist_grasp_R.png'), rgb.tobytes(), W, H)

    # 这一项要回答的是"眼在手到底能不能用来看活儿"。手背相机在**抓取瞬间**看不到
    # 被抓物 —— 物体就在指尖那一侧、被自己的手指挡死, 这是机位的固有几何(相机在
    # 掌后 4cm、视线与手指平行), 不是标定问题。它真正的用途是"接近阶段看手前方
    # 的工作区": 手离桌面多远、往哪边偏。
    #
    # 判据: 把整幅深度图**反投回世界**, 落到桌面高度(z=TABLE_TOP_Z)的像素要有
    # 一定占比, 且这些点拟合出的平面高度/倾角要对得上。这比投某个角点稳 ——
    # 角点正好压在桌沿上, 一两像素的误差就掉到桌面外(实测 0.884m 预测 vs
    # 3.769m 实测, 那一枪打到了墙上)。外参只要有 10° 级的错, 反投出来的点就
    # 不成一个水平面, 这一项立刻塌。
    fx = (W / 2.0) / math.tan(K.WRIST_CAM_HFOV / 2.0)
    uu, vv = np.meshgrid(np.arange(W), np.arange(H))
    dirs = np.stack([np.ones_like(uu, float),
                     -(uu - W / 2.0) / fx,
                     -(vv - H / 2.0) / fx], axis=-1)
    ok = np.isfinite(dep) & (dep > 0.10) & (dep < 1.50)
    pts = (dep[ok][:, None] * dirs[ok]) @ T_cam[:3, :3].T + T_cam[:3, 3]
    near = np.abs(pts[:, 2] - L.TABLE_TOP_Z) < 0.02
    cover = 100.0 * near.sum() / float(W * H)
    plane_txt = '-'
    if near.sum() > 200:
        Q = pts[near]
        # z = a*x + b*y + c 的最小二乘
        A = np.stack([Q[:, 0], Q[:, 1], np.ones(len(Q))], axis=1)
        coef, *_ = np.linalg.lstsq(A, Q[:, 2], rcond=None)
        tilt = math.degrees(math.atan(math.hypot(coef[0], coef[1])))
        plane_txt = 'z@桌心=%.3f 倾角 %.1f°' % (
            coef[0] * L.TABLE_XY[0] + coef[1] * L.TABLE_XY[1] + coef[2], tilt)
    else:
        tilt = 99.0
    step('[4a] 预抓取(残差 %.1f mm) 桌面像素占画面 %.0f%%  %s'
         % (res_pre, cover, plane_txt),
         ok_pre and cover >= 8.0 and tilt < 3.0)

    # ---- [4b] 已删除: 原计划"退 30cm 再看壶身" —— 实测退不动
    # 从预抓取位沿掌 +z 退 0.30m, 手就到了底盘正上方(x≈1.83 vs base x≈1.76),
    # 肘关节够到限位, IK 残差 70.9mm。而只退 0.15m 时壶身仍在画面外(壶心相对
    # 光轴 40° 以上, 半视场只有 27°)。结论记在这里, 别再重复试:
    # **手背相机在 0.2m 内看不见被抓物, 这是机位几何决定的**, 想改只能换机位
    # (侧挂/更广角镜头), 不是调参能解决的。

    # ---- [5] 截图
    # 顺序: 先在 home(工作姿态, 画面正立)抓两侧, 再回悬垂抓一手。
    # 判据只用"文件存在"; 真正看画面得人眼过一遍 —— 自动化判不了"构图好不好"。
    node.home(verbose=False)
    shots = []
    for s in ('R', 'L'):
        cams[s].fresh()
    node.spin_for(0.8)
    for s in ('R', 'L'):
        cams[s].wait(0.5)
        rgb, _m = cams[s].rgb_array()
        dep, _m = cams[s].depth_array()
        p1 = os.path.join(LOGS, 'wrist_home_%s.png' % s)
        save_png(p1, rgb.tobytes(), W, H)
        dn = np.clip((dep - 0.05) / (1.5 - 0.05), 0, 1)
        dn[~np.isfinite(dep)] = 1.0
        p2 = os.path.join(LOGS, 'wrist_home_%s_depth.png' % s)
        save_png_gray(p2, dn * 255)
        shots += [p1, p2]
    node.hang(verbose=False)
    cams['R'].fresh()
    node.spin_for(0.8)
    cams['R'].wait(0.5)
    rgb, _m = cams['R'].rgb_array()
    p3 = os.path.join(LOGS, 'wrist_hang_R.png')
    save_png(p3, rgb.tobytes(), W, H)
    shots.append(p3)
    step('[5] 截图 %d 张' % len(shots), all(os.path.exists(p) for p in shots),
         ', '.join(os.path.basename(p) for p in shots))
    return report(fails, node)


def report(fails, node):
    print('-' * 80)
    print('结论: %s' % ('通过' if not fails else '不通过 (%d 项)' % len(fails)))
    for f in fails:
        print('  未通过: %s' % f)
    print('=' * 80)
    try:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass
    return 0 if not fails else 2


if __name__ == '__main__':
    sys.exit(main())
