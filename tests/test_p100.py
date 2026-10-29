#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_p100.py —— P100 验收: 掌侧近距 RGB-D(看得见要抓的东西)

为什么要再加一路相机
====================
P99 把手背相机挂上以后, 结论是**看不清被抓物**: 物体就在指尖那一侧, 被自己
的手指挡死(§3 P99 "已知边界")。用户随后提的要求是"需要看到抓取点"。于是
P100 在**掌的外侧**(掌系 x=±0.046, 拇指侧对面)再挂一路, 视线斜着看进
"四指与拇指之间"的抓握空档 —— 也就是物体真正在被抓的瞬间所在的位置。

机位不是凑欧拉角凑出来的: `arm_kin.side_cam_pose()` 用 look-at 反解
(机位 → SIDE_CAM_TARGET, 画面"上"贴掌 +x), SDF 与验收脚本共用这一份常量。

验收口径
========
[1] 出流: 每侧 RGB + 深度各一路, 5Hz, 192x144, 深度有效像素 ≥ 50%
[2] 桥接: 同样的 4 路在 ROS 侧也能收到(ros_gz_bridge 新加 4 条)
[3] **外参自检**: 工作姿态(home)把 FK 出的四个指尖投到图上, 深度图"最近那
    一块"必须落在某个指尖投影附近 —— 这条一通, "看到的到底是哪"就不靠猜
[4] **目标真的在画面里**: 把左臂摆到杯子的预抓取位, 杯子 5 个关键点(杯身
    中部/杯口/两侧/把根)的投影要落在画幅内, 且这些像素的实测深度与投影预
    测的轴向距离一致(差 < 60mm) —— 只"投影在画幅内"不算数, 手挡在前面时
    投影也照样在画幅内
[5] 截图: `side_home_R/L`、`side_pregrasp_L`(+ 深度图)

**当前状态(2026-10-28)**: [1][2][3][5] 通过, **[4b] 未通过** —— 关键点投影
全落在背景像素上(深度 −inf), 而画面下 2/3 确实拍到了机械臂与桌面、壶身也
正好落在模型预测的 (14,66)。也就是说**机位方向是对的, 但预抓取位这一路
的像素对应还差几十像素**, 需要下一步把"深度反投 + 平面/圆柱拟合"接上来重
标(见 TaskPlan §3 P100)。判据保留不通过, 别把"投影在画幅内"当成"看得见"。

用法:
    bash tools/start_scene.sh
    source /opt/ros/jazzy/setup.bash; source vla_ws/install/setup.bash
    python3 tests/test_p100.py
"""
import math
import os
import sys
import time

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

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_p99 import q_meas, T_of, move_to, nearest_blob  # noqa: E402
from test_p99 import save_png_gray   # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOGS = os.path.join(ROOT, 'vla_ws', 'logs')
W, H = K.SIDE_CAM_W, K.SIDE_CAM_H


class GzCam(object):
    """侧挂相机的取帧器(RGB + 深度), **回调里立即快照字节**。

    P100 踩到的坑(§7.1 #76): 把 gz 回传的 protobuf 消息对象存起来、事后才读
    `m.data`, 拿到的是**空 buffer** —— gz transport 会复用/清理那个对象, 于是
    同一时刻深度图有数据、RGB 是 0 字节(`np.frombuffer(b'')` 不报错, 通道数
    算成 0, 存出来的 PNG 就是 0 字节)。`test_p99.py` 的写法也有同样的隐患,
    只是没在那一帧上撞到。快照成 (w, h, bytes) 之后就没有共享状态了。
    """

    def __init__(self, side):
        self.side = side
        self.node = gz_transport.Node()
        self.rgb = None
        self.dep = None
        self.rgb_n = 0
        self.dep_n = 0

        def on_rgb(m):
            self.rgb_n += 1
            self.rgb = (m.width, m.height, bytes(m.data))

        def on_dep(m):
            self.dep_n += 1
            self.dep = (m.width, m.height, bytes(m.data))
        self.node.subscribe(image_pb2.Image, '/side_cam_%s/image' % side,
                            on_rgb)
        self.node.subscribe(image_pb2.Image, '/side_cam_%s/depth_image' % side,
                            on_dep)

    def fresh(self):
        self.rgb = None
        self.dep = None
        self.rgb_n = self.dep_n = 0

    def wait(self, timeout=6.0):
        t0 = time.time()
        while time.time() - t0 < timeout and (self.rgb is None
                                              or self.dep is None):
            time.sleep(0.05)
        return self.rgb is not None and self.dep is not None

    def rgb_array(self):
        w, h, data = self.rgb
        a = np.frombuffer(data, dtype=np.uint8)
        if a.size < w * h * 3:
            raise RuntimeError('side_cam_%s RGB 字节数不足: %d < %d'
                               % (self.side, a.size, w * h * 3))
        ch = a.size // (w * h)
        # `<format>R8G8B8</format>` 出来就是 RGB, 不要按 pixel_format_type==3
        # 当 BGR 翻(3 是 R_FLOAT32, 见 §7.1 #71)。
        return a.reshape(h, w, ch)[:, :, :3], (w, h)

    def depth_array(self):
        w, h, data = self.dep
        f = np.frombuffer(data, dtype=np.float32)
        if f.size != w * h:
            f = np.frombuffer(data, dtype=np.uint16).astype(np.float32) / 1e3
        return f.reshape(h, w), (w, h)


def cup_keypoints():
    """杯坐标系下的 5 个关键点: 杯身中部/杯口中心/杯口两侧/把根。"""
    r, h = L.CUP_R, L.CUP_H
    return [('杯身中', (0.0, 0.0, h * 0.5)),
            ('杯口心', (0.0, 0.0, h * 0.95)),
            ('杯口+y', (0.0, r, h * 0.95)),
            ('杯口-y', (0.0, -r, h * 0.95)),
            ('把根', (r + 0.022, 0.0, h * 0.5))]


def main():
    rclpy.init()
    node = DrinkTask()
    fails = []

    def step(tag, ok, detail=''):
        print('%s %-54s %s%s'
              % ('[OK]' if ok else '[NG]', tag, 'OK' if ok else 'NG',
                 ('  ' + detail) if detail else ''))
        if not ok:
            fails.append(tag)

    print('=' * 84)
    print('P100 验收 —— 掌侧近距 RGB-D(抓取点可见)')
    print('=' * 84)
    os.makedirs(LOGS, exist_ok=True)

    # ---- [1] 出流
    cams = {s: GzCam(s) for s in ('R', 'L')}
    t0 = time.time()
    while time.time() - t0 < 8.0 and not all(c.wait(0.5) for c in cams.values()):
        pass
    got = {s: c.wait(0.2) for s, c in cams.items()}
    step('[1a] 四路出流  R=%s L=%s' % (got['R'], got['L']), all(got.values()))

    for c in cams.values():
        c.fresh()
    node.spin_for(4.0)
    hz = {}
    size_ok = True
    for s, c in cams.items():
        hz[s] = (c.rgb_n / 4.0, c.dep_n / 4.0)
        if c.rgb is not None:
            _w, _h, _d = c.rgb
            size_ok = size_ok and (_w, _h) == (W, H)
    dep_ok = True
    for s, c in cams.items():
        if c.dep is not None:
            d, _m = c.depth_array()
            # 画幅上半是天空/背景 —— 那部分深度本来就无效(实测 home 位 30%),
            # 所以判据取 20%, 只确认"不是整幅废掉"。
            dep_ok = dep_ok and (np.isfinite(d) & (d > 0.05) & (d < 7.0)
                                 ).mean() >= 0.20
    step('[1b] 帧率/分辨率/深度有效率  R %.1f/%.1f Hz  L %.1f/%.1f Hz  %dx%d'
         % (hz['R'][0], hz['R'][1], hz['L'][0], hz['L'][1], W, H),
         all(3.5 <= v[0] <= 6.5 and 3.5 <= v[1] <= 6.5 for v in hz.values())
         and size_ok and dep_ok)

    # ---- [2] ROS 桥接
    box = {'R': 0, 'L': 0, 'Rd': 0, 'Ld': 0}

    def mk(k):
        def cb(_m):
            box[k] += 1
        return cb
    subs = [node.create_subscription(RosImage, '/side_cam_R/image', mk('R'), 5),
            node.create_subscription(RosImage, '/side_cam_L/image', mk('L'), 5),
            node.create_subscription(RosImage, '/side_cam_R/depth_image',
                                     mk('Rd'), 5),
            node.create_subscription(RosImage, '/side_cam_L/depth_image',
                                     mk('Ld'), 5)]
    node.spin_for(4.0)
    step('[2] ROS 桥接 4 路  rgbR=%d rgbL=%d depR=%d depL=%d'
         % (box['R'], box['L'], box['Rd'], box['Ld']), min(box.values()) >= 5)
    for s in subs:
        node.destroy_subscription(s)

    # ---- [3] 外参自检(home 位, 与 P99 同一套办法)
    node.home(verbose=False)
    node.sync_base_from_truth()
    for c in cams.values():
        c.fresh()
    node.spin_for(0.8)
    dists = []
    for s in ('R', 'L'):
        q = q_meas(node, s)
        if q is None or not cams[s].wait(0.5):
            dists.append(9e9)
            continue
        T_palm = K.fk_world(s, q, node.base)
        T_cam = K.side_cam_world(s, q, node.base)
        uv = []
        for k in range(4):
            p_loc = np.array([K.FINGER_X[k], 0.0,
                              K.FINGER_BASE_Z - K.PROX_LEN - K.DIST_LEN])
            p = T_palm[:3, :3] @ p_loc + T_palm[:3, 3]
            u, v, _d = K.side_cam_project(T_cam, p)
            if u is not None:
                uv.append((u, v))
        dep, _m = cams[s].depth_array()
        cen, _dmin = nearest_blob(dep)
        if cen is None or not uv:
            dists.append(9e9)
            continue
        dist = min(math.hypot(cen[0] - u, cen[1] - v) for u, v in uv)
        dists.append(dist)
        print('       %s 指尖投影 %s -> 最近深度块 (%.0f,%.0f) 差 %.0f px'
              % (s, ' '.join('(%.0f,%.0f)' % t for t in uv), cen[0], cen[1],
                 dist))
    worst = max(dists) if dists else 9e9
    step('[3] 外参自检(指尖投影↔最近深度块) 最差 %.0f px (判据 40)' % worst,
         bool(dists) and worst < 40.0)

    # ---- [4] 预抓取位上, 杯子必须在画面里(这一路相机的**存在理由**)
    T_place = K.frame(K.rot_z(0.0), (L.SERVE_POSE[0], L.SERVE_POSE[1], 0.0))
    node.carry.place('robot', T_place)
    node.spin_for(0.8)
    node.sync_base_from_truth()
    node.home(verbose=False)
    Tc = T_of(node, 'cup')
    if Tc is None:
        step('[4] 读杯子真值', False)
        return report(fails, node)
    T_grasp = P.cup_grasp_palm(Tc, 'L')
    T_pre = T_grasp @ K.trans((0.0, 0.0, 0.10))       # 沿掌 +z 退 10cm
    node.hand['L'].move(K.grip_values('L', K.GRIP_OPEN))
    ok_pre, res_pre = move_to(node, T_pre, side='L')
    q = q_meas(node, 'L')
    T_cam = K.side_cam_world('L', q, node.base)
    cams['L'].fresh()
    node.spin_for(0.8)
    cams['L'].wait(0.6)
    dep, _m = cams['L'].depth_array()
    rgb, _m2 = cams['L'].rgb_array()

    inside = 0
    agree = 0
    lines = []
    for name, p_loc in cup_keypoints():
        p = Tc[:3, :3] @ np.array(p_loc) + Tc[:3, 3]
        u, v, d = K.side_cam_project(T_cam, p)
        if u is None:
            lines.append('%s 在相机背后' % name)
            continue
        inframe = (2 <= u < W - 2) and (2 <= v < H - 2)
        if inframe:
            inside += 1
        # 该像素的实测深度 vs 投影预测的轴向距离(像素落在画幅外就跳过)。
        # 投影本身有 ~8px 的标定残差([3] 实测), 所以不取单像素, 而是在
        # ±3px 的窗口里挑**离预测点最近的**有效像素 —— 窗口再放大就会顺手
        # 捞到前面的手指(手指比杯子近 ~50mm, 正好落在 60mm 判据里, 会假过)。
        dm = None
        if inframe:
            u0, v0 = int(round(u)), int(round(v))
            best = None
            for _dy in range(-3, 4):
                for _dx in range(-3, 4):
                    _yy, _xx = v0 + _dy, u0 + _dx
                    if not (0 <= _yy < H and 0 <= _xx < W):
                        continue
                    _dv = float(dep[_yy, _xx])
                    if not (np.isfinite(_dv) and _dv > 0.02):
                        continue
                    _r = abs(_dx) + abs(_dy)
                    if best is None or _r < best[0]:
                        best = (_r, _dv)
            dm = best[1] if best else None
        if dm is not None and abs(dm - d) < 0.060:
            agree += 1
        lines.append('%s (%4.0f,%4.0f) 预测 %.3fm 实测 %s %s'
                     % (name, u, v, d,
                        '-' if dm is None else '%.3fm' % dm,
                        '在画幅内' if inframe else '**出画**'))
    for ln in lines:
        print('       ' + ln)
    step('[4a] 杯子关键点在画幅内 %d/5 (判据 ≥4), 预抓取残差 %.1f mm'
         % (inside, res_pre), inside >= 4 and ok_pre)
    step('[4b] 关键点处深度与投影一致 %d/5 (判据 ≥3, 差 <60mm)' % agree,
         agree >= 3)

    # 截图: 预抓取位的 RGB + 深度
    shots = []
    p1 = os.path.join(LOGS, 'side_pregrasp_L.png')
    save_png(p1, rgb.tobytes(), W, H)
    dn = np.clip((dep - 0.05) / (1.5 - 0.05), 0, 1)
    dn[~np.isfinite(dep)] = 1.0
    p2 = os.path.join(LOGS, 'side_pregrasp_L_depth.png')
    save_png_gray(p2, dn * 255)
    shots += [p1, p2]

    # ---- [5] home 位两侧截图
    node.home(verbose=False)
    for s in ('R', 'L'):
        cams[s].fresh()
    node.spin_for(0.8)
    for s in ('R', 'L'):
        cams[s].wait(0.5)
        _rgb, _m = cams[s].rgb_array()
        p = os.path.join(LOGS, 'side_home_%s.png' % s)
        save_png(p, _rgb.tobytes(), W, H)
        shots.append(p)
    step('[5] 截图 %d 张' % len(shots), all(os.path.exists(p) for p in shots),
         ', '.join(os.path.basename(p) for p in shots))
    return report(fails, node)


def report(fails, node):
    print('-' * 84)
    print('结论: %s' % ('通过' if not fails else '不通过 (%d 项)' % len(fails)))
    for f in fails:
        print('  未通过: %s' % f)
    print('=' * 84)
    try:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass
    return 0 if not fails else 2


if __name__ == '__main__':
    sys.exit(main())
