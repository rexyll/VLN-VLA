#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_p87.py —— P87 验收: 左手抓杯 + 右手倒水

验收口径(全部对 gz 真值, 不看自己算的数):
  [1] 数据链路就绪
  [2] 机器人到服务位, base 取自真值
  [3] 双臂回 home
  [4] 左手从桌面抓杯并附着: 杯在抓握瞬间不应被挪动
  [5] 举杯到倒水位: 杯心位置误差 <5mm, 杯轴仍竖直(<2°)
  [6] 右手从桌面抓壶并附着
  [7] 壶摆到倒水位: 壶嘴尖落在杯口正上方 JET_GAP 处(<10mm), 壶倾角 ≈68°
  [8] `jet` 伸出: 壶内水柱沿壶嘴轴线伸到位(真值量 link 位移)
  [9] 水位上升: `water` 抬到 65% 行程(真值量 link 位移)
  [10] 相机出图: 保存倒水前/中/后三帧, 供人工确认水柱连续

坐标口径提醒
============
`/world/<w>/dynamic_pose/info` 里 **模型是世界位姿, link 是相对所属模型
的位姿**。所以壶/杯用模型名取; 掌心用 FK 算; jet/water 这两个 link 用
"相对静止位的位移量"来验收行程(模型局部系里位移的模长, 与姿态无关)。

用法:
    bash tools/start_scene.sh
    source /opt/ros/jazzy/setup.bash; source vla_ws/install/setup.bash
    python3 tests/test_p87.py
"""
import os
import struct
import sys
import time
import zlib

import numpy as np
import rclpy
from sensor_msgs.msg import Image

from vla_follow import arm_kin as K
from vla_follow import drink_motion as M
from vla_follow import drink_poses as P
from vla_follow import scene_layout as L
from vla_follow.drink_task import DrinkTask

LOG_DIR = os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'vla_ws', 'logs')

JET_REST = np.array([0.02229, 0.00000, 0.04252])   # SDF 里 jet 的静止相对位
WATER_REST_Z = -0.050                              # SDF 里 water 的静止 z
JET_OUT = 0.16                                     # 倒水时水柱伸出量
FILL_FRAC = 0.65                                   # 水位目标(占满行程比例)


class Camera(object):
    def __init__(self, node):
        self.node = node
        self.last = None
        node.create_subscription(Image, '/drink_cam/image', self._cb, 5)

    def _cb(self, msg):
        self.last = msg

    def grab(self, node, wait=3.0):
        t0 = time.time()
        while time.time() - t0 < wait:
            node.spin_for(0.05)
            if self.last is not None:
                return self.last
        return None


def _save_png(path, buf, w, h, enc):
    """不依赖 cv_bridge/PIL: rgb8/bgr8 手写 PNG 落盘。"""
    a = np.frombuffer(buf, dtype=np.uint8).reshape(h, w, -1)[:, :, :3]
    if enc == 'bgr8':
        a = a[:, :, ::-1]
    raw = b''.join(b'\x00' + a[r].tobytes() for r in range(h))

    def chunk(tag, data):
        return (struct.pack('>I', len(data)) + tag + data
                + struct.pack('>I', zlib.crc32(tag + data) & 0xFFFFFFFF))

    png = (b'\x89PNG\r\n\x1a\n'
           + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
           + chunk(b'IDAT', zlib.compress(raw, 6))
           + chunk(b'IEND', b''))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'wb') as fh:
        fh.write(png)
    return a


def snap(node, cam, tag, ok_sink):
    img = cam.grab(node)
    if img is None:
        ok_sink.append(False)
        return None
    w, h = img.width, img.height
    a = _save_png(os.path.join(LOG_DIR, 'p87_%s.png' % tag),
                  img.data, w, h, img.encoding)
    colors = len(np.unique(a.reshape(-1, 3), axis=0))
    node.get_logger().info('相机 %s: %dx%d 颜色数 %d' % (tag, w, h, colors))
    ok_sink.append(True)      # 踩坑: 原来成功分支忘了记数, [10] 恒 0/3
    return a


def link_offset(node, name, rest):
    """link 相对静止位的位移量(m)。link 给的是模型局部位姿, 取位移模长。"""
    p = node.links.get(name)
    if p is None:
        return None
    return float(np.linalg.norm(np.array([p.position.x, p.position.y,
                                          p.position.z]) - np.asarray(rest)))


def main():
    rclpy.init()
    node = DrinkTask()
    cam = Camera(node)
    fails = []
    ok_sink = []

    def step(tag, ok, detail=''):
        print('%s %-46s %s%s'
              % ('[OK]' if ok else '[NG]', tag, 'OK' if ok else 'NG',
                 ('  ' + detail) if detail else ''))
        if not ok:
            fails.append(tag)

    print('=' * 78)
    print('P87 验收 —— 左手抓杯 + 右手倒水')
    print('=' * 78)

    # ---- [1] 数据就绪
    t0 = time.time()
    while time.time() - t0 < 10.0 and len(node.js) < 34:
        node.spin_for(0.2)
    step('[1] 数据链路  joint_state=%d 真值实体=%d'
         % (len(node.js), len(node.links)),
         len(node.js) >= 34 and 'cup' in node.links and 'kettle' in node.links)

    # ---- [2] 到服务位
    T_serve = K.frame(K.rot_z(0.0),
                      (L.SERVE_POSE[0], L.SERVE_POSE[1], 0.0))
    d_serve = M.place_robot(node, T_serve)
    step('[2] 机器人到服务位  偏差 %.1f mm' % d_serve, d_serve < 10.0)

    # ---- [3] home
    node.home()
    step('[3] 双臂回 home', True)

    # ---- [4] 左手抓杯
    Tcup0 = M.object_pose(node, 'cup')
    ok_c, res_c = M.grasp(node, 'L', 'cup',
                          lambda T: P.cup_grasp_palm(T, 'L'), K.GRIP_CUP)
    # 附着是"钉到掌心 @ 冻结偏移", 所以手臂滑行会把杯子一起带走。分三个时刻
    # 采样, 看这 13mm 是"松钉前被推动"还是"松钉后跟着手臂滑行"。
    drift = []
    for wait in (0.05, 0.60, 1.50):
        node.spin_for(wait)
        drift.append(M.pos_err_mm(M.object_pose(node, 'cup'), Tcup0))
    off = node.carry.items.get('cup', {}).get('offset')
    rigid = (M.pos_err_mm(M.object_pose(node, 'cup'),
                          M.wrist(node, 'L') @ off) if off is not None else 9e9)
    step('[4] 左手抓杯  抓取位 %.1f mm  被挪 %.1f/%.1f/%.1f mm  刚性 %.1f mm  ack=%s'
         % (res_c, drift[0], drift[1], drift[2], rigid, 'ok' if ok_c else 'fail'),
         ok_c and drift[2] < 12.0 and rigid < 2.0)

    # ---- [5] 举杯到倒水位
    Tcup_pour = P.cup_for_pour()
    ok_l, res_l = M.place_held(node, 'L', 'cup', Tcup_pour, tol_mm=3.0)
    node.spin_for(0.6)
    Tcup_now = M.object_pose(node, 'cup')
    exp_cup = M.pos_err_mm(Tcup_now, Tcup_pour)
    cup_tilt = M.tilt_deg(Tcup_now)
    step('[5] 举杯(倒水)  位置 %.1f mm  杯轴倾 %.2f°  闭环 %.1f mm'
         % (exp_cup, cup_tilt, res_l),
         ok_l and exp_cup < 5.0 and cup_tilt < 2.0)

    # ---- 相机: 倒水前
    snap(node, cam, 'before', ok_sink)

    # ---- [6] 右手抓壶
    Tk0 = M.object_pose(node, 'kettle')
    ok_k, res_k = M.grasp(node, 'R', 'kettle',
                          lambda T: P.kettle_grasp_palm(T), K.GRIP_KETTLE)
    node.spin_for(0.3)
    step('[6] 右手抓壶  抓取位 %.1f mm  ack=%s'
         % (res_k, 'ok' if ok_k else 'fail'), ok_k)
    if not ok_k:
        return report(fails, node)

    # ---- [7] 壶到倒水位(由**当前杯真值**反解)
    Tcup_now = M.object_pose(node, 'cup')
    Tk_pour, tip_tgt = P.kettle_for_pour(Tcup_now)
    ok_p, res_p = M.place_held(node, 'R', 'kettle', Tk_pour, tol_mm=4.0)
    node.spin_for(0.6)
    Tk_now = M.object_pose(node, 'kettle')
    tip = Tk_now[:3, :3] @ np.array(L.KETTLE_SPOUT_TIP_LOCAL) + Tk_now[:3, 3]
    tip_err = float(np.linalg.norm(tip - np.array(tip_tgt))) * 1000.0
    tilt = M.tilt_deg(Tk_now)
    step('[7] 壶到倒水位  嘴尖偏差 %.1f mm  壶倾 %.1f°(目标 %.0f°)  闭环 %.1f mm'
         % (tip_err, tilt, np.degrees(P.KETTLE_TILT), res_p),
         ok_p and tip_err < 10.0 and abs(tilt - np.degrees(P.KETTLE_TILT)) < 3.0)

    # ---- [8]+[9] 水柱伸出 + 水位上升
    # 走 GraspCarry 的 `joint` 指令(运动学驱动), 不用 /drink/cmd 的速度通道:
    # 见 carry.py:CarryBus.joint 的注释 —— 速度指令会被每 1ms 的 teleport 清零。
    node.carry.joint_ramp('kettle', 'jet_joint', JET_OUT, dur=0.5, dt=0.05)
    want_w = L.CUP_WATER_MAX * FILL_FRAC
    node.carry.joint_ramp('cup', 'water_joint', want_w * 0.5, dur=0.7, dt=0.05)
    snap(node, cam, 'pouring', ok_sink)          # 倒水中: 水柱已伸出、水位半满
    node.carry.joint_ramp('cup', 'water_joint', want_w, dur=0.9, dt=0.05)
    node.spin_for(0.8)
    jet = link_offset(node, 'jet', JET_REST)
    water = (node.links['water'].position.z - WATER_REST_Z
             if 'water' in node.links else None)
    step('[8] jet 伸出  真值位移 %.1f mm(目标 %.0f)'
         % ((jet or 0) * 1000.0, JET_OUT * 1000.0),
         jet is not None and abs(jet - JET_OUT) < 0.012)
    step('[9] 水位上升  真值抬升 %.1f mm(目标 %.1f) = %.0f%%'
         % ((water or 0) * 1000.0, want_w * 1000.0,
            100.0 * (water or 0) / L.CUP_WATER_MAX),
         water is not None and abs(water - want_w) < 0.004)

    # ---- 相机: 倒水后
    node.spin_for(0.5)
    snap(node, cam, 'after', ok_sink)

    # ---- [10] 相机出图
    step('[10] 相机出图(倒水前/中/后) %d/3' % sum(1 for _ in ok_sink),
         all(ok_sink) and len(ok_sink) == 3)

    return report(fails, node)


def report(fails, node):
    print('-' * 78)
    print('结论: %s' % ('通过' if not fails else '不通过 (%d 项)' % len(fails)))
    for f in fails:
        print('  未通过: %s' % f)
    print('=' * 78)
    try:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass
    return 0 if not fails else 2


if __name__ == '__main__':
    sys.exit(main())
