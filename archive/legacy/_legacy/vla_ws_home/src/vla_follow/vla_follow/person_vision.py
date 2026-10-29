#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""person_vision.py —— 4 向 RGB-D 相机全向识"人" + 快照式跟踪管线

初始化数据（人的外观先验, 内置常量）:
  - "人"主体为深色圆柱(裙摆/躯干), 颜色暗(V<70)且低饱和(S<100)
  - 斑点面积 60~22000 px, 宽高比 0.2~2.8; 深度∈[0.45,11.5]m
  - 场景内其余物体均为彩色/浅色且已关闭阴影 —— 暗色斑点唯一对应"人"
快照管线(P12):
  - 4 相机以 snapshot_rate_hz(默认2Hz)抓拍; 仅保存检出目标的帧(带
    相机/方位/距离标注的 JPG), 未检出帧不落盘(等效立即删除);
  - 滚动保留最近 snapshot_keep 张(默认10, ~/vla_demo/vision_config.json 可调);
  - 维护最近 N 条世界坐标历史, 发布 /perception/person_history 供
    follower 做去向推测与搜索导航。
输入: gz /depth_X/image + /depth_X/depth_image + dynamic_pose/info
输出: /perception/person  (10Hz 实时检测, JSON)
      /perception/person_history (2Hz, {"seq":[...], "lateral":bool})
"""
import json
import math
import os
import threading
import time

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import gz.transport13 as gz_transport
from gz.msgs10.image_pb2 import Image as GzImage
from gz.msgs10.pose_v_pb2 import Pose_V as GzPose_V

# ---- 人的外观先验(初始化数据) ----
DARK_V_MAX = 70
SAT_MAX = 100
AREA_MIN, AREA_MAX = 60, 22000
ASPECT_MIN, ASPECT_MAX = 0.2, 2.8
HFOV = 1.8
DEPTH_MIN = 0.45  # 过滤近端伪影与自视伪影(自身轮/后角成超大暗斑)
DEPTH_MAX = 11.5

CAMS = (('front', 0.0), ('left', 1.5707963),
        ('right', -1.5707963), ('back', 3.1415926))
CAM_CN = {'front': '前', 'left': '左', 'right': '右', 'back': '后'}

CONFIG_PATH = os.path.join(
    os.environ.get('VLA_HOME', '/home/earl/workspace/VLA'),
    'vla_demo', 'vision_config.json')
SNAP_DIR = os.path.join(
    os.environ.get('VLA_HOME', '/home/earl/workspace/VLA'),
    'vla_demo', 'snapshots')

DEFAULT_CFG = {
    'snapshot_keep': 10,
    'snapshot_rate_hz': 2.0,
    'track_dist_m': 1.0,        # 跟踪保持距离(P15)
    'track_dist_tol_m': 0.10,   # 距离带宽 ±10cm
    'recenter_rot_speed': 1.5,  # 对正旋转角速度 rad/s
    'approach_speed': 0.2,      # 靠近速度 m/s(保底; 上限随人速自适应)
    # PID 增益(P16): 距离环→线速度, 航向环→角速度
    'pid_kp_v': 0.6, 'pid_ki_v': 0.20, 'pid_kd_v': 0.08,
    'pid_kp_w': 2.2, 'pid_ki_w': 0.40, 'pid_kd_w': 0.30,
}


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def load_config():
    """读配置并回写缺失键(老配置文件自动补全新参数)"""
    cfg = dict(DEFAULT_CFG)
    try:
        with open(CONFIG_PATH, encoding='utf-8') as f:
            cfg.update(json.load(f))
    except Exception:
        pass
    try:
        with open(CONFIG_PATH, 'w', encoding='utf-8') as f:
            json.dump(cfg, f, ensure_ascii=False, indent=2)
    except Exception:
        pass
    return cfg


class PersonVision(Node):

    def __init__(self):
        super().__init__('person_vision')
        self.pub = self.create_publisher(String, '/perception/person', 10)
        self.pub_hist = self.create_publisher(String,
                                              '/perception/person_history', 10)
        self.cfg = load_config()
        os.makedirs(SNAP_DIR, exist_ok=True)
        self.get_logger().info(
            f'VISION 快照配置: 保留{self.cfg["snapshot_keep"]}张 @ '
            f'{self.cfg["snapshot_rate_hz"]}Hz (配置: {CONFIG_PATH})')
        self._gz = gz_transport.Node()
        self._lock = threading.Lock()
        self.rgb, self.dep = {}, {}
        self.rgb_t, self.dep_t = {}, {}
        self.rx, self.ry, self.ryaw = 0.0, 0.0, 0.0
        for name, _ in CAMS:
            self._gz.subscribe(GzImage, '/depth_' + name + '/image',
                               self._mk_rgb(name))
            self._gz.subscribe(GzImage, '/depth_' + name + '/depth_image',
                               self._mk_dep(name))
        self._gz.subscribe(GzPose_V,
                           '/world/follow_world/dynamic_pose/info',
                           self._pose_cb)
        self._det_n, self._frm_n = 0, 0
        self._last_log = 0.0
        self._prev_cam = None
        self._cam_streak = {}
        self._last_snap = 0.0
        self.hist = []            # 最近 N 条快照记录(世界系)
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            'VISION 4 向识人+快照管线就绪, 输出 /perception/person[_history]')

    def _mk_rgb(self, name):
        def cb(msg):
            h, w = msg.height, msg.width
            if h < 10 or w < 10:
                return
            n = len(msg.data)
            if n == w * h * 3:
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
            elif n == w * h * 4:
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 4)[:, :, :3]
            else:
                return
            with self._lock:
                self.rgb[name] = img
                self.rgb_t[name] = time.monotonic()
        return cb

    def _mk_dep(self, name):
        def cb(msg):
            h, w = msg.height, msg.width
            if h < 10 or w < 10 or len(msg.data) < w * h * 4:
                return
            d = np.frombuffer(msg.data[:w * h * 4], dtype='<f4').reshape(h, w)
            with self._lock:
                self.dep[name] = d
                self.dep_t[name] = time.monotonic()
        return cb

    def _pose_cb(self, msg):
        for p in msg.pose:
            if p.name == 'robot':
                q = p.orientation
                with self._lock:
                    self.rx = p.position.x
                    self.ry = p.position.y
                    self.ryaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                           1.0 - 2.0 * (q.y * q.y + q.x * q.x))
                return

    def _detect_cam(self, name, rgb=None, dep=None, want_bbox=False):
        """单相机检测: (detected, dist, bearing_cam, u, area[, bbox])"""
        with self._lock:
            rgb = self.rgb.get(name) if rgb is None else rgb
            dep = self.dep.get(name) if dep is None else dep
        if rgb is None or dep is None:
            return (False, -1.0, 0.0, -1, 0, None) if want_bbox \
                else (False, -1.0, 0.0, -1, 0)
        if rgb.shape[:2] != dep.shape:
            dep = cv2.resize(dep, (rgb.shape[1], rgb.shape[0]))
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, (0, 0, 0), (179, SAT_MAX, DARK_V_MAX))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                np.ones((5, 5), np.uint8))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        best = None
        for c in cnts:
            a = cv2.contourArea(c)
            if a < AREA_MIN:
                continue
            x, y, w, h = cv2.boundingRect(c)
            asp = w / max(h, 1)
            if not (ASPECT_MIN <= asp <= ASPECT_MAX):
                continue
            if best is None or a > best[0]:
                best = (a, x, y, w, h)
        if best is None:
            return (False, -1.0, 0.0, -1, 0, None) if want_bbox \
                else (False, -1.0, 0.0, -1, 0)
        a, x, y, w, h = best
        u = x + w / 2.0
        y0 = y + int(h * 0.15)
        y1 = y + int(h * 0.60)
        sub = dep[y0:max(y1, y0 + 1), x:x + w]
        valid = sub[np.isfinite(sub) & (sub > DEPTH_MIN) & (sub < DEPTH_MAX)]
        if valid.size < 10:
            return (False, -1.0, 0.0, int(u), int(a), None) if want_bbox \
                else (False, -1.0, 0.0, int(u), int(a))
        dist = float(np.quantile(valid, 0.30))
        img_w = rgb.shape[1]
        # 图像 u 向右增; 方位角正方向=左 → 用(中心-u)。
        # (P19: 此前用(u-中心)导致方位左右镜像, 机器人对正时反向旋转
        #  打转不收敛 —— 实测 +84°人在左, w=+1.5 左转, 方位反增至+112°)
        bearing_cam = (img_w / 2.0 - u) * HFOV / img_w
        if want_bbox:
            return True, dist, bearing_cam, int(u), int(a), (x, y, w, h)
        return True, dist, bearing_cam, int(u), int(a)

    # ---------- 快照管线 ----------
    def _snapshot(self, results, now):
        """2Hz: 保存检出目标的标注帧, 维护滚动窗口与历史"""
        keep = int(self.cfg['snapshot_keep'])
        with self._lock:
            rx, ry, ryaw = self.rx, self.ry, self.ryaw
        ts = time.strftime('%H%M%S')
        saved = []
        for name, (det, dist, bc, u, area) in results.items():
            if not det:
                continue  # 未检出目标: 不落盘(等效立即删除)
            with self._lock:
                rgb = self.rgb.get(name)
            if rgb is None:
                continue
            _, _, _, _, _, bbox = self._detect_cam(name, want_bbox=True)
            if bbox is None:
                continue
            bearing = wrap(dict(CAMS)[name] + bc)
            frame = rgb.copy()
            x, y, w, h = bbox
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame,
                        f'[{CAM_CN[name]}] d={dist:.1f}m '
                        f'b={math.degrees(bearing):+.0f}deg',
                        (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0), 1)
            wx = rx + math.cos(ryaw + bearing) * dist
            wy = ry + math.sin(ryaw + bearing) * dist
            fn = os.path.join(
                SNAP_DIR, f'{ts}_{name}_d{dist:.1f}m_'
                          f'b{math.degrees(bearing):+.0f}deg.jpg')
            cv2.imwrite(fn, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            saved.append(fn)
            self.hist.append({'t': now, 'cam': name,
                              'bearing': round(bearing, 3),
                              'dist': round(dist, 2),
                              'wx': round(wx, 2), 'wy': round(wy, 2)})
        # 滚动窗口: 超出保留数则删除最旧文件与历史
        self.hist = self.hist[-keep:] if keep > 0 else []
        files = sorted(f for f in os.listdir(SNAP_DIR) if f.endswith('.jpg'))
        excess = len(files) - keep
        for f in files[:max(excess, 0)]:
            try:
                os.remove(os.path.join(SNAP_DIR, f))
            except OSError:
                pass
        if saved:
            self.get_logger().info(
                f'VISION 📷 快照保存 {len(saved)} 张: '
                + ', '.join(os.path.basename(s) for s in saved))
        # 历史话题: 最近 N 张是否同一相机(跨相机=横向移动)
        seq = [h['cam'] for h in self.hist]
        lateral = len(set(seq)) > 1 if seq else False
        msg = String()
        msg.data = json.dumps({'seq': self.hist, 'lateral': lateral})
        self.pub_hist.publish(msg)

    # ---------- 主循环 ----------
    def _tick(self):
        now = time.monotonic()
        results = {}
        for name, _ in CAMS:
            results[name] = self._detect_cam(name)
        # 时间一致性: 优先保持上一主相机, 切换需连续 2 帧检出
        prev = self._prev_cam
        cand, cand_area = None, -1
        for name, (det, dist, bc, u, area) in results.items():
            if det and area > cand_area:
                cand, cand_area = name, area
        primary = None
        if prev and results.get(prev, (False,))[0]:
            primary = prev
        elif cand is not None:
            streak = self._cam_streak.get(cand, 0) + 1
            self._cam_streak[cand] = streak
            if streak >= 2:
                primary = cand
        if primary is not None:
            self._prev_cam = primary
        else:
            self._prev_cam = None
        self._frm_n += 1
        out = {'detected': False, 'cam': '', 'dist': -1.0,
               'bearing': 0.0, 'bearing_cam': 0.0, 'u': -1, 'area': 0,
               'age': 99.0}
        if primary is not None:
            self._det_n += 1
            det, dist, bc, u, area = results[primary]
            bearing = wrap(dict(CAMS)[primary] + bc)
            out = {'detected': True, 'cam': primary,
                   'dist': round(dist, 3), 'bearing': round(bearing, 4),
                   'bearing_cam': round(bc, 4), 'u': u, 'area': area,
                   'age': 0.0}
            if primary != 'front':
                self.get_logger().info(
                    f'VISION ★ 人位于[{CAM_CN[primary]}]相机视野 '
                    f'偏离中线{math.degrees(bc):+.1f}° '
                    f'(相对机器人{math.degrees(bearing):+.1f}°) '
                    f'距离={dist:.2f}m')
        msg = String()
        msg.data = json.dumps(out)
        self.pub.publish(msg)
        # 快照节拍
        rate = max(0.5, float(self.cfg['snapshot_rate_hz']))
        if now - self._last_snap >= 1.0 / rate:
            self._last_snap = now
            self._snapshot(results, now)
        # 周期汇总
        if now - self._last_log > 3.0:
            self._last_log = now
            if primary == 'front':
                det, dist, bc, u, area = results['front']
                self.get_logger().info(
                    f'VISION ★ 人位于[前]相机视野 偏离中线'
                    f'{math.degrees(bc):+.1f}° 距离={dist:.2f}m 面积={area}px')
            elif primary is None:
                self.get_logger().info(
                    f'VISION 四相机视野内均无人 (检出率 '
                    f'{self._det_n}/{max(self._frm_n, 1)})')

    def stats(self):
        return self._det_n, self._frm_n


def main(args=None):
    rclpy.init(args=args)
    node = PersonVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
