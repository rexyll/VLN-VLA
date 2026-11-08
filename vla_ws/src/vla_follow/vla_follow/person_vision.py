#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""person_vision.py —— 4 向 RGB-D 相机全向识"人" + 快照式跟踪管线

初始化数据（人的外观先验, 内置常量）:
  - "人"主体为深色圆柱(裙摆/躯干), 颜色暗(V<70)且低饱和(S<100)
  - 斑点面积 45~22000 px, 宽高比 0.2~2.8; 深度∈[0.45,11.5]m
  - 场景内其余物体均为彩色/浅色且已关闭阴影 —— 暗色斑点唯一对应"人"
快照管线(P12):
  - 4 相机以 snapshot_rate_hz(默认2Hz)抓拍; 仅保存检出目标的帧(带
    相机/方位/距离标注的 JPG), 未检出帧不落盘(等效立即删除);
  - 滚动保留最近 snapshot_keep 张(默认10);
  - 维护最近 N 条世界坐标历史, 发布 /perception/person_history 供
    follower 做去向推测与搜索导航。
目标选择(P20):
  - 单次扫描同时给出 bbox, 快照直接复用, 避免二次检测造成 bearing 与
    落盘标注不一致;
  - 跨相机时按"与上一拍世界方位的接近程度"选主相机(面积作兜底),
    防止目标骑在两台相机重叠区时主相机高频跳变、方位左右乱跳。
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
# P81: 面积上限必须覆盖"跟随时人最近能到多近"。
#   人中心 0.67m 时裙摆(0.52m 宽 / 0.44m 高)在 320x240 上已是 1.0m 外的
#   2.4 倍面积 ≈ 22000px, 正好撞上原来的 AREA_MAX → 0.67m 以内必然整帧
#   未检出。而需求带宽下界是 0.5m(中心 0.5m 时面积 ≈ 4.2 万)。
#   取 60000: 覆盖到中心 0.43m, 再近就靠 DEPTH_MIN(0.28m 表面 = 中心
#   0.54m)与上位机的 close_follow 兜底。
AREA_MIN, AREA_MAX = 20, 60000
# P81: 宽高比下界 0.2 → 0.14。人绕到柱子/箱子后面时只剩一条竖直残片,
#   实测面积 269~374px、宽高比 0.15~0.19, 上一版被 0.2 判死 → 一贴障碍
#   就丢。上界 2.8 不动(防止宽扁的暗色地砖/阴影碎片)。
ASPECT_MIN, ASPECT_MAX = 0.14, 2.8
HFOV = 1.8            # P81: 1.7453(100°) 与 SDF 的 horizontal_fov=1.8 不符,
                      # 方位角整体偏 3%, 边缘(±51°处)差 1.6°。按 SDF 改。
DEPTH_MIN = 0.28  # P38: 0.55 → 0.28。DEPTH_MIN 卡的是"人体表面"距离,
                  # 人中心距 0.6m 时表面只有 0.34m, 用 0.55 会整帧判未检出,
                  # 实测 <0.6m 检出率 0%(贴身即失明 → 反复丢失/GOTO)。
                  # P81: 不再下调 —— 机器人自身车体/脚轮在 0.10~0.25m 处
                  # 也是深色, 0.28 是"排除自遮挡"与"跟到 0.54m"的分界线。
# P34/P81: 由深度反推的"可见宽度"先验(米)。
#   侧身 ~0.20 / 腿部 ~0.34 / 躯干 ~0.52; 被障碍切掉一部分时只剩残片,
#   实测残片可窄到 0.09m。上限 0.60 用来挡"暗色墙面/暗色地板"这种远比
#   人体宽的目标(3m 处 200px 宽 = 4.7m, 一定剔除)。
W_VIS_MIN, W_VIS_MAX = 0.09, 0.60
W_REAL = 0.36     # 仅用于"该不该接受"的日志说明, 不再参与判决
DEPTH_MAX = 7.0   # P36: 11.5→7.0, 更远处 5px 的暗斑会被误判成人
# P104: 视觉观测只接受足够新鲜且 RGB/Depth 时间相近的配对。
# sensor 更新率为 10Hz；0.40s 已容忍数帧抖动，但不会把断流旧帧继续当新目标。
MAX_SENSOR_AGE_S = 0.40
MAX_RGB_DEPTH_SKEW_S = 0.18
MAX_POSE_RGBD_SKEW_S = 0.12
TRACK_ASSOC_MAX_AGE_S = 2.5
TRACK_ASSOC_BASE_GATE_M = 0.85
TRACK_ASSOC_SPEED_MPS = 0.85

CAMS = (('front', 0.0), ('left', 1.5707963),
        ('right', -1.5707963), ('back', 3.1415926))
CAM_CN = {'front': '前', 'left': '左', 'right': '右', 'back': '后'}


def timestamp_fresh(stamp, now, max_age=MAX_SENSOR_AGE_S):
    """判断单路时钟戳是否为有效的非未来新鲜观测。"""
    if stamp is None:
        return False
    stamp, now = float(stamp), float(now)
    return stamp > 0.0 and 0.0 <= now - stamp <= float(max_age)


def sensor_pair_fresh(rgb_t, dep_t, now,
                      max_age=MAX_SENSOR_AGE_S,
                      max_skew=MAX_RGB_DEPTH_SKEW_S):
    """判断 RGB/Depth 是否都是新帧且来自足够接近的采样时刻。"""
    if not timestamp_fresh(rgb_t, now, max_age):
        return False
    if not timestamp_fresh(dep_t, now, max_age):
        return False
    return abs(float(rgb_t) - float(dep_t)) <= float(max_skew)


def sensor_pose_pair_fresh(rgb_t, dep_t, pose_t, now,
                             max_age=MAX_SENSOR_AGE_S,
                             max_rgbd_skew=MAX_RGB_DEPTH_SKEW_S,
                             max_pose_skew=MAX_POSE_RGBD_SKEW_S):
    """RGB/Depth 与底盘位姿都新鲜，且机器人位姿时间接近图像帧对。"""
    if not sensor_pair_fresh(rgb_t, dep_t, now, max_age, max_rgbd_skew):
        return False
    if not timestamp_fresh(pose_t, now, max_age):
        return False
    obs_t = 0.5 * (float(rgb_t) + float(dep_t))
    return abs(float(pose_t) - obs_t) <= float(max_pose_skew)


def association_gate(age):
    """遮挡期间连续性门限随年龄渐进放宽，上限 1.8m。"""
    return min(1.80, TRACK_ASSOC_BASE_GATE_M
               + 0.65 * max(0.0, float(age)))


def association_error(world_xy, predicted):
    """候选人体世界位置与预测位置的欧氏误差；无预测时返回 None。"""
    if predicted is None:
        return None
    return math.hypot(float(world_xy[0]) - float(predicted[0]),
                      float(world_xy[1]) - float(predicted[1]))


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
    # PID 增益(P16): 距离环→线速度
    'pid_kp_v': 0.6, 'pid_ki_v': 0.20, 'pid_kd_v': 0.08,
    'pid_kp_w': 2.2, 'pid_ki_w': 0.40, 'pid_kd_w': 0.30,
    # 对正参数(P20): 目标允许偏离 front 相机中线的角度带
    'aim_band_deg': 15.0,
    'recenter_kp': 2.0,
    'turn_min_w': 0.35,
    'hold_kp': 1.6,
}


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def yaw_of(q):
    """四元数 → 航向角。

    P20 修正: 分母必须是 1-2*(y²+z²)。原写成 1-2*(y²+x²) 会把航向角
    严重压缩(真值 90° 读成 45°、180° 读成 0°), 导致世界坐标估计与
    方位换算全错, 跟随器对不正目标。
    """
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


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
        self.pose_t = 0.0
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
        self._prev_bearing = None      # 上一拍的目标世界方位
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
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    h, w, 4)[:, :, :3]
            else:
                return
            with self._lock:
                # P104: protobuf 消息 buffer 的生命周期不跨回调保证；必须复制，
                # 否则底层复用消息对象时缓存图像可能被下一帧静默改写。
                self.rgb[name] = img.copy()
                self.rgb_t[name] = time.monotonic()
        return cb

    def _mk_dep(self, name):
        def cb(msg):
            h, w = msg.height, msg.width
            if h < 10 or w < 10 or len(msg.data) < w * h * 4:
                return
            d = np.frombuffer(msg.data[:w * h * 4], dtype='<f4').reshape(h, w)
            with self._lock:
                self.dep[name] = d.copy()
                self.dep_t[name] = time.monotonic()
        return cb

    def _pose_cb(self, msg):
        for p in msg.pose:
            if p.name == 'robot':
                q = p.orientation
                with self._lock:
                    self.rx = p.position.x
                    self.ry = p.position.y
                    self.ryaw = yaw_of(q)
                    self.pose_t = time.monotonic()
                return

    def _predict_world(self, now):
        """P104: 最近一次主目标世界位置 + 限幅速度外推，供跨帧关联使用。

        历史只在每个采样周期追加一次（不再将四相机对同一人的重复观测
        当成四个连续时间点）。超过关联时窗则不强行锁旧位置，允许重新捕获。
        """
        h = self.hist or []
        if not h:
            return None
        last = h[-1]
        age = max(0.0, float(now) - float(last.get('t', now)))
        if age > TRACK_ASSOC_MAX_AGE_S:
            return None
        vx = vy = 0.0
        if len(h) >= 2:
            first = h[max(0, len(h) - 4)]
            dt = float(last.get('t', 0.0)) - float(first.get('t', 0.0))
            if dt >= 0.25:
                vx = (float(last['wx']) - float(first['wx'])) / dt
                vy = (float(last['wy']) - float(first['wy'])) / dt
                speed = math.hypot(vx, vy)
                if speed > TRACK_ASSOC_SPEED_MPS:
                    vx *= TRACK_ASSOC_SPEED_MPS / speed
                    vy *= TRACK_ASSOC_SPEED_MPS / speed
        horizon = min(age, 1.0)
        return (float(last['wx']) + vx * horizon,
                float(last['wy']) + vy * horizon, age)

    def _detect_cam(self, name, predicted=None):
        """单相机检测；优先选择与运动预测世界位置连续的候选，而非最大暗斑。"""
        now = time.monotonic()
        with self._lock:
            rgb = self.rgb.get(name)
            dep = self.dep.get(name)
            tr = self.rgb_t.get(name, 0.0)
            td = self.dep_t.get(name, 0.0)
            rx, ry, ryaw, pose_t = (
                self.rx, self.ry, self.ryaw, self.pose_t)
        miss = {'det': False, 'dist': -1.0, 'bc': 0.0, 'u': -1,
                'area': 0, 'bbox': None, 'world_xy': None,
                'track_error': None, 'obs_t': None}
        if rgb is None or dep is None or tr <= 0.0 or td <= 0.0:
            return miss
        # P104: 不能将相机断流前的旧图与另一条新深度帧拼成“检出”。
        # 世界坐标投影要求 RGB、Depth 与底盘位姿都新鲜，且位姿采样时刻
        # 靠近图像帧对；底盘移动期间不能拿“刚刚但不同步”的 pose 投影旧帧。
        if not sensor_pose_pair_fresh(tr, td, pose_t, now):
            return miss
        obs_t = 0.5 * (tr + td)
        if rgb.shape[:2] != dep.shape:
            dep = cv2.resize(dep, (rgb.shape[1], rgb.shape[0]))
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, (0, 0, 0), (179, SAT_MAX, DARK_V_MAX))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                np.ones((5, 5), np.uint8))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        img_w = rgb.shape[1]
        tan_h = math.tan(HFOV / 2.0)
        cam_yaw = dict(CAMS)[name]
        for cnt in cnts:
            area = cv2.contourArea(cnt)
            if area < AREA_MIN or area > AREA_MAX:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            asp = w / max(h, 1)
            if not (ASPECT_MIN <= asp <= ASPECT_MAX):
                continue
            # 先过滤无效深度，再比较目标候选，避免最大伪影吞掉真实人体。
            y0 = y + int(h * 0.15)
            y1 = y + int(h * 0.60)
            sub = dep[y0:max(y1, y0 + 1), x:x + w]
            valid = sub[np.isfinite(sub) & (sub > DEPTH_MIN)
                        & (sub < DEPTH_MAX)]
            if valid.size < 10:
                continue
            dist = float(np.quantile(valid, 0.30))
            # 被障碍切断的人体允许只剩 9cm 可见宽度。
            w_vis = dist * 2.0 * w * tan_h / img_w
            if not (W_VIS_MIN <= w_vis <= W_VIS_MAX):
                continue
            u = x + w / 2.0
            bc = (img_w / 2.0 - u) * HFOV / img_w
            bearing = wrap(cam_yaw + bc)
            wx = rx + math.cos(ryaw + bearing) * dist
            wy = ry + math.sin(ryaw + bearing) * dist
            track_error = None
            if predicted is not None:
                px, py, age = predicted
                track_error = association_error((wx, wy), predicted)
                # 门限随遮挡时间增长；不允许连续跟踪时跳到远处背景暗物体，
                # 但人已被遮挡较久时仍逐渐放宽，避免锁死旧目标。
                gate = association_gate(age)
                if age <= TRACK_ASSOC_MAX_AGE_S and track_error > gate:
                    continue
            candidates.append({
                'det': True, 'dist': dist, 'bc': bc, 'u': int(u),
                'area': int(area), 'bbox': (x, y, w, h),
                'world_xy': (float(wx), float(wy)),
                'track_error': track_error, 'obs_t': float(obs_t),
                # rgb 在回调中已 copy；后续回调会替换 dict 值而不会改写该数组，
                # 所以快照可复用与 bbox/深度完全对应的那一帧。
                'rgb_frame': rgb,
            })
        if not candidates:
            return miss
        if predicted is None:
            best = max(candidates, key=lambda v: v['area'])
        else:
            # 连续性是主排序键；仅在误差近似时才偏好面积较大的完整人体。
            best = min(candidates, key=lambda v: (
                float(v['track_error']), -v['area']))
        return best

    # ---------- 快照管线 ----------
    def _snapshot(self, results, now, primary=None):
        """2Hz: 只记录本周期选中的主目标，避免重叠相机制造重复速度样本。"""
        keep = int(self.cfg['snapshot_keep'])
        with self._lock:
            rx, ry, ryaw = self.rx, self.ry, self.ryaw
        saved = []
        names = [primary] if primary in results else []
        for name in names:
            r = results[name]
            if not r['det'] or r['bbox'] is None:
                continue
            # 复用候选生成时那张 RGB 快照，保证 bbox/深度/世界坐标/图像
            # 都属于同一帧，而不是从缓存取可能更新过的下一帧。
            rgb = r.get('rgb_frame')
            if rgb is None or not timestamp_fresh(
                    r.get('obs_t'), time.monotonic(), MAX_SENSOR_AGE_S):
                continue
            frame = rgb.copy()
            x, y, w, h = r['bbox']
            if frame.shape[1] <= x + w or frame.shape[0] <= y + h:
                continue
            bearing = wrap(dict(CAMS)[name] + r['bc'])
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame,
                        f'[{CAM_CN[name]}] d={r["dist"]:.1f}m '
                        f'b={math.degrees(bearing):+.0f}deg',
                        (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0), 1)
            wx, wy = r.get('world_xy') or (
                rx + math.cos(ryaw + bearing) * r['dist'],
                ry + math.sin(ryaw + bearing) * r['dist'])
            ts = time.strftime('%H%M%S')
            fn = os.path.join(
                SNAP_DIR, f'{ts}_{name}_d{r["dist"]:.1f}m_'
                          f'b{math.degrees(bearing):+.0f}deg.jpg')
            cv2.imwrite(fn, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            saved.append(fn)
            # 每轮最多一条观测。多相机同时看到同一个人时，不能把重复世界点
            # 当成同一人在很短时间内走了几米。
            self.hist.append({'t': float(r.get('obs_t') or now), 'cam': name,
                              'bearing': round(bearing, 3),
                              'dist': round(r['dist'], 2),
                              'wx': round(wx, 2), 'wy': round(wy, 2)})
        self.hist = self.hist[-keep:] if keep > 0 else []
        try:
            files = sorted(f for f in os.listdir(SNAP_DIR) if f.endswith('.jpg'))
            excess = len(files) - keep
            for f in files[:max(excess, 0)]:
                try:
                    os.remove(os.path.join(SNAP_DIR, f))
                except OSError:
                    pass
        except OSError:
            pass
        if saved:
            self.get_logger().info(
                f'VISION 📷 快照保存 {len(saved)} 张: '
                + ', '.join(os.path.basename(s) for s in saved))
        seq = [h['cam'] for h in self.hist]
        lateral = len(set(seq)) > 1 if seq else False
        msg = String()
        msg.data = json.dumps({'seq': self.hist, 'lateral': lateral})
        self.pub_hist.publish(msg)

    # ---------- 主循环 ----------
    def _tick(self):
        now = time.monotonic()
        predicted = self._predict_world(now)
        results = {}
        for name, _ in CAMS:
            results[name] = self._detect_cam(name, predicted)
        with self._lock:
            ryaw = self.ryaw
        prev = self._prev_cam
        cands = [n for n, r in results.items() if r['det']]
        primary = None
        if cands:
            front_ok = (results['front']['det']
                        and abs(results['front']['bc']) <= HFOV * 0.5 * 0.85)
            if predicted is not None:
                # 世界位置连续性优先；front 只在误差接近时提供视野质量偏好。
                ranked = sorted(cands, key=lambda n: (
                    float(results[n]['track_error']), -results[n]['area']))
                chosen = ranked[0]
                best_err = float(results[chosen]['track_error'])
                if prev in cands and float(results[prev]['track_error']) <= best_err + 0.18:
                    chosen = prev
                elif front_ok and float(results['front']['track_error']) <= best_err + 0.12:
                    chosen = 'front'
            elif front_ok:
                chosen = 'front'
            elif prev in cands:
                chosen = prev
            else:
                chosen = max(cands, key=lambda n: results[n]['area'])
            # 主相机切换仍保留连续两帧迟滞；中间若切回旧相机，候选 streak 必须清零，
            # 不能把不连续的两次出现累计成“连续两帧”。
            for _cam in tuple(self._cam_streak):
                if _cam != chosen:
                    self._cam_streak[_cam] = 0
            if chosen == prev or prev is None:
                primary = chosen
            else:
                self._cam_streak[chosen] = self._cam_streak.get(chosen, 0) + 1
                if self._cam_streak[chosen] >= 2:
                    primary = chosen
            if primary is not None:
                self._prev_cam = primary
                self._cam_streak[primary] = 2
        else:
            self._prev_cam = None
            self._cam_streak.clear()

        self._frm_n += 1
        out = {'detected': False, 'cam': '', 'dist': -1.0,
               'bearing': 0.0, 'bearing_cam': 0.0, 'u': -1, 'area': 0,
               'age': 99.0}
        if primary is not None:
            self._det_n += 1
            r = results[primary]
            bearing = wrap(dict(CAMS)[primary] + r['bc'])
            self._prev_bearing = bearing
            out = {'detected': True, 'cam': primary,
                   'dist': round(r['dist'], 3), 'bearing': round(bearing, 4),
                   'bearing_cam': round(r['bc'], 4), 'u': r['u'],
                   'area': r['area'], 'age': 0.0,
                   'track_error': (None if r['track_error'] is None
                                   else round(r['track_error'], 3))}
            if primary != 'front' and now - self._last_log > 1.0:
                self.get_logger().info(
                    f'VISION ★ 人位于[{CAM_CN[primary]}]相机视野 '
                    f'偏离中线{math.degrees(r["bc"]):+.1f}° '
                    f'(相对机器人{math.degrees(bearing):+.1f}°) '
                    f'距离={r["dist"]:.2f}m')
        msg = String()
        msg.data = json.dumps(out)
        self.pub.publish(msg)
        rate = max(0.5, float(self.cfg['snapshot_rate_hz']))
        if now - self._last_snap >= 1.0 / rate:
            self._last_snap = now
            self._snapshot(results, now, primary)
        if now - self._last_log > 3.0:
            self._last_log = now
            if primary == 'front':
                r = results['front']
                self.get_logger().info(
                    f'VISION ★ 人位于[前]相机视野 偏离中线'
                    f'{math.degrees(r["bc"]):+.1f}° 距离={r["dist"]:.2f}m '
                    f'面积={r["area"]}px')
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
