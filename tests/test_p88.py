#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_p88.py —— P88 验收: 放回水壶 + 双手捧杯（杯柄朝人）

验收口径(全部对 gz 真值, 不看自己算的数):
  [1] 数据链路就绪
  [2] 机器人到服务位
  [3] 双臂回 home
  [4] 倒水全链(pour_into_cup): 抓杯 → 举杯 → 抓壶 → 壶到倒水位 → 水柱 + 水位
  [5] `jet` 缩回 0、水位保持 65%（收水, 不能倒流)
  [6] 壶放回桌面原位: 位置 ≤30mm、壶轴竖直 ≤3°（"回原位后松手"）
  [7] 左手把杯送到胸前捧杯位: 位置 ≤5mm、杯轴倾 ≤2°
  [8] 右手贴到杯的对侧并冻结偏移: 两掌径向对称、夹角 ~180°
  [9] 原地**半速**转身面向人(62.9°): 真值 yaw 误差 ≤2°
  [10] 双手捧杯总验收: 杯竖直、两手对称、1s 不掉、**杯柄朝向人 ≤15°**、截图

为什么"转身"放在 P88 而不是 P89
================================
离线体检(`tests/diag/diag_p88_ik.py sweep`)算过: 机器人**还没转身**时,
"杯柄朝人"要求杯 yaw≈62.9°, 两手腕要跟着转 63° —— 左手腕余量只剩 4.0°
(r=0.50), r≤0.45 直接不可达(差 29~78mm), 右手同样不可达。
而 base 转到 62.9° 之后, "手端在身前"与"杯柄朝人"是**同一件事**, 余量
11.5°/24.2° 非常宽裕。所以捧杯和转身是一件事: 捧起来顺带转过去。
P89 只管"前伸递出 / 接回 / 放回"。

用法:
    bash tools/start_scene.sh
    source /opt/ros/jazzy/setup.bash; source vla_ws/install/setup.bash
    python3 tests/test_p88.py
"""
import math
import os
import sys
import time

import numpy as np
import rclpy
from sensor_msgs.msg import Image

from vla_follow import arm_kin as K
from vla_follow import drink_motion as M
from vla_follow import drink_poses as P
from vla_follow import scene_layout as L
from vla_follow.drink_task import DrinkTask
from vla_follow.pngsnap import save_png

LOG_DIR = os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'vla_ws', 'logs')

FILL_FRAC = 0.65
SPOT_TOL = 30.0        # 壶回原位容差 mm
HOLD_TOL = 5.0         # 捧杯位容差 mm


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


def snap(node, cam, tag, ok_sink):
    img = cam.grab(node)
    if img is None:
        ok_sink.append(False)
        return
    save_png(os.path.join(LOG_DIR, 'p88_%s.png' % tag),
             img.data, img.width, img.height, img.encoding)
    node.get_logger().info('相机 %s: %dx%d' % (tag, img.width, img.height))
    ok_sink.append(True)


def handle_angle_to(T_obj, target_xy):
    """杯柄方向(世界 xy) 与"杯子→目标点"方向的夹角(度)。"""
    h = np.asarray(T_obj)[:3, :3] @ np.array(L.CUP_HANDLE_DIR, float)
    v = np.array([target_xy[0] - T_obj[0, 3], target_xy[1] - T_obj[1, 3]])
    h2 = np.array([h[0], h[1]])
    nh, nv = np.linalg.norm(h2), np.linalg.norm(v)
    if nh < 1e-9 or nv < 1e-9:
        return 999.0
    c = max(-1.0, min(1.0, float(h2 @ v) / (nh * nv)))
    return math.degrees(math.acos(c))


def main():
    rclpy.init()
    node = DrinkTask()
    cam = Camera(node)
    fails = []
    ok_sink = []

    def step(tag, ok, detail=''):
        print('%s %-58s %s%s'
              % ('[OK]' if ok else '[NG]', tag, 'OK' if ok else 'NG',
                 ('  ' + detail) if detail else ''))
        if not ok:
            fails.append(tag)

    print('=' * 86)
    print('P88 验收 —— 放回水壶 + 双手捧杯（杯柄朝人）')
    print('=' * 86)

    # ---- [1] 数据就绪
    t0 = time.time()
    while time.time() - t0 < 10.0 and len(node.js) < 34:
        node.spin_for(0.2)
    step('[1] 数据链路  joint_state=%d 真值实体=%d'
         % (len(node.js), len(node.links)),
         len(node.js) >= 34 and 'cup' in node.links and 'kettle' in node.links)

    # ---- [2] 到服务位
    bx, by, _ = L.SERVE_POSE
    T_serve = K.frame(K.rot_z(0.0), (bx, by, 0.0))
    d_serve = M.place_robot(node, T_serve)
    step('[2] 机器人到服务位  偏差 %.1f mm' % d_serve, d_serve < 10.0)

    # ---- [3] home
    node.home()
    step('[3] 双臂回 home', True)

    # ---- [4] 倒水全链
    Tk_home = M.object_pose(node, 'kettle')
    pr = M.pour_into_cup(node, fill=FILL_FRAC)
    ok_p, res = pr['ok'], pr['res']
    step('[4] 倒水全链  抓杯 %.1f / 举杯 %.1f / 抓壶 %.1f / 壶就位 %.1f mm'
         % (res.get('grasp_cup', 9e9), res.get('lift_cup', 9e9),
            res.get('grasp_kettle', 9e9), res.get('kettle_pour', 9e9)), ok_p)
    if not ok_p:
        return report(fails, node)

    # ---- [5] 收水
    node.carry.joint_ramp('kettle', 'jet_joint', 0.0, dur=0.5)
    node.spin_for(0.4)
    jet = node.links.get('jet')
    jet_off = (float(np.linalg.norm(
        np.array([jet.position.x, jet.position.y, jet.position.z])
        - np.array([0.02229, 0.0, 0.04252]))) if jet is not None else None)
    water = (node.links['water'].position.z + 0.050
             if 'water' in node.links else None)
    step('[5] 收水  jet 缩回 %.1f mm  水位 %.1f mm(%.0f%%)'
         % ((jet_off or 0) * 1000.0, (water or 0) * 1000.0,
            100.0 * (water or 0) / L.CUP_WATER_MAX),
         jet_off is not None and jet_off < 0.006
         and water is not None and abs(water - L.CUP_WATER_MAX * FILL_FRAC) < 0.004)

    # ---- [6] 壶放回原位
    ok_d, res_d = M.put_down(node, 'R', 'kettle', Tk_home)
    node.spin_for(0.6)
    Tk_now = M.object_pose(node, 'kettle')
    d_k = M.pos_err_mm(Tk_now, Tk_home)
    tilt_k = M.tilt_deg(Tk_now)
    step('[6] 壶放回原位  偏差 %.1f mm  壶轴倾 %.2f°  松钉前 %.1f mm'
         % (d_k, tilt_k, res_d), ok_d and d_k < SPOT_TOL and tilt_k < 3.0)
    snap(node, cam, 'kettle_back', ok_sink)

    # ---- 右臂退到 home, 把桌面让给左手的杯子
    node.arm['R'].move(K.Q_REST, timeout=10.0)
    node.spin_for(0.4)

    # ---- [7] 杯到胸前捧杯位
    # 踩坑(P88): 底盘会**缓慢前移** —— 十几分钟的搬运动作过程中, 轮子被机械臂
    # 的反作用力一点点推着走, 实测从 (1.75,0.35) 漂到 (1.884,0.347), 还有个
    # 3° 的偏航。如果捧杯位用**名义** base 算, 杯子和机器人就错位 13cm, 转身
    # 之后量出来"杯子离 base 只有 0.366m"。所以先重新摆正底盘, 再用**真值**
    # base 算目标 —— 杯子一定在机器人正前方 0.50m, 后面所有期望位都自洽。
    M.place_robot(node, T_serve)
    bx_a, by_a, byaw_a = node.base
    Tcup_carry = P.cup_in_base(bx_a, by_a, byaw_a,
                               L.CUP_CARRY_R, L.CUP_CARRY_Z)
    ok_c, res_c = M.place_held(node, 'L', 'cup', Tcup_carry, tol_mm=HOLD_TOL)
    node.spin_for(0.5)
    Tcup_now = M.object_pose(node, 'cup')
    d_c = M.pos_err_mm(Tcup_now, Tcup_carry)
    tilt_c = M.tilt_deg(Tcup_now)
    step('[7] 杯到捧杯位  位置 %.1f mm  杯轴倾 %.2f°'
         % (d_c, tilt_c), ok_c and d_c < 8.0 and tilt_c < 2.5)

    # ---- [8] 右手贴到对侧
    # 用**捧杯偏置**(横向 0.075)而不是单手握的 0.036: 两手都按 0.036 摆的话
    # 掌心中心只差 0.072m, 而手掌本身就 0.08m 宽 —— 两只手会叠在一起。
    off_r = M.hand_onto(node, 'R', 'cup', lambda T: P.cup_cradle_palm(T, 'R'))
    dl = M.hand_gap(node, 'L', 'cup')
    dr = M.hand_gap(node, 'R', 'cup')
    spread = abs(dl[1] - dr[1]) if (dl and dr) else 0.0
    step('[8] 右手贴上对侧  杯系 y: L %+.3f / R %+.3f m  横向间距 %.3f m'
         % (dl[1] if dl else 9, dr[1] if dr else 9, spread),
         off_r is not None and dl and dr
         and dl[1] * dr[1] < 0 and 0.09 < spread < 0.18)
    snap(node, cam, 'two_hand', ok_sink)
    # 转身前记下"杯子在 base 系里的位姿" —— 转身期间手臂关节角一个都不变,
    # 所以这个量必须**守恒**。它才是"没掉"的正确判据: 世界坐标下的期望位会
    # 被底盘漂移污染(实测转身时底盘自己走了 6cm), 拿它当判据会假 NG。
    rel0 = np.linalg.inv(K.base_pose(*node.base)) @ M.object_pose(node, 'cup')

    # ---- [9] 原地半速转身面向人
    # 转多少不是常量, 而是"朝着人" —— 底盘有漂移, 按名义 62.9° 转会差几度,
    # 杯柄就不正对人的方向了。
    yaw_face = math.atan2(L.PERSON_SPOT[1] - node.base[1],
                          L.PERSON_SPOT[0] - node.base[0])
    ok_t, err_t = M.turn_base(node, yaw_face)
    dev_handover = M.wrap_deg(math.degrees(node.base[2] - L.HANDOVER_YAW))
    step('[9] 转身面向人  真值 yaw 误差 %.2f°(对名义 %.1f° 差 %.1f°)  base=(%.3f, %.3f)'
         % (err_t, math.degrees(L.HANDOVER_YAW), dev_handover,
            node.base[0], node.base[1]),
         ok_t and abs(err_t) < 2.5 and abs(dev_handover) < 8.0)

    # ---- [10] 双手捧杯总验收
    Tcup_now = M.object_pose(node, 'cup')
    drift0 = Tcup_now.copy()
    node.spin_for(1.0)
    Tcup_now = M.object_pose(node, 'cup')
    drift = M.pos_err_mm(Tcup_now, drift0)
    tilt_c = M.tilt_deg(Tcup_now)
    dl = M.hand_gap(node, 'L', 'cup')
    dr = M.hand_gap(node, 'R', 'cup')
    spread = abs(dl[1] - dr[1])
    hd = handle_angle_to(Tcup_now, L.PERSON_SPOT)
    bx_a, by_a, byaw_a = node.base
    Tcup_exp = P.cup_in_base(bx_a, by_a, byaw_a, L.CUP_CARRY_R, L.CUP_CARRY_Z)
    d_exp = M.pos_err_mm(Tcup_now, Tcup_exp)
    rel1 = np.linalg.inv(K.base_pose(*node.base)) @ Tcup_now
    d_rel = M.pos_err_mm(rel0, rel1)
    print('-' * 86)
    print('  诊断: 杯实位 %s  名义期望 %s  左手掌心 %s  持有=%s'
          % (np.round(Tcup_now[:3, 3], 3), np.round(Tcup_exp[:3, 3], 3),
             np.round(M.wrist(node, 'L')[:3, 3], 3), node.carry.holding()))
    step('[10a] 杯竖直且不掉  倾角 %.2f°  1s 漂移 %.2f mm  转身前后(base 系)位移 %.1f mm'
         % (tilt_c, drift, d_rel), tilt_c < 2.5 and drift < 2.0 and d_rel < 10.0)
    step('[10b] 双手捧住  横向间距 %.3f m  杯系 x: L %+.3f / R %+.3f  深度差 %.1f mm'
         % (spread, dl[0], dr[0], abs(dl[0] - dr[0]) * 1000.0),
         dl[1] * dr[1] < 0 and 0.09 < spread < 0.18
         and abs(dl[0] - dr[0]) < 0.035)
    step('[10c] 杯柄朝人  偏差 %.1f°  (人 @ %.2f,%.2f)'
         % (hd, L.PERSON_SPOT[0], L.PERSON_SPOT[1]), hd < 15.0)
    snap(node, cam, 'present', ok_sink)
    step('[10d] 截图 %d/3' % sum(1 for _ in ok_sink),
         all(ok_sink) and len(ok_sink) == 3)

    return report(fails, node)


def report(fails, node):
    print('-' * 86)
    print('结论: %s' % ('通过' if not fails else '不通过 (%d 项)' % len(fails)))
    for f in fails:
        print('  未通过: %s' % f)
    print('=' * 86)
    try:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass
    return 0 if not fails else 2


if __name__ == '__main__':
    sys.exit(main())
