#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_robust.py —— P94 接水位扰动鲁棒性验收

§6.7 后续方向第 1 项: 原来端到端验收里人的接水位是**固定**的, "人站得准"
是一个从没被拿掉的前提 —— 递杯位按 PERSON_SPOT 算, 人站准了自然对得上。
这里给"去喝水"的目标点加 ±0.15 m / ±20° 的扰动, 看整条链还成立不成立。

关键指标是 `d_hand_mm` = **递杯(PRESENT)阶段杯子到人手的最小水平距离**。
它直接决定了 `hand_to` 交接那一刻杯子会不会"瞬移": 递不到位的话, 杯子被
钉到人手上时会直接跳过去那么远(抓取附着是插件的非物理约束, §2.2)。

兜底链路(都是 P94 新加 / 放宽的):
    人站偏 → person_walk 走带扰动的目标点
           → prepare_present 按**实测人位**转身(原来写死 PERSON_SPOT)
           → _approach_if_far: 臂伸满还差 >35mm 就底盘朝人挪 ≤0.12m
           → _present_target: 前伸上限 0.62 → 0.70(diag_p94_reach 扫出来的
             安全上限), 方向始终指向**实测人手**

先起场景和节点:
    bash tools/start_scene.sh
    bash tools/start_drink_demo.sh
    python3 tests/test_robust.py             # 全部 7 组
    python3 tests/test_robust.py far yaw+    # 只跑指定组

产物: vla_ws/logs/test_robust.json + vla_ws/logs/p94_{case}_*.png

**本脚本只读**: 与 test_drink.py 同理, 用 `DrinkTask(observer_only=True)`。
"""
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import String

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'vla_ws', 'src', 'vla_follow'))

from vla_follow import drink_motion as M      # noqa: E402
from vla_follow import drink_poses as P       # noqa: E402
from vla_follow import scene_layout as L      # noqa: E402
from vla_follow.drink_task import DrinkTask   # noqa: E402
from vla_follow.pngsnap import save_png       # noqa: E402

LOG_DIR = os.path.join(ROOT, 'vla_ws', 'logs')
CASE_TIMEOUT = float(os.environ.get('P94_TIMEOUT', '300'))
# 人"开口"的等待上限: person_walk 的 GOTO_TIMEOUT 是 120s, 超过就放弃喝水
WAIT_SPEAK = float(os.environ.get('P94_WAIT_SPEAK', '150'))
# 递到手边判据(与 test_p89 [3] 同口径 45mm)
# 人到位判据 150mm: GOTO 的容差是 0.10m(#51), 加上绕障退出时的侧移,
# 实测 74~137mm。120mm 会把"站准了只是绕了一下"判成没走到。
D_HAND_OK_MM = 45.0
SHOT_AT = {'PRESENT': 'present', 'PERSON_DRINK': 'drink'}

# 扰动组: (名字, jitter) —— a 是扰动方位(世界, 度), 62.9° = "远离机器人"
# 的方向(PERSON_SPOT 相对服务位的方位), 242.9° = 靠近机器人, 另两个是切向。
CASES = [
    ('base',  {'d': 0.00, 'a': 0.0,   'yaw': 0.0}),
    ('far',   {'d': 0.15, 'a': 62.9,  'yaw': 0.0}),
    ('near',  {'d': 0.15, 'a': 242.9, 'yaw': 0.0}),
    ('side+', {'d': 0.15, 'a': 152.9, 'yaw': 0.0}),
    ('side-', {'d': 0.15, 'a': -27.1, 'yaw': 0.0}),
    ('yaw-',  {'d': 0.00, 'a': 0.0,   'yaw': -20.0}),
    ('yaw+',  {'d': 0.00, 'a': 0.0,   'yaw': 20.0}),
]


def wrap_deg(a):
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


class Watch(object):
    """只读地跟状态 / 抓图 / 发指令(带扰动)。"""

    def __init__(self, node):
        self.node = node
        self.state = 'BOOT'
        self.stages = []
        self.payload = {}
        self.cmds = []
        self.img = None
        self.t0 = time.time()
        node.create_subscription(String, '/drink/state', self._state_cb, 10)
        node.create_subscription(String, '/robot/command', self._cmd_cb, 10)
        node.create_subscription(Image, '/drink_cam/image', self._img_cb, 5)
        self.pub = node.create_publisher(String, '/person/command', 10)

    def _state_cb(self, m):
        try:
            d = json.loads(m.data)
        except Exception:
            return
        s = d.get('state')
        if s and s != self.state:
            self.state = s
            self.stages.append(s)
            self.payload[s] = d
            print('      [阶段] %-12s t=%5.1fs' % (s, time.time() - self.t0))

    def _cmd_cb(self, m):
        try:
            c = json.loads(m.data)
        except Exception:
            return
        if c.get('action') == 'DRINK':
            self.cmds.append(c)

    def _img_cb(self, m):
        self.img = m

    def spin(self, s):
        self.node.spin_for(s)

    def reset(self):
        self.state = 'IDLE'
        self.stages = []
        self.payload = {}
        self.cmds = []
        self.t0 = time.time()

    def send(self, action, extra=None):
        d = {'action': action}
        if extra:
            d.update(extra)
        msg = String(data=json.dumps(d))
        # P94 踩的坑: **publisher 刚建好就发, 消息会静默丢掉** —— DDS 发现
        # (discovery)完成之前 publisher 眼里"没有订阅者", 那一帧直接蒸发
        # (§6.4 #24 是同一个病, 只不过那条是单向 gz 话题)。症状是"人一直
        # 在随机走、150s 超时没开口", 而手动 `ros2 topic pub` 一试就灵 ——
        # 因为 CLI 自己会等发现。所以先等 `get_subscription_count() > 0`。
        for _ in range(100):
            if self.pub.get_subscription_count() > 0:
                break
            self.spin(0.2)
        for _ in range(3):
            self.pub.publish(msg)
            self.spin(0.3)

    def shot(self, name):
        if self.img is None:
            return None
        path = os.path.join(LOG_DIR, 'p94_%s.png' % name)
        save_png(path, self.img.data, self.img.width, self.img.height,
                 self.img.encoding)
        return path


def run_case(node, w, name, jit, shots):
    """跑一组扰动, 返回 (结果 dict, 是否通过)。"""
    print('-' * 92)
    print('[%s] 扰动: d=%.2fm a=%+.1f° yaw=%+.0f°'
          % (name, jit['d'], jit['a'], jit['yaw']))
    tgt = (L.PERSON_SPOT[0] + jit['d'] * math.cos(math.radians(jit['a'])),
           L.PERSON_SPOT[1] + jit['d'] * math.sin(math.radians(jit['a'])))
    r = {'case': name, 'jitter': jit, 'target': [round(v, 3) for v in tgt]}
    fails = []

    def step(tag, ok, detail=''):
        print('    %s %-46s %s%s' % ('[OK]' if ok else '[NG]', tag,
                                     'OK' if ok else 'NG',
                                     ('  ' + detail) if detail else ''))
        if not ok:
            fails.append(tag)

    def T(name_):
        return M.object_pose(node, name_)

    def xy(name_):
        t = T(name_)
        return None if t is None else (float(t[0, 3]), float(t[1, 3]))

    w.reset()
    p0 = xy('person')
    r['person_start'] = [round(v, 3) for v in p0] if p0 else None
    r['start_dist_m'] = (round(math.hypot(p0[0] - tgt[0], p0[1] - tgt[1]), 2)
                         if p0 else None)
    w.send('GO_DRINK', {'jitter': jit})

    # ---- 跟一轮: 采样 PRESENT 段的"杯-手"距离, 等终态
    d_hand = []
    arrive = None
    t0 = time.time()
    deadline = t0 + CASE_TIMEOUT
    spoken = False
    while time.time() < deadline:
        w.spin(0.15)
        if arrive is None and w.cmds:
            spoken = True
            px, py = xy('person')
            yaw = math.atan2(T('person')[1, 0], T('person')[0, 0])
            # P95: "面向谁" 要跟**服务位**比, 不能跟机器人当前位置比 ——
            # 人开口那一刻 follower 正带着机器人在他周围绕, 而人刻意面向
            # 服务位(机器人做喝水任务时必然就位的地方, 也是 PERSON_SPOT /
            # CUP_PRESENT_R 那套布局数字的前提)。
            arrive = {
                'to_target_mm': math.hypot(px - tgt[0], py - tgt[1]) * 1000.0,
                'off_nominal_mm': math.hypot(px - L.PERSON_SPOT[0],
                                             py - L.PERSON_SPOT[1]) * 1000.0,
                'face_deg': abs(wrap_deg(math.degrees(
                    math.atan2(L.SERVE_POSE[1] - py, L.SERVE_POSE[0] - px)
                    - yaw))),
                'pos': [round(px, 3), round(py, 3)],
            }
        if w.state == 'PRESENT':
            Tc, Tp = T('cup'), T('person')
            Th = M.person_hand_world(node, Tp)
            if Tc is not None and Th is not None:
                d_hand.append(float(np.linalg.norm(
                    Tc[:3, 3] - Th[:3, 3])) * 1000.0)
        tag = SHOT_AT.get(w.state)
        if tag and tag not in shots:
            shots.add(tag)
            sh = w.shot('%s_%s' % (name, tag))
            if sh:
                r.setdefault('shots', []).append(sh)
        if w.state in ('DONE', 'ERROR', 'FAIL_PREPARE', 'FAIL_HANDOVER'):
            break
        if not spoken and time.time() - t0 > WAIT_SPEAK:
            print('      [超时] %.0fs 内人没有开口(走不到扰动后的接水位?)'
                  % WAIT_SPEAK)
            break

    r['elapsed_s'] = round(time.time() - w.t0, 1)
    r['stages'] = list(w.stages)
    r['spoken'] = spoken
    r['arrive'] = arrive
    r['d_hand_mm'] = round(min(d_hand), 1) if d_hand else None
    r['d_hand_end_mm'] = round(d_hand[-1], 1) if d_hand else None
    r['n_samples'] = len(d_hand)
    r['approach'] = w.payload.get('HANDOVER', {}).get('approach')

    # ---- [a] 人走到扰动后的目标点
    if arrive:
        r['to_target_mm'] = round(arrive['to_target_mm'], 1)
        r['off_nominal_mm'] = round(arrive['off_nominal_mm'], 1)
        r['face_deg'] = round(arrive['face_deg'], 1)
        step('[a] 人走到扰动目标点  偏差 %.0fmm (离名义位 %.0fmm, 面向差 %.0f°)'
             % (arrive['to_target_mm'], arrive['off_nominal_mm'],
                arrive['face_deg']),
             arrive['to_target_mm'] < 150.0)
    else:
        step('[a] 人走到扰动目标点  没走到 / 没开口', False)

    # ---- [b] 递到手边(核心指标)
    if r['d_hand_mm'] is not None:
        step('[b] 递杯到人手边  最小间距 %.1fmm (末值 %.1fmm, %d 采样)'
             % (r['d_hand_mm'], r['d_hand_end_mm'], r['n_samples']),
             r['d_hand_mm'] < D_HAND_OK_MM)
    else:
        step('[b] 递杯到人手边  PRESENT 段没采到样本', False)

    # ---- [c] 阶段链
    need = ['PREPARE', 'PRESENT', 'HANDOVER', 'PERSON_DRINK',
            'TAKE_BACK', 'PUT_BACK', 'RETREAT', 'HANDBACK', 'DONE']
    miss = [s for s in need if s not in w.stages]
    r['miss_stages'] = miss
    step('[c] 动作全流程  阶段 %d/%d  缺 %s  用时 %.0fs'
         % (len(need) - len(miss), len(need), miss or '无', r['elapsed_s']),
         not miss)

    # ---- [d] 杯子回原位
    Tcup = T('cup')
    if Tcup is not None:
        d_home = float(np.linalg.norm(
            Tcup[:3, 3] - np.asarray(P.cup_on_table())[:3, 3])) * 1000.0
        tilt = math.degrees(math.acos(
            max(-1.0, min(1.0, float(Tcup[2, 2])))))
        r['cup_home_mm'] = round(d_home, 1)
        r['cup_tilt_deg'] = round(tilt, 2)
        step('[d] 杯子放回原位  偏差 %.1fmm  杯轴倾 %.2f°' % (d_home, tilt),
             d_home < 10.0 and tilt < 2.0)
    else:
        step('[d] 杯子放回原位  拿不到真值', False)

    app = r.get('approach') or {}
    print('      补位: %s' % ('车→人手 %.3f→%.3fm (挪 %.3fm, 行驶残差 %.0fmm)'
                              % (app.get('d_hb', -1), app.get('d_after', -1),
                                 app.get('step_m', 0),
                                 app.get('drive_res_mm', -1))
                              if app.get('moved')
                              else ('没挪(间距 %.3fm 在臂长内)'
                                    % app.get('d_hb', -1))
                              if app else '无数据'))
    r['ok'] = not fails
    r['fails'] = fails
    return r, not fails


def main():
    rclpy.init()
    node = DrinkTask(observer_only=True)
    w = Watch(node)
    print('=' * 92)
    print('P94 接水位扰动鲁棒性 —— 人站偏 ±0.15m / ±20° 时递杯还成不成立')
    print('  名义接水位 (%.3f, %.3f)  前伸上限 %.2f→%.2fm  判据: 杯-手 <%.0fmm'
          % (L.PERSON_SPOT[0], L.PERSON_SPOT[1], L.CUP_PRESENT_R,
             L.CUP_PRESENT_R_MAX, D_HAND_OK_MM))
    print('=' * 92)

    # DDS 发现预热: 本进程是新加入的订阅者, 发现完成前收发的都是静默丢帧
    # (§6.4 #24)。drink_task 每 5s 广播一次 state 心跳, 等到收到第一条再
    # 开跑 —— 否则**第一组会缺 PREPARE**(实测踩过: 第一组 9 个阶段只收到
    # 8 个, 第二组开始就齐了, 因为到那时候发现早已完成)。
    t0 = time.time()
    while time.time() - t0 < 30.0 and not w.stages:
        w.spin(0.3)
    print('DDS 预热: %s' % ('已收到 /drink/state 心跳'
                            if w.stages else
                            '30s 内没收到心跳, 继续(第一组可能缺阶段)'))

    want = sys.argv[1:]
    cases = [c for c in CASES if not want or c[0] in want]
    if not cases:
        print('没有匹配的组: %s' % want)
        return 2

    # 数据链路
    t0 = time.time()
    while time.time() - t0 < 20.0 and not all(
            M.object_pose(node, n) is not None
            for n in ('person', 'cup', 'robot')):
        w.spin(0.3)
    ok_link = all(M.object_pose(node, n) is not None
                  for n in ('person', 'cup', 'robot'))
    print('数据链路: %s' % ('OK' if ok_link else 'NG (拿不到真值)'))
    if not ok_link:
        return 2

    results = []
    shots_all = set()
    for name, jit in cases:
        r, _ok = run_case(node, w, name, jit, shots_all)
        results.append(r)
        # 组间: 等 drink_task 回到 IDLE(它在 DONE 上停 3s), 再给人和
        # follower 一点时间散开, 免得下一组人还在 LEAVE 里。
        t0 = time.time()
        while time.time() - t0 < 20.0 and w.state not in ('IDLE', 'BOOT'):
            w.spin(0.3)
        w.spin(3.0)

    # ---- 汇总
    print('=' * 92)
    print('%-8s %-22s %10s %10s %10s %8s  %s'
          % ('组', '扰动', '到位mm', '离名义mm', '杯-手mm', '放回mm', '结论'))
    print('-' * 92)
    npass = 0
    for r in results:
        if r.get('ok'):
            npass += 1
        j = r['jitter']
        print('%-8s %-22s %10s %10s %10s %8s  %s'
              % (r['case'],
                 'd=%.2f a=%+.0f° yaw=%+.0f°' % (j['d'], j['a'], j['yaw']),
                 r.get('to_target_mm', '-'), r.get('off_nominal_mm', '-'),
                 r.get('d_hand_mm', '-'), r.get('cup_home_mm', '-'),
                 '通过' if r.get('ok') else 'FAIL: ' + ';'.join(r['fails'])))
    print('-' * 92)
    dh = [r['d_hand_mm'] for r in results if r.get('d_hand_mm') is not None]
    if dh:
        print('杯-手间距: 最大 %.1fmm / 中位 %.1fmm / 判据 %.0fmm'
              % (max(dh), sorted(dh)[len(dh) // 2], D_HAND_OK_MM))
    print('结论: %d/%d 组通过' % (npass, len(results)))
    print('=' * 92)

    out = {'ok': npass == len(results), 'n_pass': npass,
           'n_case': len(results), 'criteria_mm': D_HAND_OK_MM,
           'results': results}
    os.makedirs(LOG_DIR, exist_ok=True)
    path = os.path.join(LOG_DIR, 'test_robust.json')
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print('结果 JSON: %s' % path)
    try:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass
    return 0 if npass == len(results) else 2


if __name__ == '__main__':
    sys.exit(main())
