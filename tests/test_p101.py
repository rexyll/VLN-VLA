#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_p101.py —— P101 三项"像人一样"的行为验收

用户 2026-10-28 提的三条, 逐条给判据:

[1] **收尾复位**: 任务做完、准备跟人走的时候, 双臂必须回到悬垂(Q_HANG)、
    十指回到 GRIP_HANG。判据: 实测关节与 Q_HANG 的最大偏差 ≤ 3°。
    (原来收尾停在 Q_REST —— 肘弯着、十指张着, 交权后就这样跟着人走。)

[2] **肘朝外**: 双手捧杯那一刻, 肘要像人一样架在体侧。
    判据用 `out_line`(肘点在"肩→腕"轴垂线上的外侧分量, 与手臂伸出多远
    无关): ①捧杯(carry, 0.50m 处) ≥ 60mm; ②递杯(present) ≥ 60mm。
    老版本 R 侧这个口径是 **−14mm**(肘拐向身体中线、肘尖朝下像端托盘);
    修好后实测 +112~+242mm。
    `out_base`(相对车身横向)只记录不判: 它随手臂弯曲程度大幅摆动(0.62m 时
    L +48mm、0.57m 时 −16mm), 而在极限伸出(0.70m)时会退化到 +12mm ——
    直臂本来就没有肘, 拿它当闸门会把几何必然误判成姿态不对。

[3] **双手一起放回**: 杯子落稳之前两只手都不许松。
    判据: ①落稳时两只手到**杯轴**的水平距离 ≤ 160mm(相切 92mm, 撤走一步
    就 ≥190mm, 所以这一条能分开"还捧着"和"已经撤了"); ②松钉后杯子没有往
    下掉(|drop| ≤ 5mm) —— ②是"提前松手"的物理证据, 松早了杯子会先下坠。

先起场景和节点:
    bash tools/start_scene.sh
    bash tools/start_drink_demo.sh
    python3 tests/test_p101.py

产物: vla_ws/logs/p101_{present,grip,takeback,placed}.png + tests/out/test_p101.json
本脚本只读(observer_only=True), 不参与控制。
"""
import json
import math
import os
import subprocess
import sys
import time

import rclpy
from sensor_msgs.msg import Image
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'vla_ws', 'src', 'vla_follow'))

from vla_follow import arm_kin as K            # noqa: E402
from vla_follow import drink_motion as M      # noqa: E402
from vla_follow import scene_layout as L      # noqa: E402
from vla_follow.drink_task import DrinkTask   # noqa: E402
from vla_follow.pngsnap import save_png       # noqa: E402

LOG_DIR = os.path.join(ROOT, 'vla_ws', 'logs')
OUT_DIR = os.path.join(ROOT, 'tests', 'out')
TOTAL_TIMEOUT = 300.0
SHOT_AT = {'PRESENT': 'present', 'HANDOVER': 'grip',
           'TAKE_BACK': 'takeback', 'DONE': 'placed'}


class Watch(object):
    def __init__(self, node):
        self.node = node
        self.state = 'BOOT'
        self.stages = []
        self.result = {}
        self.img = None
        self.t0 = time.time()
        # 仿真时钟: 判"全流程用时"要用**逻辑时长**。RTF 0.3 的机器上墙上
        # 时间没有可移植性(同一份代码 300s / 390s 都可能), 而机器人自己经历
        # 的时间是 /clock。
        self.sim = None
        node.create_subscription(Clock, '/clock', self._clock_cb, 10)
        node.create_subscription(String, '/drink/state', self._state_cb, 10)
        node.create_subscription(Image, '/drink_cam/image', self._img_cb, 5)
        self.pub = node.create_publisher(String, '/person/command', 10)
        self.pub_robot = node.create_publisher(String, '/robot/command', 10)

    def _clock_cb(self, m):
        self.sim = float(m.clock.sec) + float(m.clock.nanosec) * 1e-9

    def _state_cb(self, m):
        try:
            d = json.loads(m.data)
        except Exception:
            return
        r = d.get('result') or {}
        # 每个阶段都会带一份 result, **合并**而不是整份覆盖:
        # 节点在状态不变时只发 {'state': s} (无 result), 而 P101 的三组量分别
        # 在 HANDOVER / PUT_BACK / RETREAT 之后才写进 result, 后到的若是
        # 一份小 dict, 整份替换会把前一组量冲掉。非 None 的字段才算有新值。
        if isinstance(r, dict):
            for k, v in r.items():
                if v is not None:
                    self.result[k] = v
        s = d.get('state')
        if s and s != self.state:
            self.state = s
            self.stages.append(s)
            print('    [阶段] %-12s t=%.1fs' % (s, time.time() - self.t0))

    def _img_cb(self, m):
        self.img = m

    def spin(self, s):
        self.node.spin_for(s)

    def reset(self):
        self.state = 'IDLE'
        self.stages = []
        self.result = {}

    def send(self, action):
        for _ in range(2):
            self.pub.publish(String(data=json.dumps({'action': action})))
            self.spin(0.6)

    def send_robot(self, action):
        """直接给机器人任务节点下指令(单向话题, 连发几帧防发现期丢包)。"""
        for _ in range(3):
            self.pub_robot.publish(String(data=json.dumps({'action': action})))
            self.spin(0.5)

    def shot(self, tag, prefix='p101'):
        if self.img is None:
            return None
        p = os.path.join(LOG_DIR, '%s_%s.png' % (prefix, tag))
        save_png(p, self.img.data, self.img.width, self.img.height,
                 self.img.encoding)
        return p


def layout_scene(node, w, quiet=False):
    """布景: 车摆回服务位并面朝人, 人摆到 PERSON_SPOT。

    P102: 这段被 test_p102 复用, 所以从 main() 里提出来(两边布景必须一致,
    否则同一套判据在不同布景上量出来的数没法比)。
    """
    px, py = L.PERSON_SPOT
    rx, ry = L.SERVE_POSE[0], L.SERVE_POSE[1]
    yaw_r = math.atan2(py - ry, px - rx)          # 车面朝人
    node.carry.place('robot', K.frame(K.rot_z(yaw_r), (rx, ry, 0.0)))
    w.spin(1.2)
    node.sync_base_from_truth(quiet=True)
    bx, by, _byaw = node.base
    Tp = M.object_pose(node, 'person')
    Th = M.person_hand_world(node, Tp) if Tp is not None else None
    d_hb = (math.hypot(Th[0, 3] - bx, Th[1, 3] - by) if Th is not None else -1.0)
    if not quiet:
        print('    布景: 车(%.2f,%.2f,yaw%.0f°) 人(%.2f,%.2f) 车→人手 %.3fm'
              '(设计 0.62, 臂长极限 0.70)'
              % (bx, by, math.degrees(_byaw),
                 Tp[0, 3] if Tp is not None else -1,
                 Tp[1, 3] if Tp is not None else -1, d_hb))
    return d_hb


def main():
    rclpy.init()
    node = DrinkTask(observer_only=True)
    w = Watch(node)
    fails = []
    res = {'ok': False, 'fails': fails, 'shots': []}

    def step(tag, ok, detail=''):
        print('%s %-56s %s%s' % ('[OK]' if ok else '[NG]', tag,
                                 'OK' if ok else 'NG',
                                 ('  ' + detail) if detail else ''))
        if not ok:
            fails.append(tag)

    print('=' * 96)
    print('P101 验收 —— 收尾悬垂 / 捧杯肘朝外 / 双手一起放回')
    print('=' * 96)

    t0 = time.time()
    while time.time() - t0 < 20.0 and M.object_pose(node, 'cup') is None:
        w.spin(0.3)
    if M.object_pose(node, 'cup') is None:
        step('[0] 数据链路(真值)', False, '没有 cup 真值')
        return report(node, res)

    # 布景: 只关掉 **follower**。它看到人就把车往"跟随距离"上拽, 摆好的
    # SERVE_POSE 会被拽走(实测被拽到 (1.62,0.12)/(1.61,0.18)/(1.54,0.06),
    # 车→人手在 0.56~0.92m 之间乱跳)。
    #
    # person_walk / cmd_mux **必须留着**:
    #   · person_walk 一死, "人"就变成一个没人控的自由刚体 —— 摆好之后会
    #     自己漂走(实测 0.9s 漂 6cm, 半分钟漂到车→人手 0.44m), 几何不可复现;
    #   · cmd_mux 一死, `_approach_if_far` 的补位和 `turn_base` 的转身都发
    #     不出去 —— 人贴得近时那 4cm 的超前量没人补, 双手捧杯会折成一团
    #     (实测 R 侧肘线口径 −229mm、放回时杯子掉 92mm)。
    subprocess.call(['pkill', '-f', 'vla_follow/follower'])
    w.spin(0.8)
    layout_scene(node, w)

    w.reset()
    # 走真实入口: 让"人"自己走到桌边说一句"我要喝水"。人贴到哪就是哪 ——
    # 这正是 P94 的扰动口径(±0.15m/±20°), 姿态验收必须在它上面成立。
    w.send('GO_DRINK')
    # 但人偶尔会在桌沿"绕行"里打转(P95 遗留), 90s 还没进 PREPARE 就退回
    # 直接给机器人下指令, 别干等 300s。
    t_wait = time.time()
    while time.time() - t_wait < 90.0 and w.state not in ('PREPARE',) \
            and not w.state.startswith('FAIL'):
        w.spin(0.3)
    if w.state != 'PREPARE':
        print('    人的步行 90s 内没把任务带起来(当前 %s), 直接给机器人下 DRINK'
              % w.state)
        w.send_robot('DRINK')
        w.spin(2.0)
    seen = set()
    t0 = time.time()
    while time.time() - t0 < TOTAL_TIMEOUT:
        w.spin(0.2)
        # 失败态(IK 不到位/交接失败/异常)别干等到超时: 立刻收尾, 让人能马上
        # 看到是哪一项挂了 —— 否则要白等 300s, 而且失败现场也被拖没了。
        if w.state.startswith('FAIL') or w.state == 'ERROR':
            w.spin(1.0)
            break
        if w.state in SHOT_AT and w.state not in seen:
            seen.add(w.state)
            p = w.shot(SHOT_AT[w.state])
            if p:
                res['shots'].append(p)
        if w.state == 'DONE':
            w.spin(1.0)          # 抓 DONE 终态(截图 + 判完成)
            # P101 的三组量挂在**最后那条**广播上: 节点在 DONE 停 3s 才回
            # IDLE, 带 result 的那条是在 `finally` 里发的(`{'state','result'}`),
            # 比 DONE 晚 ~3s。只等 1.5s 就会读到空 result —— 必须等 IDLE。
            t1 = time.time()
            while time.time() - t1 < 15.0 and w.state != 'IDLE':
                w.spin(0.2)
            w.spin(1.0)
            break
    done = 'DONE' in w.stages
    step('[1] 任务跑完(%s)' % ' → '.join(w.stages[-4:]), done,
         'stages=%d' % len(w.stages))
    r = w.result
    res['result'] = r

    # ---- [2] 肘朝外。两个时刻都量, 判据用 `out_line`(与伸出多远无关):
    #      · 捧杯(carry, 车前方 0.50m): 手臂有余量弯曲, 肘应该明显架出去
    #      · 递杯(present, 0.62~0.70m): 手臂接近伸直, 也要求架出去
    #      `out_base`(相对车身横向)在直臂构型下会掉到 +12mm, 那是几何必然,
    #      所以只作参考、不用作判据(对照见 arm_kin.elbow_metrics 注释)。
    ec = r.get('elbows_carry') or {}
    ep = r.get('elbows') or {}

    def _g(pk, s, k):
        return (pk.get(s) or {}).get(k)

    lc, rc = _g(ec, 'L', 'out_line_mm'), _g(ec, 'R', 'out_line_mm')
    step('[2a] 捧杯(carry)肘外扩(线)  L %s / R %s mm (判据 ≥60)'
         % (lc, rc), lc is not None and rc is not None
         and lc >= 60.0 and rc >= 60.0,
         '基础系 L %s / R %s mm, 垂 L %s / R %s mm'
         % (_g(ec, 'L', 'out_base_mm'), _g(ec, 'R', 'out_base_mm'),
            _g(ec, 'L', 'droop_mm'), _g(ec, 'R', 'droop_mm')))
    lp, rp = _g(ep, 'L', 'out_line_mm'), _g(ep, 'R', 'out_line_mm')
    lb, rb = _g(ep, 'L', 'out_base_mm'), _g(ep, 'R', 'out_base_mm')
    step('[2b] 递杯(present)肘外扩(线)  L %s / R %s mm (判据 ≥60)'
         % (lp, rp), lp is not None and rp is not None
         and lp >= 60.0 and rp >= 60.0,
         '基础系 L %s / R %s mm(仅供参考, 见下), 伸出 L %s / R %s m'
         % (lb, rb, _g(ep, 'L', 'reach_m'), _g(ep, 'R', 'reach_m')))
    # 车身横向口径**只记录不判**: 杯子在正前方 0.62m 时它是 +48/+174, 而杯子
    # 更近(0.57m)时 L 会落到 −16mm —— 肘是往下外架(垂 164mm、线 +131mm 都
    # 说明"架出去了"), 但从车身看略微内收。这个口径随手臂弯曲程度大幅摆动,
    # 不适合当闸门; 老版本的病态姿态在**线口径**上就已经是 −14mm(R), 判据
    # [2a][2b] 的 60mm 门槛足以把它拦下。

    # ---- [3] 双手一起放回
    pb = r.get('putback') or {}
    ha = pb.get('hand_axis_rest') or {}
    hg = pb.get('hand_gap_rest') or {}
    al, ar = ha.get('L'), ha.get('R')
    rest = pb.get('rest_mm')
    drop = pb.get('drop_mm')
    # 判据用"掌心到杯轴的水平距离": 捧杯相切值 92mm; 松手撤走一步(0.10m)
    # 就 ≥190mm, 所以 160mm 能把"还捧着"和"已经撤了"分开, 同时吸收手部
    # 跟随的滞后(实测 80~135mm)。3D 到杯心的距离含 60mm 级高差, 只作参考。
    step('[3a] 落稳时两只手都在杯旁  到杯轴 L %s / R %s mm (判据 ≤160)'
         % (al, ar), al is not None and ar is not None
         and al <= 160.0 and ar <= 160.0,
         '到杯心3D L %s / R %s mm; 杯落稳 %.1fmm' % (hg.get('L'), hg.get('R'),
                                                     rest if rest is not None else -1))
    step('[3b] 落稳残差 %.1f mm(判据 ≤5) 松钉后下坠 %s mm(判据 |·|≤5)'
         % (rest if rest is not None else -1, drop),
         rest is not None and rest <= 5.0
         and drop is not None and abs(drop) <= 5.0)

    # ---- [4] 收尾悬垂(读实测关节, 不只读节点自己算的)
    hd = r.get('hang_deg') or {}
    q_now = {}
    for s in ('R', 'L'):
        names = ['%s_%s_joint' % (s, j[0]) for j in K.JOINTS]
        q = M.q_meas(node, s)
        q_now[s] = None if q is None else float(
            max(abs(v) for v in q))
    step('[4] 收尾悬垂  节点报 L %s° / R %s° | 实测最大|q| L %s / R %s rad'
         % (hd.get('L'), hd.get('R'),
            None if q_now['L'] is None else round(q_now['L'], 4),
            None if q_now['R'] is None else round(q_now['R'], 4)),
         hd.get('L') is not None and hd.get('R') is not None
         and hd['L'] <= 3.0 and hd['R'] <= 3.0
         and q_now['L'] is not None and q_now['L'] < 0.06
         and q_now['R'] is not None and q_now['R'] < 0.06)

    return report(node, res)


def report(node, res):
    out = os.path.join(OUT_DIR, 'test_p101.json')
    os.makedirs(OUT_DIR, exist_ok=True)
    res['ok'] = not res['fails']
    with open(out, 'w', encoding='utf-8') as f:
        json.dump(res, f, ensure_ascii=False, indent=1)
    print('-' * 96)
    print('结论: %s   (%s)' % ('PASS' if res['ok'] else 'FAIL', out))
    if res['fails']:
        print('失败项: %s' % ', '.join(res['fails']))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if res['ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
