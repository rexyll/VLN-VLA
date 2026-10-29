#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_p103.py —— 全程"人体关节规则"验收(倒水肘内收的回归)

用户 2026-10-28:
    "从水壶倒水的时候, 机器人左手臂肘部弯向内侧, 这与人体骨骼运动规则不符合,
     修复这个问题。另外检查其他运动关节是否有违背人体肢体正常运动轨迹的
     现象, 如果有, 一起修复。"

**病根不是"肘这个关节", 而是 7 轴臂对 6 维位姿冗余的那一维没人管**: 同一只
手要到同一处, 有整族等价解, KDL 只按种子收敛, 于是它给出
    举杯时 L q=[0.12, −0.61, −0.41, −1.64, −0.32, +0.71, −2.67]
—— 肘贴着身体(相对肩只外扩 22mm)、前臂反拧 **171°**、肘尖垂在掌下 219mm。
这个位形位置/姿态全对, 闭环报"到位 1.7mm", 但画面上就是"肘拐进怀里、手腕
拧了一圈"。修法见 `arm_kin.ik_human` / `human_penalty` / `violation_list`:
候选解扫一遍, 按人体代价挑; 并且**只在进入动作时定一次分支**(迟滞),
否则闭环补位会在两条等价分支之间来回挪, 把 up_roll/elbow 顶到限位。

本脚本判的就是那七条规则, 在**整条动作链的每个关键时刻**都量一遍:

    R1 肘反关节   elbow > 0
    R2 肘过身体中线
    (原 R3"手过身体中线"已删 —— 杯子落点会偏基座中线 ±110mm, 左手捧杯
     必然跨过中线; "手拐进怀里"由 R2 + out_line 兜住, 那两条是相对**同侧肩**
     量的)
    R4 肩内旋过大 |up_roll| > 1.95 rad(112°, 机械限位 2.00, 见 arm_kin 的校准依据)
    R5 前臂反拧   |fore_roll + wrist_r| > 2.00 rad(115°)
    R6 腕反折     |wrist_p| > 1.50 rad(86°, 即机械限位)
    R7 肘过折     |elbow| > 2.10 rad(120°)

判据:
  [1] 任务跑完(否则后面没有数据)
  [2] 倒水段(抓杯→举杯→倒水)两侧 0 次违反
  [3] 举杯时**左肘朝外**: out_line ≥ 100mm 且 droop ≤ 120mm 且 |twist| ≤ 2.2
      —— 这条是用户指的那个动作, 老实现是 49mm / 219mm / 2.94
  [4] 倒水时**右肘**朝外: out_line ≥ 60mm(老实现 82mm, 但腕反折 1.50)
  [5] 捧杯 / 放回 / 撤手 / 悬垂四个静止姿态 0 次违反
      ("撤手"= 松钉后把手掌平移离开杯子那一小段, 以前没人量)
  [6] 全流程 trace(每个检查点)累计 0 次违反
  [7] 收尾悬垂: 实测 max|q| ≤ 0.10rad(与 P102 [4] 同口径)

用法(世界与节点都在跑):
    bash tools/start_scene.sh && bash tools/start_drink_demo.sh
    source /opt/ros/jazzy/setup.bash; source vla_ws/install/setup.bash
    python3 tests/test_p103.py

产物: tests/out/test_p103.json
本脚本只读(observer_only=True), 不参与控制。
"""
import json
import os
import sys
import time

import rclpy

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'vla_ws', 'src', 'vla_follow'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from vla_follow import arm_kin as K            # noqa: E402
from vla_follow import drink_motion as M      # noqa: E402
from vla_follow.drink_task import DrinkTask   # noqa: E402
from test_p101 import Watch, layout_scene     # noqa: E402

OUT_DIR = os.path.join(ROOT, 'tests', 'out')
TOTAL_TIMEOUT = 380.0


def main():
    rclpy.init()
    node = DrinkTask(observer_only=True)
    w = Watch(node)
    fails = []
    res = {'ok': False, 'fails': fails}

    def step(tag, ok, detail=''):
        print('%s %-62s %s%s' % ('[OK]' if ok else '[NG]', tag, 'OK' if ok else 'NG',
                                 ('  ' + detail) if detail else ''))
        if not ok:
            fails.append(tag)

    print('=' * 96)
    print('P103 验收 —— 全程关节姿态是否符合人体运动规则(倒水左肘内收回归)')
    print('=' * 96)

    t0 = time.time()
    while time.time() - t0 < 20.0 and M.object_pose(node, 'cup') is None:
        w.spin(0.3)
    if M.object_pose(node, 'cup') is None:
        step('[0] 数据链路(真值)', False, '没有 cup 真值')
        return report(node, res)

    layout_scene(node, w, quiet=False)
    w.reset()
    w.send('GO_DRINK')
    t_wait = time.time()
    while time.time() - t_wait < 90.0 and w.state != 'PREPARE' \
            and not w.state.startswith('FAIL'):
        w.spin(0.3)
    if w.state != 'PREPARE':
        print('    人的步行 90s 内没把任务带起来(当前 %s), 直接下 DRINK'
              % w.state)
        w.send_robot('DRINK')
        w.spin(2.0)

    t0 = time.time()
    while time.time() - t0 < TOTAL_TIMEOUT:
        w.spin(0.2)
        if w.state.startswith('FAIL') or w.state == 'ERROR':
            w.spin(1.0)
            break
        if w.state == 'DONE':
            w.spin(1.0)
            t1 = time.time()
            while time.time() - t1 < 15.0 and w.state != 'IDLE':
                w.spin(0.2)
            w.spin(1.0)
            break

    step('[1] 任务跑完(%s)' % ' → '.join(w.stages[-4:]), 'DONE' in w.stages,
         'stages=%d' % len(w.stages))
    r = w.result
    res['result'] = r
    arm = r.get('arm') or {}
    trace = r.get('arm_trace') or {}

    def bad_of(d):
        out = []
        for s, st in (d or {}).items():
            # 形状容错: 只认 {side: state} 的嵌套; 早期版本把**单侧** state 直接
            # 塞进来过(键是 q/bad/...), 那种条目跳过而不是崩掉整个验收。
            if not isinstance(st, dict) or 'bad' not in st:
                continue
            for b in st.get('bad') or []:
                out.append('%s.%s' % (s, b))
        return out

    # ---- [2] 倒水段 -------------------------------------------------------
    seg = {k: arm.get(k) for k in ('grasp_cup', 'lift_cup', 'pour')}
    seg_bad = []
    for k, d in seg.items():
        for b in bad_of(d):
            seg_bad.append('%s:%s' % (k, b))
    step('[2] 倒水段(抓杯/举杯/倒水)两侧无违反人体规则',
         not seg_bad and all(v for v in seg.values()),
         ('★ ' + ' '.join(seg_bad)) if seg_bad else '7 条规则全过')

    # ---- [3] 举杯时左肘朝外 ------------------------------------------------
    lf = (arm.get('lift_cup') or {}).get('L') or {}
    step('[3] 举杯时左肘朝外  外扩线 %s / droop %s / twist %s'
         ' (判据 ≥100 / ≤120 / ≤2.2)'
         % (lf.get('out_line_mm'), lf.get('droop_mm'), lf.get('twist')),
         (lf.get('out_line_mm') or 0) >= 100.0
         and abs(lf.get('droop_mm') or 9e9) <= 120.0
         and abs(lf.get('twist') or 9e9) <= 2.2,
         '肘y %+.0fmm 掌y %+.0fmm up_roll %+.2f'
         % (lf.get('elbow_mm') or 0, lf.get('palm_mm') or 0,
            lf.get('up_roll') or 0))

    # ---- [4] 倒水时右肘 ----------------------------------------------------
    rf = (arm.get('pour') or {}).get('R') or {}
    step('[4] 倒水时右肘朝外  外扩线 %s(判据 ≥60)' % rf.get('out_line_mm'),
         (rf.get('out_line_mm') or 0) >= 60.0,
         'twist %+.2f up_roll %+.2f 肘y %+.0fmm'
         % (rf.get('twist') or 0, rf.get('up_roll') or 0,
            rf.get('elbow_mm') or 0))

    # ---- [5] 三个静止姿态 --------------------------------------------------
    still = {k: bad_of(arm.get(k)) for k in ('捧杯', '放回', '撤手', '悬垂')}
    still_bad = ['%s:%s' % (k, b) for k, v in still.items() for b in v]
    step('[5] 捧杯/放回/撤手/悬垂 四个静止姿态无违反',
         all(arm.get(k) for k in ('捧杯', '放回', '撤手', '悬垂')) and not still_bad,
         ('★ ' + ' '.join(still_bad)) if still_bad else
         '捧杯 L %s/R %s mm 外扩' % (
             ((arm.get('捧杯') or {}).get('L') or {}).get('out_line_mm'),
             ((arm.get('捧杯') or {}).get('R') or {}).get('out_line_mm')))

    # ---- [6] 全流程 trace --------------------------------------------------
    all_bad = []
    for tag, d in sorted(trace.items()):
        for b in bad_of(d):
            all_bad.append('%s:%s' % (tag, b))
    step('[6] 全流程 %d 个检查点零违反' % len(trace),
         bool(trace) and not all_bad,
         ('★ ' + ' '.join(sorted(set(all_bad)))) if all_bad else
         '检查点: ' + ' '.join(sorted(trace)))

    # ---- [7] 收尾悬垂 ------------------------------------------------------
    q_max = {}
    for s in ('R', 'L'):
        q = M.q_meas(node, s)
        q_max[s] = None if q is None else round(float(max(abs(v) for v in q)), 3)
    step('[7] 收尾悬垂  L %s / R %s rad(判据 ≤0.10)'
         % (q_max['L'], q_max['R']),
         all(v is not None and v <= 0.10 for v in q_max.values()))

    return report(node, res)


def report(node, res):
    out = os.path.join(OUT_DIR, 'test_p103.json')
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
