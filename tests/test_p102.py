#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_p102.py —— "放回→退到位→自然垂手" 与 "站远抬臂→驶入倒水" 的验收

用户 2026-10-28 的两条(接在 P101 的三条后面):

【A】放回水杯以后怎么收臂
    "机器人略微往后退一段距离, 使机器人与桌子的距离**大于机械臂+灵巧手整体
     长度**, 然后机械臂和灵巧手**从前方自然往下放**, 前臂以肘部为支点进行
     回收, 回到与上臂平行的状态, 让机械臂整体回到悬垂位置, 不能让机械臂做
     翻转等违背人体骨骼运动规律的动作。"

    判据:
      [2] 收臂**时**车离桌沿 ≥ 臂+手全长(0.715m) —— 这条同时证明了"先退后收"
          (老代码是先收臂再退, 那时车离桌沿只有 0.21m, 前臂绕肘的圆弧是在
          桌面上方画的);
      [3] 前臂以**肘尖为支点**: 折肘那一段肘点的世界位移 ≤ 5mm
          (`elbow_drift_mm`, 由 K.hang_path 的第①段直接量);
      [4] 收完是**悬垂**: 实测关节与 Q_HANG 的偏差 ≤ 3°(节点自报 + 本脚本
          独立复算两侧 max|q|);
      [2b] 撤手/倒车/收臂这一路**不许碰杯子**: 全程水平漂移 ≤ 15mm
          (老顺序"松钉→直接倒车"实测 47.9mm —— 手还捧在杯壁高度上, 车一退
          就贴着杯壁刮过去)。现在松手后先 `hands_off`(一起抬 90mm + 后撤
          100mm)再倒车;
      [5] 全程没有"翻转": 腕部关节总摆动只允许是"归零"那点量级(≤120°),
          而且关节是单调走向 0 的 —— 老实现直接插值到 Q_HANG 时, 腕部会在
          前臂还横着的时候先拧一圈(实测 1.2~1.5rad)。

【B】去倒水前的入场
    "在离桌子的距离略大于臂+手全长的位置时, 先让机器人将机械臂往上抬, 同时
     以肘部为支点进行弯曲, 将前臂和双手都抬高到与水壶/水杯同等高度, 然后再
     靠近桌子进行倒水的动作。"

    判据:
      [6] 抬臂发生在**臂长之外**: `entry.gap_before` ≥ 0.715m;
      [7] 双手真的抬到了器物高度: 预备位掌心 z 与 READY_Z(0.82m = 台面 0.72
          + 壶身把握)差 ≤ 20mm, 且 IK 残差 ≤ 5mm;
      [8] 抬完才靠近: `entry.gap_after` ≤ 0.30m(服务位是 0.21m), 且这段
          距离是**开过去**的(`drive_mm` ≤ 30mm 残差, 说明真的闭环行驶了)。

先起场景和节点(与 test_p101 相同):
    bash tools/start_scene.sh
    bash tools/start_drink_demo.sh
    python3 tests/test_p102.py

产物: vla_ws/logs/p102_*.png + tests/out/test_p102.json
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
from vla_follow import scene_layout as L      # noqa: E402
from vla_follow.drink_task import DrinkTask   # noqa: E402
from test_p101 import Watch, layout_scene     # noqa: E402

OUT_DIR = os.path.join(ROOT, 'tests', 'out')
TOTAL_TIMEOUT = 320.0


def main():
    rclpy.init()
    node = DrinkTask(observer_only=True)
    w = Watch(node)
    fails = []
    res = {'ok': False, 'fails': fails, 'shots': []}

    def step(tag, ok, detail=''):
        print('%s %-58s %s%s' % ('[OK]' if ok else '[NG]', tag,
                                 'OK' if ok else 'NG',
                                 ('  ' + detail) if detail else ''))
        if not ok:
            fails.append(tag)

    print('=' * 96)
    print('P102 验收 —— 退到位再自然垂手 / 站远抬臂再驶入倒水')
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
        if w.state.startswith('FAIL') or w.state == 'ERROR':
            w.spin(1.0)
            break
        if w.state in ('PRESENT', 'DONE') and w.state not in seen:
            seen.add(w.state)
            p = w.shot(w.state.lower(), prefix='p102')
            if p:
                res['shots'].append(p)
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

    # ---- [A] 退到位再收臂 --------------------------------------------------
    hp = r.get('hang_path') or {}
    rg = r.get('retreat_geom') or {}
    gaps = {s: (hp.get(s) or {}).get('gap_m') for s in ('L', 'R')}
    reach = K.ARM_REACH
    step('[2] 收臂时车在臂长之外  L %s / R %s m(臂+手 %.3fm, 判据 ≥ %.3f)'
         % (gaps['L'], gaps['R'], reach, reach),
         r.get('hang_path') is not None
         and all(v is not None and v >= reach for v in gaps.values()),
         '退到位后 gap %.3fm(余量 %+.0fmm)'
         % (rg.get('gap_m', -1), rg.get('clear_mm', 0)))

    # 倒车 + 收臂这一路**不许碰杯子**。杯子是无重力自由体, 手一蹭就滑走:
    # 老顺序(松钉→直接倒车)实测带偏 47.9mm, 画面上像"机器人一退, 杯子跟着
    # 滑一下"。现在松手后先 `hands_off`(两手分别向两侧张开 120mm、再抬 120mm)
    # 再倒车 —— 外张而不是后撤, 理由见 drink_motion.hands_off 的实测。
    cd_mm = rg.get('cup_drive_mm')
    ch_mm = rg.get('cup_hang_mm')
    ct_mm = rg.get('cup_total_mm')
    pb = r.get('putback') or {}
    step('[2b] 撤手/倒车/收臂都没碰杯子  倒车段 %s / 收臂段 %s / 全程 %s mm'
         ' (判据 ≤15)'
         % (cd_mm, ch_mm, ct_mm),
         ct_mm is not None and ct_mm <= 15.0,
         '撤手: 外张 %s mm + 抬 %s mm(残差 %s mm)'
         % (pb.get('handoff_out_mm'), pb.get('handoff_up_mm'),
            pb.get('handoff_mm')))

    drifts = {s: (hp.get(s) or {}).get('elbow_drift_mm') for s in ('L', 'R')}
    folds = {s: (hp.get(s) or {}).get('fold_deg') for s in ('L', 'R')}

    step('[3] 前臂绕肘回收(肘点不动)  L %s / R %s mm(判据 ≤5)'
         % (drifts['L'], drifts['R']),
         all(v is not None and v <= 5.0 for v in drifts.values()),
         '折肘 L %s° / R %s°, 路点 %s/%s 段'
         % (folds['L'], folds['R'], (hp.get('L') or {}).get('waypoints'),
            (hp.get('R') or {}).get('waypoints')))

    hd = r.get('hang_deg') or {}
    q_now = {}
    for s in ('R', 'L'):
        q = M.q_meas(node, s)
        q_now[s] = None if q is None else float(max(abs(v) for v in q))
    step('[4] 收完是悬垂  节点报 L %s° / R %s° | 实测 max|q| L %s / R %s rad'
         % (hd.get('L'), hd.get('R'),
            None if q_now['L'] is None else round(q_now['L'], 4),
            None if q_now['R'] is None else round(q_now['R'], 4)),
         hd.get('L') is not None and hd.get('R') is not None
         and hd['L'] <= 3.0 and hd['R'] <= 3.0
         and q_now['L'] is not None and q_now['L'] < 0.06
         and q_now['R'] is not None and q_now['R'] < 0.06)

    # 腕部: 判"**没有一步翻过去**"。`wrist_step_deg` 是相邻路点之间腕部三个
    # 关节的最大变化量 —— 直接插值到 Q_HANG 时前臂还横着腕就先拧 1.2rad,
    # 那一步就是 70°, 画面上是手腕拧一圈再甩下来。现在把腕的归零放在第①段
    # (上臂不动、掌心贴近身体)并按 0.30rad 限步, 单步不超过 ~17°。
    wd = {s: (hp.get(s) or {}).get('wrist_deg') for s in ('L', 'R')}
    ws = {s: (hp.get(s) or {}).get('wrist_step_deg') for s in ('L', 'R')}
    step('[5] 收臂过程无翻转(腕单步 ≤30°)  L %s° / R %s°(总摆 %s° / %s°)'
         % (ws['L'], ws['R'], wd['L'], wd['R']),
         all(v is not None and v <= 30.0 for v in ws.values()),
         '起手 q0 L %s / R %s, 掌心 %s / %s'
         % ((hp.get('L') or {}).get('q0'), (hp.get('R') or {}).get('q0'),
            (hp.get('L') or {}).get('palm0_m'),
            (hp.get('R') or {}).get('palm0_m')))

    # ---- [B] 站远抬臂再驶入 ------------------------------------------------
    en = r.get('entry') or {}
    gb = en.get('gap_before')
    ga = en.get('gap_after')
    step('[6] 抬臂发生在臂长之外  gap %s m(臂+手 %.3fm, 判据 ≥ %.3f)'
         % (gb, reach, reach), gb is not None and gb >= reach,
         '入场起点 gap_at_arrival %s m' % en.get('gap_at_arrival'))

    lift = en.get('lift') or {}
    le = [lift.get(s) for s in ('L', 'R')]
    step('[7] 双手抬到器物高度(残差 ≤5mm)  L %s / R %s mm'
         % (le[0], le[1]),
         all(v is not None and v <= 5.0 for v in le),
         '预备位掌心 z=%.2fm(台面 %.2f + 壶把握), 前伸 %.2fm, 左右 %.3f'
         % (K.READY_Z, L.TABLE_TOP_Z, K.READY_R, K.READY_Y))

    step('[8] 抬完才靠近(驶入而非瞬移)  gap %s m(判据 ≤0.30) 行驶残差 %s mm'
         % (ga, en.get('drive_mm')),
         ga is not None and ga <= 0.30
         and en.get('drive_mm') is not None and en.get('drive_mm') <= 30.0,
         '服务位 gap %.2fm, 到位微调 %s mm'
         % (L.table_clearance(L.SERVE_POSE[0], L.SERVE_POSE[1]),
            en.get('snap_mm')))

    return report(node, res)


def report(node, res):
    out = os.path.join(OUT_DIR, 'test_p102.json')
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
