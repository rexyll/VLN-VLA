"""离线穷举: 倒水(持壶 R)与放回(装杯 L)两个位形, 到底有没有"既合人体规则
又肘朝外"的解?

背景(P103 验收第二次跑):
  [4] 倒水时右肘 out_line = +40mm(判据 ≥60), 但 up_roll/twist 全合规;
  [5] 放回时左 up_roll = −1.90(规则上限 1.75), 但外扩 216mm 很漂亮。
两者都是"合规集合里外扩不够 / 外扩集合里不合规"的取舍问题。到底是
候选解没扫到, 还是几何上就没有, 这一支脚本回答它: 用**稠密种子网格**
把 pan/lift/up_roll/elbow 四个维度的初值都撒开, 收敛后按 (up_roll, elbow,
twist) 去重, 每个分支打印度量。

用法: python3 tests/diag/_p103_branch_sweep.py
"""
import sys

import numpy as np

sys.path.insert(0, 'vla_ws/src/vla_follow')
from vla_follow import arm_kin as K, drink_poses as P, scene_layout as L  # noqa: E402

PANS = (-0.9, -0.3, 0.3, 0.9)
LIFTS = (-1.2, -0.6, 0.0, 0.6)
UROLLS = (-1.8, -1.3, -0.8, -0.3, 0.0, 0.3, 0.8, 1.3, 1.8)
ELBOWS = (-0.4, -1.0, -1.6, -2.1)


def sweep(side, T_local, tag):
    seen = {}
    for p in PANS:
        for lf in LIFTS:
            for v in UROLLS:
                for e in ELBOWS:
                    s = K.Q_REST.copy()
                    s[0], s[1], s[2], s[3] = p, lf, v, e
                    q, err, _how = K.ik_best(side, T_local, s)
                    if err > 2e-3:
                        continue
                    q = np.clip(q, K.LIM_LO, K.LIM_HI)
                    err2 = float(np.linalg.norm(
                        K.pose_err(K.fk(side, q), T_local)))
                    if err2 > 2e-3:
                        continue
                    key = tuple(np.round(q / 0.08).astype(int))
                    if key in seen:
                        continue
                    m = dict(K.elbow_metrics(side, q))
                    m.update(
                        q=q, err=err2, cost=K.human_penalty(side, q),
                        bad=K.violation_list(side, q),
                        twist=float(q[4] + q[6]), elbow=float(q[3]),
                        wrist_p=float(q[5]))
                    seen[key] = m
    rows = list(seen.values())
    rows.sort(key=lambda r: r['cost'])
    print('=' * 96)
    print('%s  独立分支 %d 个   合规 %d 个'
          % (tag, len(rows), sum(1 for r in rows if not r['bad'])))
    print('%6s %7s %7s %8s %8s %8s %8s  %s'
          % ('代价', '外扩线', '基外扩', 'droop', 'twist', 'up_roll', 'elbow',
             '违反/构型'))
    for r in rows[:12]:
        print('%6.3f %+7.0f %+7.0f %+8.0f %+8.2f %+8.2f %+8.2f  %s'
              % (r['cost'], r['out_line_mm'], r['out_base_mm'],
                 r['droop_mm'], r['twist'], r['up_roll'], r['elbow'],
                 ('★' + ' '.join(r['bad'])) if r['bad'] else 'OK'))
        print('        q=%s' % np.round(r['q'], 2))
    ok = [r for r in rows if not r['bad']]
    if ok:
        best = max(ok, key=lambda r: r['out_line_mm'])
        print('合规解里外扩线最大: %+.0fmm (droop %+.0f, twist %+.2f, '
              'up_roll %+.2f)' % (best['out_line_mm'], best['droop_mm'],
                                  best['twist'], best['up_roll']))
    else:
        print('★ 一个合规解都没有')
    return rows


def main():
    bx, by, _ = L.SERVE_POSE
    Tb = K.base_pose(bx, by, 0.0)
    Tcp = P.cup_for_pour()
    Tkp, tip = P.kettle_for_pour(Tcp)
    print('壶嘴目标 %s' % np.round(tip, 3))
    sweep('R', np.linalg.inv(Tb) @ P.kettle_grasp_palm(Tkp), '倒水: R 持壶')
    # 放回: 杯在桌面上方一点(装杯位)时的左掌目标
    Tcu = P.cup_for_pour()
    Tcu[2, 3] = P.TABLE_Z + 0.0
    sweep('L', np.linalg.inv(Tb) @ P.cup_grasp_palm(Tcu, 'L'), '放回: L 持杯')


if __name__ == '__main__':
    main()
