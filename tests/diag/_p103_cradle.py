"""捧杯(carry)/桌面抓杯两个位形在不同分支下的腕屈伸与肘折叠(离线)。"""
import sys
import numpy as np
sys.path.insert(0, 'vla_ws/src/vla_follow')
from vla_follow import arm_kin as K, drink_poses as P, scene_layout as L

bx, by, _ = L.SERVE_POSE
base = K.base_pose(bx, by, 0.0)
Tcarry = P.carry_cup(bx, by, 0.0)
cases = [('L 捧杯', 'L', P.cup_cradle_palm(Tcarry, 'L')),
         ('R 捧杯', 'R', P.cup_cradle_palm(Tcarry, 'R')),
         ('L 桌面抓杯', 'L', P.cup_grasp_palm(P.cup_on_table(), 'L'))]
for tag, side, T in cases:
    Tl = np.linalg.inv(base) @ T
    rows = []
    for v in (-1.6, -1.2, -0.8, -0.4, 0.0, 0.4, 0.8, 1.2, 1.6):
        for e in (-0.3, -0.8, -1.3, -1.8):
            s = K.Q_REST.copy(); s[2], s[3] = v, e
            q, err, how = K.ik_best(side, Tl, s)
            if err > 2e-3:
                continue
            m = K.elbow_metrics(side, q)
            rows.append((abs(q[5]), q[5], q[2], q[3], q[4] + q[6],
                         m['out_line_mm'], m['droop_mm']))
    rows.sort()
    print('--- %s: %d 个合理解' % (tag, len(rows)))
    print('  %7s %7s %7s %7s %8s %7s %7s'
          % ('|wrist_p|', 'wrist_p', 'up_roll', 'elbow', 'twist', '外扩', 'droop'))
    for r in rows[:5]:
        print('  %7.2f %7.2f %7.2f %7.2f %8.2f %7.0f %7.0f' % r)
