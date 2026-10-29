"""看倒水(持壶)那个位形本身要求多大的腕屈伸 —— 判 R6 阈值定得是否合理。"""
import sys
import numpy as np
sys.path.insert(0, 'vla_ws/src/vla_follow')
from vla_follow import arm_kin as K, drink_poses as P, scene_layout as L

bx, by, _ = L.SERVE_POSE
base = (bx, by, 0.0)
Tcp = P.cup_for_pour()
Tkp, tip = P.kettle_for_pour(Tcp)
T = P.kettle_grasp_palm(Tkp)
Tl = np.linalg.inv(K.base_pose(*base)) @ T
rows = []
for v in (-1.6, -1.2, -0.8, -0.4, 0.0, 0.4, 0.8, 1.2, 1.6):
    for e in (-0.5, -1.1, -1.7):
        s = K.Q_REST.copy(); s[2], s[3] = v, e
        q, err, how = K.ik_best('R', Tl, s)
        if err > 2e-3:
            continue
        rows.append((abs(q[5]), q[5], q[2], q[3], q[4] + q[6], err * 1e3))
rows.sort()
print('壶嘴目标 %s' % (np.round(tip, 3),))
print('%7s %7s %7s %7s %8s %7s' % ('|wrist_p|', 'wrist_p', 'up_roll', 'elbow',
                                   'twist', 'err'))
for r in rows[:10]:
    print('%7.2f %7.2f %7.2f %7.2f %8.2f %7.2f' % r)
print('解数 %d, |wrist_p| 最小 %.2f 最大 %.2f' % (len(rows), rows[0][0], rows[-1][0]))
