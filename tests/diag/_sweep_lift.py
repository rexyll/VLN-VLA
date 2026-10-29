import sys, numpy as np
sys.path.insert(0, 'vla_ws/src/vla_follow')
from vla_follow import arm_kin as K, drink_poses as P, scene_layout as L

bx, by, _ = L.SERVE_POSE
base = (bx, by, 0.0)
Tcp = P.cup_for_pour()
T = P.cup_grasp_palm(Tcp, 'L')
Tl = np.linalg.inv(K.base_pose(*base)) @ T
print('杯位(世界)', np.round(Tcp[:3, 3], 3), ' 掌心目标(本体)', np.round(Tl[:3, 3], 3))
rows = []
for v in (-2.0, -1.7, -1.5, -1.2, -0.9, -0.6, -0.3, 0.0, 0.3, 0.6, 0.9, 1.2, 1.5, 1.7, 2.0):
    for el in (0.0, -0.4, -0.8):
        s = K.Q_REST.copy(); s[2] = v; s[3] = el
        q, err, how = K.ik_best('L', Tl, s)
        if err > 2e-3:
            continue
        m = K.elbow_metrics('L', q)
        el_p = K.elbow_pos('L', q)
        palm = K.fk('L', q)[:3, 3]
        rows.append((m['out_line_mm'], el_p[1] * 1e3, palm[1] * 1e3, q[2], q[3],
                     q[4] + q[6], q[5], err * 1e3))
rows.sort(key=lambda r: -r[0])
print('%7s %8s %8s %7s %7s %9s %7s' % ('out_line', 'elbow_y', 'palm_y',
                                       'up_roll', 'elbow', 'fore+wrst', 'wrst_p'))
for r in rows[:16]:
    print('%7.1f %8.1f %8.1f %7.2f %7.2f %9.2f %7.2f' % r[:7])
print('总解数', len(rows))
