import faulthandler, numpy as np
faulthandler.enable()
from vla_follow import arm_kin as K, drink_poses as P, scene_layout as L

base = (L.SERVE_POSE[0], L.SERVE_POSE[1], 0.0)
Tb = K.base_pose(*base)
Tk = K.frame(K.rot_z(np.deg2rad(90.0)),
             (L.KETTLE_XY[0], L.KETTLE_XY[1], L.KETTLE_CZ))
Tg = P.kettle_grasp_palm(Tk)
Tl = np.linalg.inv(Tb) @ Tg
print('壶 yaw=90 时:')
print('  抓取掌心目标(世界)', np.round(Tg[:3, 3], 4))
print('  目标(本体)      ', np.round(Tl[:3, 3], 4))
print('  目标-肩距离      %.4f m' % np.linalg.norm(Tl[:3, 3] - np.array(K.SHOULDER['R'])))

Trest = K.fk('R', K.Q_REST)
print('  静止掌心(本体)  ', np.round(Trest[:3, 3], 4),
      ' 到目标 %.4f m' % np.linalg.norm(Trest[:3, 3] - Tl[:3, 3]))

def show(tag, q, e):
    Fe = K.fk('R', q)
    ep = np.linalg.norm(Fe[:3, 3] - Tl[:3, 3])
    er = np.linalg.norm(K.pose_err(Fe, Tl)[3:])
    over = np.maximum(K.LIM_LO - q, 0) + np.maximum(q - K.LIM_HI, 0)
    print('  %-10s e=%.4f  pos=%.4f m  rot=%.2f°  越限 %.2f rad'
          % (tag, e, ep, np.degrees(er), np.max(over)))
    print('             q=', np.round(q, 3))

q, e = K.ik('R', Tl, K.Q_REST);            show('dls', q, e)
q2, e2 = K.ik_kdl('R', Tl, K.Q_REST, 'lma'); show('kdl-lma', q2, e2)
