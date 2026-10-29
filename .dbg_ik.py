import sys, os, math
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'vla_ws/src/vla_follow'))
import numpy as np
from vla_follow import arm_kin as K, drink_poses as P, scene_layout as L

print('Q_REST', K.Q_REST, 'lim', K.LIM_LO, K.LIM_HI)
T0 = K.fk('R', K.Q_REST)
print('fk(Q_REST) t=', T0[:3,3])
Tgt = P.kettle_grasp_palm(P.kettle_on_table())
print('target     t=', Tgt[:3,3])
print('dist shoulder->target', np.linalg.norm(Tgt[:3,3]-np.array(K.SHOULDER['R'])))
e = K.pose_err(T0, Tgt)
print('err0', e, np.linalg.norm(e))
q, err = K.ik('R', Tgt, K.Q_REST)
print('dls q', np.round(q,3), 'err', err, 'pos', K.fk('R',q)[:3,3])
if K.HAVE_KDL:
    print('kdl lma', K.ik_kdl('R', Tgt, K.Q_REST, 'lma'))
    print('kdl nrjl', K.ik_kdl('R', Tgt, K.Q_REST, 'nr_jl'))
