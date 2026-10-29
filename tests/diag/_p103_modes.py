"""离线比较三种逆解策略在"抓杯 → 举杯 → 倒水"这条链上的表现。

不起仿真: 用与运行时相同的目标位形, 种子链模拟 reach() 的 q_now 种子。
"""
import sys

import numpy as np

sys.path.insert(0, 'vla_ws/src/vla_follow')
from vla_follow import arm_kin as K          # noqa: E402
from vla_follow import drink_poses as P     # noqa: E402
from vla_follow import scene_layout as L    # noqa: E402


def seed_chain(mode, side, steps, base):
    q = K.Q_REST.copy()
    out = []
    for tag, T in steps:
        q, err, how = K.ik_world(side, T, base, q, elbow=mode)
        q = np.clip(q, K.LIM_LO, K.LIM_HI)
        m = K.elbow_metrics(side, q)
        out.append((tag, q, err * 1e3, how, m, K.human_penalty(side, q),
                    K.violation_list(side, q)))
    return out


def show(mode, side, steps, base):
    print('\n===== mode=%s  side=%s =====' % (mode, side))
    for tag, q, err, how, m, pen, bad in seed_chain(mode, side, steps, base):
        print('%-10s err %6.1fmm  肘y%+6.0f 掌y%+6.0f 外扩线%+6.0f droop%+6.0f '
              'up_roll%+5.2f 肘%+5.2f twist%+5.2f 代价%.2f %s'
              % (tag, err, K.elbow_pos(side, q)[1] * 1e3,
                 K.fk(side, q)[:3, 3][1] * 1e3, m['out_line_mm'],
                 m['droop_mm'], q[2], q[3], q[4] + q[6], pen,
                 ('★' + ' '.join(bad)) if bad else 'OK'))
        print('           q=%s  [%s]' % (np.round(q, 2), how))


def main():
    bx, by, _ = L.SERVE_POSE
    base = (bx, by, 0.0)
    cup0 = P.cup_on_table()
    Tcp = P.cup_for_pour()
    Tkp, _ = P.kettle_for_pour(Tcp)
    steps_L = [('接近杯', P.cup_grasp_palm(cup0, 'L') @ K.trans((0, 0, 0.10))),
               ('抓杯', P.cup_grasp_palm(cup0, 'L')),
               ('举杯', P.cup_grasp_palm(Tcp, 'L'))]
    steps_R = [('接近壶', P.kettle_grasp_palm(P.kettle_on_table())
                @ K.trans((0, 0, 0.10))),
               ('抓壶', P.kettle_grasp_palm(P.kettle_on_table())),
               ('倒水', P.kettle_grasp_palm(Tkp))]
    for mode in (None, 'out', 'human'):
        show(mode, 'L', steps_L, base)
        show(mode, 'R', steps_R, base)


if __name__ == '__main__':
    main()
