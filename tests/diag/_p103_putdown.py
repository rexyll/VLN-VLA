"""离线模拟"双手放回"的下降路径: 直线 vs 先伸出去再落下(离线, 不起仿真)。

实测问题: 捧杯(carry, 弯着肘)直接往下走到桌面, 同一条分支到不了 —— 腕屈伸
被顶到限位(1.50)、肘折到 −2.30、前臂拧到 −2.27。人放杯子不是这样: 先把手臂
**伸出去**(肘打开), 再往下放到桌面, 全程手臂是舒展的。

本脚本从实测的 carry 关节角出发, 用 `arm_kin.ik_human` 逐步解算两条路径:
    直线: 物体从 carry 位直接线性到桌面目标
    弧线: carry → 前方偏外/偏高的一点(肘打开) → 桌面目标
并打印每步的关节与人体规则违反情况。
"""
import sys

import numpy as np

sys.path.insert(0, 'vla_ws/src/vla_follow')
from vla_follow import arm_kin as K          # noqa: E402
from vla_follow import drink_poses as P     # noqa: E402
from vla_follow import scene_layout as L    # noqa: E402

Q_CARRY_L = np.array([0.604, -1.555, -1.382, -1.872, 0.071, 1.408, -1.758])


def blend(Ta, Tb, a):
    Ra, Rb = np.asarray(Ta)[:3, :3], np.asarray(Tb)[:3, :3]
    d = np.asarray(Tb)[:3, 3] - np.asarray(Ta)[:3, 3]
    R = Ra.T @ Rb
    c = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
    ang = np.arccos(c)
    if ang < 1e-8:
        Rn = Ra
    else:
        ax = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0],
                       R[1, 0] - R[0, 1]]) / (2.0 * np.sin(ang))
        Rn = Ra @ K.rot(tuple(ax), ang * a)
    T = np.eye(4)
    T[:3, :3] = Rn
    T[:3, 3] = np.asarray(Ta)[:3, 3] + d * a
    return T


def run(tag, pts, q0, base, off):
    print('--- %s' % tag)
    q = q0.copy()
    for i, Tobj in enumerate(pts):
        Tp = Tobj @ off
        q, err, how = K.ik_world('L', Tp, base, q, elbow='human')
        m = K.elbow_metrics('L', q)
        bad = K.violation_list('L', q)
        print('  %d 物体%s → 掌%s  IK %.1fmm 肘%+.2f up%+.2f tw%+.2f wp%+.2f '
              '外扩%+.0f %s'
              % (i, np.round(Tobj[:3, 3], 3), np.round(Tp[:3, 3], 3),
                 err * 1e3, q[3], q[2], q[4] + q[6], q[5], m['out_line_mm'],
                 ('★' + ' '.join(bad)) if bad else 'OK'))


def main():
    bx, by, _ = L.SERVE_POSE
    base = (bx, by, 0.0)
    Tcarry = P.carry_cup(bx, by, 0.0)
    Ttable = P.cup_on_table()
    # 冻结偏移: 用名义抓握几何(实测与名义差 <1°)
    off = np.linalg.inv(P.cup_cradle_palm(Tcarry, 'L')) @ Tcarry

    run('直线(现状)', [blend(Tcarry, Ttable, i / 4.0) for i in range(1, 5)],
        Q_CARRY_L, base, off)

    # 弧线: 先在"更前、更高"处把肘打开, 再落桌
    Pmid = np.asarray(Ttable, float).copy()
    Pmid[:3, 3] = np.asarray(Ttable)[:3, 3] + np.array([0.075, 0.0, 0.115])
    pts = [blend(Tcarry, Pmid, i / 2.0) for i in (1, 2)]
    pts += [blend(Pmid, Ttable, i / 3.0) for i in (1, 2, 3)]
    run('弧线(先伸出去再落)', pts, Q_CARRY_L, base, off)


if __name__ == '__main__':
    main()
