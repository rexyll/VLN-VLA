"""检查 ik_human 在"目标有扰动"时还能不能找到干净分支(离线, 不起仿真)。

运行时目标来自 gz 实测(杯子/底盘都有几毫米到二十几毫米的偏差), 若扫种子的
残差门限卡得太死(2mm), 干净分支会被判"不合格", 于是回落到 KDL 从当前位形
解出来的那条折叠分支 —— 这就是"抓杯成功后举杯顶到限位"的来源。
"""
import sys

import numpy as np

sys.path.insert(0, 'vla_ws/src/vla_follow')
from vla_follow import arm_kin as K          # noqa: E402
from vla_follow import drink_poses as P     # noqa: E402
from vla_follow import scene_layout as L    # noqa: E402


def sweep(side, T_loc, q0, tol):
    """返回 (干净解的个数, 最好的干净解, 它的残差)。

    种子形状随 `K.HUMAN_SEEDS` 走(P103 后它是 pan/lift/up_roll/elbow 四元组),
    这里按下标取, 多出来的维度自动跟上。
    """
    clean = []
    for v in K.HUMAN_SEEDS:
        s = K.Q_REST.copy()
        for i, x in enumerate(v):
            s[i] = x
        q, err, how = K.ik_best(side, T_loc, s)
        if err > tol:
            continue
        if K.violation_list(side, q):
            continue
        clean.append((K.human_penalty(side, q) + 0.08 * float(np.max(np.abs(q - q0))),
                      q, err))
    if not clean:
        return 0, None, None
    clean.sort(key=lambda c: c[0])
    return len(clean), clean[0][1], clean[0][2]


def main():
    bx, by, _ = L.SERVE_POSE
    base = K.base_pose(bx, by, 0.0)
    Tcp = P.cup_for_pour()
    T = P.cup_grasp_palm(Tcp, 'L')
    Tl = np.linalg.inv(base) @ T
    rng = np.random.default_rng(3)
    print('%6s %7s %7s %7s %7s' % ('tol', '有解', '残差mm', '肘y', 'up_roll'))
    for tol in (2e-3, 5e-3, 8e-3, 12e-3):
        hit = 0
        for _ in range(30):
            d = rng.normal(0.0, 0.008, 3)
            Tp = Tl.copy()
            Tp[:3, 3] += d
            n, q, err = sweep('L', Tp, K.Q_REST, tol)
            if n:
                hit += 1
        print('%6.0fmm %7d/30' % (tol * 1e3, hit))
    # 单个样例的细节
    for k in range(3):
        d = rng.normal(0.0, 0.008, 3)
        Tp = Tl.copy()
        Tp[:3, 3] += d
        print('扰动 %s mm' % np.round(d * 1e3, 1))
        for tol in (2e-3, 8e-3):
            n, q, err = sweep('L', Tp, K.Q_REST, tol)
            if q is None:
                print('  tol %4.0fmm: 无干净解' % (tol * 1e3))
            else:
                print('  tol %4.0fmm: %2d 个干净解, 选 残差 %.1fmm 肘y%+.0f '
                      'up_roll%+.2f 肘%+.2f'
                      % (tol * 1e3, n, err * 1e3,
                         K.elbow_pos('L', q)[1] * 1e3, q[2], q[3]))


if __name__ == '__main__':
    main()
