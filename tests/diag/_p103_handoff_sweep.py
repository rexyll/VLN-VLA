"""量"撤手"目标的可行域: 放回位平移 (back, up), 哪些组合存在**合规**解。

判据: 4mm 残差内、R1~R7 全过的候选条数 > 0, 且最优代价可接受。
用法: python3 tests/diag/_p103_handoff_sweep.py [tests/out/test_p103.json]
"""
import json
import sys

import numpy as np

sys.path.insert(0, 'vla_ws/src/vla_follow')
from vla_follow import arm_kin as K  # noqa: E402


def solve(side, T, tol=4e-3):
    """返回 (合规候选条数, 最优代价, 最优解) —— 四维种子全撒。"""
    ok = []
    for v in K.HUMAN_SEEDS:
        s = K.Q_REST.copy()
        s[0], s[1], s[2], s[3] = v
        q, err, _h = K.ik_best(side, T, s)
        if err > tol:
            continue
        q = np.clip(q, K.LIM_LO, K.LIM_HI)
        if K.violation_list(side, q):
            continue
        ok.append((K.human_penalty(side, q), q))
    ok.sort(key=lambda c: c[0])
    return len(ok), (ok[0][0] if ok else float('nan')), (ok[0][1] if ok else None)


def relax(T, tilt):
    """把掌心姿态朝'手心朝下'的中立姿态转 tilt 比例(撤手时手会自然松下来)。"""
    from scipy.spatial.transform import Rotation as R
    R0 = np.asarray(T[:3, :3], float)
    # 中立: 掌心朝下、手指朝前(只保留原姿态的航向角)
    yaw = np.arctan2(R0[1, 0], R0[0, 0])
    Rn = (R.from_euler('z', yaw) * R.from_euler('x', np.pi)).as_matrix()
    T2 = np.asarray(T, float).copy()
    M = (1.0 - tilt) * R0 + tilt * Rn
    u, _s, vt = np.linalg.svd(M)          # 正交化, 保证仍是旋转
    T2[:3, :3] = u @ vt
    return T2


def main():
    p = sys.argv[1] if len(sys.argv) > 1 else 'tests/out/test_p103.json'
    arm = json.load(open(p, encoding='utf-8'))['result']['arm']
    grid_back = (0.10, 0.12, 0.14, 0.16)
    grid_up = (0.00, 0.03, 0.05, 0.07, 0.09)
    sides = sys.argv[2:] or ['L', 'R']
    for side in sides:
        st = (arm.get('放回') or {}).get(side)
        if not st:
            continue
        q0 = np.asarray(st['q'], float)
        T = K.fk(side, q0)
        print('=' * 76)
        print('放回 %s  起点 up=%+.2f el=%+.2f wp=%+.2f'
              % (side, q0[2], q0[3], q0[5]))
        print('%-8s' % 'back\\up', end='', flush=True)
        for du in grid_up:
            print('%12s' % ('%.2f' % du), end='')
        print('   ← 单元格: 合规条数/最优代价')
        for db in grid_back:
            print('%-8.2f' % db, end='')
            for du in grid_up:
                T2 = T.copy()
                T2[:3, 3] = T2[:3, 3] + np.array([-db, 0.0, du])
                n, cost, q = solve(side, T2)
                print('%12s' % ('%d/%.2f' % (n, cost) if n else '0'), end='', flush=True)
            print(flush=True)
        if '--relax' not in sys.argv:
            continue
        # 姿态放松 35% 之后再看同一网格
        print('-- 掌心姿态朝"松手"松弛 35%')
        for db in grid_back:
            print('%-8.2f' % db, end='')
            for du in grid_up:
                T2 = T.copy()
                T2[:3, 3] = T2[:3, 3] + np.array([-db, 0.0, du])
                T2 = relax(T2, 0.35)
                n, cost, q = solve(side, T2)
                print('%12s' % ('%d/%.2f' % (n, cost) if n else '0'), end='', flush=True)
            print()


if __name__ == '__main__':
    main()
