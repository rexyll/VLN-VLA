#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""probe_palm_world.py —— 读 /world/<w>/dynamic_pose/info, 打印指定实体的
世界位姿(模型)与相对模型位姿(link), 用于核对掌系各轴的**世界朝向**。

用法: python3 tests/diag/probe_palm_world.py [实体名...]
默认: robot R_palm L_palm R_L6 R_hand_index_1 R_hand_index_2
"""
import sys
import time

import gz.transport13 as gz_transport
import gz.msgs10.pose_v_pb2 as pose_v_pb2

WANT = sys.argv[1:] or ['robot', 'R_palm', 'L_palm', 'R_L6',
                        'R_hand_index_1', 'R_hand_index_2']
WORLD = '/world/follow_world/dynamic_pose/info'


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)


def qrot(q, v):
    w, x, y, z = q
    R = [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
         [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
         [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]
    return tuple(sum(R[i][j] * v[j] for j in range(3)) for i in range(3))


def main():
    found = {}
    node = gz_transport.Node()

    def cb(msg):
        for p in msg.pose:
            if p.name in WANT:
                found[p.name] = p
    node.subscribe(pose_v_pb2.Pose_V, WORLD, cb)
    t0 = time.time()
    while time.time() - t0 < 6.0 and len(found) < len(WANT):
        time.sleep(0.15)

    model = found.get('robot')
    mp = model.position if model else None
    mq = (model.orientation.w, model.orientation.x,
          model.orientation.y, model.orientation.z) if model else (1, 0, 0, 0)
    print('model robot: pos=(%.3f, %.3f, %.3f) quat=(%.3f %.3f %.3f %.3f)'
          % (mp.x, mp.y, mp.z, *mq) if model else 'model robot: 未找到')
    for name in WANT:
        p = found.get(name)
        if p is None:
            print('%-16s 未找到' % name)
            continue
        lp = (p.position.x, p.position.y, p.position.z)
        lq = (p.orientation.w, p.orientation.x,
              p.orientation.y, p.orientation.z)
        wp = tuple(lp[i] + qrot(mq, lp)[i] - lp[i] +
                   (0 if model is None else 0) for i in range(3))
        # 世界位姿 = T_model * T_link
        wp = tuple(mp_component + d for mp_component, d in
                   zip((mp.x, mp.y, mp.z) if model else (0, 0, 0), qrot(mq, lp)))
        wq = qmul(mq, lq)
        ax = {k: qrot(wq, v) for k, v in
              (('+x', (1, 0, 0)), ('+y', (0, 1, 0)), ('+z', (0, 0, 1)))}
        print('%-16s world pos=(%.4f, %.4f, %.4f)' % (name, *wp))
        print('%-16s   +x=(%+.2f %+.2f %+.2f)  +y=(%+.2f %+.2f %+.2f)  '
              '+z=(%+.2f %+.2f %+.2f)'
              % ('', *ax['+x'], *ax['+y'], *ax['+z']))
        print('%-16s   -z=(%+.2f %+.2f %+.2f)  (掌系 -z = 手指延伸方向)'
              % ('', *(-v for v in ax['+z'])))
    return 0


if __name__ == '__main__':
    sys.exit(main())
