#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""truth_probe.py —— 一次性真值探针: 直接订阅 dynamic_pose 打印关键实体。"""
import sys
import time

import gz.transport13 as gz_transport
import gz.msgs10.pose_v_pb2 as pose_v


def yaw_of(q):
    import math
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    want = ('robot', 'person', 'kettle', 'cup',
            'cup_water_disc', 'stream0', 'stream2', 'stream5')
    got = {}
    node = gz_transport.Node()

    def cb(msg):
        for p in msg.pose:
            if p.name in want and p.name not in got:
                got[p.name] = (p.position.x, p.position.y, p.position.z,
                               yaw_of(p.orientation))
    node.subscribe(pose_v.Pose_V,
                   '/world/follow_world/dynamic_pose/info', cb)
    t0 = time.time()
    n = 0
    while time.time() - t0 < 5.0:
        time.sleep(0.1)
    print('frames-with-targets: %d' % len(got))
    for k in want:
        if k in got:
            x, y, z, yw = got[k]
            print('%-8s (%+.3f, %+.3f, %+.3f) yaw=%+.3f' % (k, x, y, z, yw))
        else:
            print('%-8s <未收到>' % k)
    sys.exit(0)


if __name__ == '__main__':
    main()
