#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""qpose.py —— 一秒钟读一次关键实体的世界坐标(排错用)

用法: python3 tests/diag/qpose.py [实体名 ...]
      默认 robot chassis kettle cup
"""
import sys
import time

import gz.transport13 as gz_transport
from gz.msgs10.pose_v_pb2 import Pose_V

d = {}


def cb(m):
    for p in m.pose:
        d[p.name] = (p.position.x, p.position.y, p.position.z)


def main():
    names = sys.argv[1:] or ['robot', 'chassis', 'kettle', 'cup']
    gz_transport.Node().subscribe(
        Pose_V, '/world/follow_world/dynamic_pose/info', cb)
    time.sleep(2.0)
    for n in names:
        v = d.get(n)
        print('%-12s %s' % (n, ('(%.4f, %.4f, %.4f)' % v) if v else '缺失'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
