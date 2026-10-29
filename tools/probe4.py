#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""probe4.py —— ① EntityFactory 真实字段名 ② UserCommands 队列活性。"""
import math
import subprocess
import sys
import time

import gz.msgs10.entity_factory_pb2 as ef
import gz.transport13 as gz_transport
import gz.msgs10.pose_v_pb2 as pose_v

WORLD = 'follow_world'

print('EntityFactory fields:',
      list(ef.EntityFactory.DESCRIPTOR.fields_by_name.keys()))


def gz(svc, reqtype, reptype, req):
    r = subprocess.run(
        ['gz', 'service', '-s', svc, '--reqtype', reqtype,
         '--reptype', reptype, '--timeout', '5000', '--req', req],
        capture_output=True, text=True)
    return ((r.stdout or '') + (r.stderr or '')).strip()[:200]


def truth(names, window=2.5):
    got = {}
    node = gz_transport.Node()
    def cb(msg):
        for p in msg.pose:
            if p.name in names:
                got[p.name] = (round(p.position.x, 3),
                               round(p.position.y, 3))
    node.subscribe(pose_v.Pose_V,
                   '/world/%s/dynamic_pose/info' % WORLD, cb)
    t0 = time.time()
    while time.time() - t0 < window:
        time.sleep(0.1)
    return got


def main():
    t0 = truth(['robot'])
    if 'robot' not in t0:
        print('truth 不可达')
        sys.exit(1)
    x0, y0 = t0['robot']
    req = ('name: "robot", position: {x: %s, y: %s, z: 0.0}, '
           'orientation: {z: 0.0, w: 1.0}' % (x0 + 0.05, y0))
    out = gz('/world/%s/set_pose' % WORLD, 'gz.msgs.Pose',
             'gz.msgs.Boolean', req)
    print('set_pose: %r' % out)
    time.sleep(1.0)
    t1 = truth(['robot'])
    moved = t1.get('robot') and abs(t1['robot'][0] - x0) > 0.03
    print('robot: %s -> %s  %s' % (t0['robot'], t1.get('robot'),
                                   '队列活着' if moved else '★队列死了'))
    sys.exit(0)


if __name__ == '__main__':
    main()
