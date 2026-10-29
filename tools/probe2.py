#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""probe2.py —— 端到端验证 create/remove: 真值确认, 不看应答脸色。"""
import subprocess
import sys
import time

WORLD = 'follow_world'
NAME = 'probe_liquid2'
SDF = ('<model name="%s"><pose>0 0 -1.0 0 0 0</pose>'
       '<link name="body"><gravity>false</gravity>'
       '<inertial><mass>0.002</mass><inertia><ixx>1e-6</ixx>'
       '<iyy>1e-6</iyy><izz>1e-6</izz><ixy>0</ixy><ixz>0</ixz>'
       '<iyz>0</iyz></inertia></inertial>'
       '<visual name="v"><geometry><cylinder><radius>0.02</radius>'
       '<length>0.01</length></cylinder></geometry>'
       '<material><ambient>0.2 0.55 0.85 1</ambient>'
       '<diffuse>0.2 0.55 0.85 1</diffuse></material>'
       '<transparency>0.4</transparency></visual>'
       '</link></model>' % NAME)


def gz(svc, reqtype, reptype, req, blocking=True, tries=3):
    for _ in range(tries):
        r = subprocess.run(
            ['gz', 'service', '-s', svc, '--reqtype', reqtype,
             '--reptype', reptype, '--timeout', '5000', '--req', req],
            capture_output=True, text=True)
        out = (r.stdout or '') + (r.stderr or '')
        if 'true' in out.lower():
            return True, out.strip()[:120]
        time.sleep(0.4)
    return False, out.strip()[:120]


def truth(names, window=3.0):
    import gz.transport13 as gz_transport
    import gz.msgs10.pose_v_pb2 as pose_v
    got = {}
    node = gz_transport.Node()
    def cb(msg):
        for p in msg.pose:
            if p.name in names:
                got[p.name] = (p.position.x, p.position.y, p.position.z)
    node.subscribe(pose_v.Pose_V,
                   '/world/%s/dynamic_pose/info' % WORLD, cb)
    t0 = time.time()
    while time.time() - t0 < window:
        time.sleep(0.1)
    return got


def main():
    path = '/tmp/vla_probe2.sdf'
    with open(path, 'w') as f:
        f.write(SDF)
    req = 'sdf: "%s", name: "%s"' % (path, NAME)
    ok_b, out_b = gz('/world/%s/create/blocking' % WORLD,
                     'gz.msgs.EntityFactory', 'gz.msgs.Boolean', req)
    print('create/blocking: %s %r' % (ok_b, out_b))
    ok_n, out_n = gz('/world/%s/create' % WORLD,
                     'gz.msgs.EntityFactory', 'gz.msgs.Boolean', req)
    print('create:          %s %r' % (ok_n, out_n))
    time.sleep(1.0)
    t = truth([NAME], 3.0)
    print('after-create truth: %s' % ('存在 %s' % (t[NAME],) if NAME in t
                                       else '未发现'))
    ok_r, out_r = gz('/world/%s/remove' % WORLD, 'gz.msgs.Entity',
                     'gz.msgs.Boolean', 'name: "%s", type: MODEL' % NAME)
    print('remove:          %s %r' % (ok_r, out_r))
    time.sleep(1.0)
    t2 = truth([NAME], 3.0)
    print('after-remove truth: %s' % ('仍在!' if NAME in t2 else '已消失'))
    sys.exit(0)


if __name__ == '__main__':
    main()
