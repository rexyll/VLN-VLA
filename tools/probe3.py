#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""probe3.py —— EntityFactory 变体矩阵: sdf文件低/高空位 + data 内联。"""
import subprocess
import sys
import time

WORLD = 'follow_world'


def gz(svc, reqtype, reptype, req):
    r = subprocess.run(
        ['gz', 'service', '-s', svc, '--reqtype', reqtype,
         '--reptype', reptype, '--timeout', '5000', '--req', req],
        capture_output=True, text=True)
    return ((r.stdout or '') + (r.stderr or '')).strip()[:150]


def truth(names, window=3.0):
    import gz.transport13 as gz_transport
    import gz.msgs10.pose_v_pb2 as pose_v
    got = {}
    node = gz_transport.Node()
    def cb(msg):
        for p in msg.pose:
            if p.name in names:
                got[p.name] = (round(p.position.x, 2), round(p.position.y, 2),
                               round(p.position.z, 2))
    node.subscribe(pose_v.Pose_V,
                   '/world/%s/dynamic_pose/info' % WORLD, cb)
    t0 = time.time()
    while time.time() - t0 < window:
        time.sleep(0.1)
    return got


def mk(name, z):
    return ('<model name="%s"><pose>2.0 1.0 %s 0 0 0</pose>'
            '<link name="body"><gravity>false</gravity>'
            '<inertial><mass>0.002</mass><inertia><ixx>1e-6</ixx>'
            '<iyy>1e-6</iyy><izz>1e-6</izz><ixy>0</ixy><ixz>0</ixz>'
            '<iyz>0</iyz></inertia></inertial>'
            '<visual name="v"><geometry><cylinder><radius>0.02</radius>'
            '<length>0.01</length></cylinder></geometry>'
            '<material><ambient>0.2 0.55 0.85 1</ambient>'
            '<diffuse>0.2 0.55 0.85 1</diffuse></material>'
            '<transparency>0.4</transparency></visual>'
            '</link></model>' % (name, z))


def main():
    variants = []
    with open('/tmp/vla_p3a.sdf', 'w') as f:
        f.write(mk('probe3a', '-1.0'))
    variants.append(('sdf-file z=-1', 'sdf: "/tmp/vla_p3a.sdf", '
                     'name: "probe3a"'))
    with open('/tmp/vla_p3b.sdf', 'w') as f:
        f.write(mk('probe3b', '0.5'))
    variants.append(('sdf-file z=0.5', 'sdf: "/tmp/vla_p3b.sdf", '
                     'name: "probe3b"'))
    inline = mk('probe3c', '0.5').replace('"', '\\"')
    variants.append(('data-inline z=0.5', 'data: "%s", name: "probe3c"'
                     % inline))
    names = [v[0].split()[1] for v in variants]
    for tag, req in variants:
        out = gz('/world/%s/create/blocking' % WORLD,
                 'gz.msgs.EntityFactory', 'gz.msgs.Boolean', req)
        print('%-18s -> %r' % (tag, out), flush=True)
    time.sleep(1.0)
    t = truth(names, 4.0)
    for n in names:
        print('%-9s %s' % (n, ('出现 %s' % (t[n],)) if n in t else '未出现'),
              flush=True)
    sys.exit(0)


if __name__ == '__main__':
    main()
