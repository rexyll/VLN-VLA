#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""liquid_probe.py —— 一次性探针: 验证 /world/*/create 运行时注入链路。

注入一个探针小柱(停在地板下) → 确认应答 → remove 收尾。成功 = P98 液面
模型可免重启注入; 同时本文件的 spawn/remove 写法就是 liquid.py 的原型。
"""
import subprocess
import sys
import time

WORLD = 'follow_world'
SDF = """<model name="probe_liquid">
  <pose>0 0 -1.0 0 0 0</pose>
  <link name="body">
    <gravity>false</gravity>
    <inertial><mass>0.002</mass>
      <inertia><ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz>
        <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
    <visual name="v">
      <geometry><cylinder><radius>0.02</radius><length>0.01</length></cylinder></geometry>
      <material><ambient>0.2 0.55 0.85 1</ambient><diffuse>0.2 0.55 0.85 1</diffuse></material>
      <transparency>0.4</transparency>
    </visual>
  </link>
</model>
"""


def gz_req(svc, reqtype, reptype, req, timeout=3000):
    for i in range(3):
        r = subprocess.run(
            ['gz', 'service', '-s', svc, '--reqtype', reqtype,
             '--reptype', reptype, '--timeout', str(timeout), '--req', req],
            capture_output=True, text=True)
        out = (r.stdout or '')
        if 'true' in out.lower():
            return True
        time.sleep(0.4)
    return False


def main():
    path = '/tmp/vla_probe_liquid.sdf'
    with open(path, 'w') as f:
        f.write(SDF)
    ok = gz_req('/world/%s/create' % WORLD,
                'gz.msgs.EntityFactory', 'gz.msgs.Boolean',
                'sdf: "%s", name: "probe_liquid"' % path)
    print('CREATE probe_liquid:', 'OK' if ok else 'FAIL')
    if not ok:
        sys.exit(1)
    time.sleep(1.0)
    gone = gz_req('/world/%s/remove' % WORLD,
                  'gz.msgs.Entity', 'gz.msgs.Boolean',
                  'name: "probe_liquid"')
    print('REMOVE probe_liquid:', 'OK' if gone else 'FAIL (手动 gz service 删)')
    sys.exit(0 if gone else 2)


if __name__ == '__main__':
    main()
