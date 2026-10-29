#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""probe5.py —— 单发 sdf_filename create 验证(带原始应答与服务端日志对照)。"""
import importlib.util
import os
import subprocess
import sys
import time

spec = importlib.util.spec_from_file_location(
    'gen_drink_scene',
    '/home/earl/workspace/VLA/tools/gen_drink_scene.py')
g = importlib.util.module_from_spec(spec)
spec.loader.exec_module(g)

WORLD = 'follow_world'
name = sys.argv[1] if len(sys.argv) > 1 else 'kettle'
sdf = g._kettle_sdf() if name == 'kettle' else g._cup_sdf()
path = '/tmp/vla_probe5_%s.sdf' % name
with open(path, 'w') as f:
    f.write(sdf)
req = 'sdf_filename: "%s", name: "%s"' % (path, name)
for i in range(3):
    r = subprocess.run(
        ['gz', 'service', '-s', '/world/%s/create/blocking' % WORLD,
         '--reqtype', 'gz.msgs.EntityFactory', '--reptype',
         'gz.msgs.Boolean', '--timeout', '8000', '--req', req],
        capture_output=True, text=True)
    print('try%d reply=%r stderr=%r' % (i, (r.stdout or '').strip()[:80],
                                       (r.stderr or '').strip()[:120]),
          flush=True)
    time.sleep(1.0)
