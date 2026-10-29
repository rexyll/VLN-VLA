#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sync_deploy.py —— 从 VM 拉取最新源码到本地 deploy/

用 base64 + 分隔符一次性取多个文件, 避免多次 SSH 往返。
"""
import base64
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vm  # noqa: E402

FILES = [
    ('vla_ws/src/vla_follow/vla_follow/follower.py', 'follower.py'),
    ('vla_ws/src/vla_follow/vla_follow/depth_percept.py', 'depth_percept.py'),
    ('vla_ws/src/vla_follow/vla_follow/person_vision.py', 'person_vision.py'),
    ('vla_ws/src/vla_follow/vla_follow/person_walk.py', 'person_walk.py'),
    ('vla_ws/src/vla_follow/worlds/work_area.sdf', 'work_area.sdf'),
    ('vla_demo/vision_config.json', 'vision_config.json'),
    ('vla_demo/traj_viz.py', 'traj_viz.py'),
]

ROOT = os.environ.get('VLA_HOME', '/home/earl/workspace/VLA')
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'deploy')

c = vm.client()
try:
    for remote, local in FILES:
        cmd = f"cd {ROOT} && base64 -w0 {remote}"
        _in, out, _err = c.exec_command(cmd, timeout=120)
        raw = out.read().decode('utf-8', 'replace')
        rc = out.channel.recv_exit_status()
        if rc != 0:
            print(f'!! 读取失败 {remote}')
            continue
        data = base64.b64decode(raw.strip())
        dst = os.path.join(OUT, local)
        with open(dst, 'wb') as f:
            f.write(data)
        print(f'  {local:22s} {len(data):7d}B')
finally:
    c.close()
print('SYNC_DONE')
