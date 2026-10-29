#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""patch_tools.py —— 把 tools/ 与 vla_demo/ 里的硬编码根路径改为可配置"""
import os

ROOT = os.environ.get('VLA_HOME', '/home/earl/workspace/VLA')


def edit(path, pairs):
    p = os.path.join(ROOT, path)
    if not os.path.isfile(p):
        print('  !! 缺失 %s' % path)
        return 0
    raw = open(p, 'rb').read().decode('utf-8')
    crlf = '\r\n' in raw
    n = 0
    for old, new in pairs:
        o = old.replace('\n', '\r\n') if crlf else old
        w = new.replace('\n', '\r\n') if crlf else new
        if o in raw:
            raw = raw.replace(o, w, 1)
            n += 1
        else:
            print('  !! 未命中 [%s] %s' % (path, old.split('\n')[0][:60]))
    open(p, 'wb').write(raw.encode('utf-8'))
    print('  ok %-26s %d/%d' % (path, n, len(pairs)))
    return n


T = 0
T += edit('tools/sync_deploy.py', [
    ("ROOT = '/home/earl/workspace/VLA'",
     "ROOT = os.environ.get('VLA_HOME', '/home/earl/workspace/VLA')"),
])
T += edit('tools/vmsync.py', [
    ('LOCAL_ROOT = r"F:\\workspace\\_coding_workspace\\VLA"',
     'LOCAL_ROOT = os.environ.get("VLA_LOCAL_ROOT", r"Z:\\VLA")'),
    ('REMOTE_ROOT = "/home/earl/workspace/VLA"',
     'REMOTE_ROOT = os.environ.get("VLA_REMOTE_ROOT", "/home/earl/workspace/VLA")'),
])
T += edit('vla_demo/traj_viz.py', [
    ("ap.add_argument('--out', default='/home/earl/workspace/VLA/traj_log.csv')",
     "ap.add_argument('--out',\n"
     "                    default=os.path.join(\n"
     "                        os.environ.get('VLA_HOME',\n"
     "                                       '/home/earl/workspace/VLA'),\n"
     "                        'tests', 'out', 'traj_log.csv'))"),
])
print('总替换 %d 处' % T)
