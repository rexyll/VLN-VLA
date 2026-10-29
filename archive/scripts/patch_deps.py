#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""patch_deps.py —— 把测试脚本里的硬编码绝对路径改为 $VLA_HOME 可配置

配合目录归置(docs/ tests/ tools/ archive/)执行: 脚本从顶层散落位置移入
tests/ 之后, 原来写死的 /home/earl/workspace/VLA/... 仍然是对的(根目录没
变), 但脚本之间的相互调用(tmp_run_eval.sh / tmp_follow.sh / tmp_random_eval.py)
路径已变, 必须跟着改。同时把产物统一输出到 tests/out/, 不再污染项目根。
"""
import os
import sys

ROOT = os.environ.get('VLA_HOME', '/home/earl/workspace/VLA')


def edit(path, pairs):
    """对 path 做精确替换; pairs = [(old, new), ...]; 自动保持 CRLF/LF"""
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
    print('  ok %-28s %d/%d' % (path, n, len(pairs)))
    return n


T = []
TOTAL = 0

# ---------------------------------------------------------- random_eval.py
TOTAL += edit('tests/random_eval.py', [
    # 1) import os + VLA_HOME
    ("""import argparse
import json
import math
import subprocess
import sys
import time
""",
     """import argparse
import json
import math
import os
import subprocess
import sys
import time

# 项目根: 所有依赖与产物都从这里解析, 不再硬编码
VLA_HOME = os.environ.get('VLA_HOME', '/home/earl/workspace/VLA')
"""),
    # 2) docstring 用法
    ('  python3 tmp_random_eval.py --dur 180 --out eval_random',
     '  python3 tests/random_eval.py --dur 180 --out eval_random'),
    # 3) --outdir 参数
    ("""    ap.add_argument('--out', default='eval_random')""",
     """    ap.add_argument('--out', default='eval_random')
    ap.add_argument('--outdir', default=os.path.join(VLA_HOME, 'tests', 'out'),
                    help='产物目录(json/png), 默认 $VLA_HOME/tests/out')"""),
    # 4) load_band 用绝对路径 + 建输出目录
    ("""    else:
        load_band()
""",
     """    else:
        load_band(os.path.join(VLA_HOME, 'vla_demo', 'vision_config.json'))
    os.makedirs(a.outdir, exist_ok=True)
"""),
    # 5) 下发 FOLLOW 的外部脚本路径
    ("""    # rclpy 的 publisher 在"刚创建就发"的情况下实测不可靠(订阅端没收到),
    # 这里额外用命令行发(独立进程, DDS 发现完整; 命令写在 tmp_follow.sh 里
    # 避免多层引号转义出错)
    for k in range(3):
        r = subprocess.run(['bash', '/home/earl/workspace/VLA/tmp_follow.sh'],""",
     """    # rclpy 的 publisher 在"刚创建就发"的情况下实测不可靠(订阅端没收到),
    # 这里额外用命令行发(独立进程, DDS 发现完整; 命令写在
    # tests/follow_cmd.sh 里, 避免多层引号转义出错)
    for k in range(3):
        r = subprocess.run(['bash', os.path.join(VLA_HOME, 'tests',
                                                 'follow_cmd.sh')],"""),
    # 6) json / png 产物路径
    ("""    out_json = '/home/earl/workspace/VLA/%s.json' % a.out""",
     """    out_json = os.path.join(a.outdir, '%s.json' % a.out)"""),
    ("""    png = draw(recs, '/home/earl/workspace/VLA/shots/%s_traj.png' % a.out)""",
     """    png = draw(recs, os.path.join(a.outdir, '%s_traj.png' % a.out))"""),
])

# ------------------------------------------------------- 三个受控场景测试
for fn, key, usage in (
        ('tests/test_orbit.py', 'orbit',
         'python3 tests/test_orbit.py <tag> [dur=90] [R=1.0] [spd=0.35]'),
        ('tests/test_hide.py', 'hide',
         'python3 tests/test_hide.py <tag> [dur=30]'),
        ('tests/test_stick.py', 'stick',
         'python3 tests/test_stick.py <tag> [dur=25]')):
    # 在 import 区块里插入 os + OUTDIR: 找第一个 'import json'
    TOTAL += edit(fn, [
        ("""import json
import math""",
         """import json
import math
import os

# 项目根: 产物统一输出到 tests/out/, 不再散落在项目根
VLA_HOME = os.environ.get('VLA_HOME', '/home/earl/workspace/VLA')
OUTDIR = os.path.join(VLA_HOME, 'tests', 'out')
os.makedirs(OUTDIR, exist_ok=True)"""),
        ("""self.f = open('%s_%%s.jsonl' %% tag, 'w', encoding='utf-8')""" % key,
         """self.f = open(os.path.join(OUTDIR, '%s_%%s.jsonl' %% tag),
                      'w', encoding='utf-8')""" % key),
    ])

# stick 还有一个回读
TOTAL += edit('tests/test_stick.py', [
    ("""    rows = [json.loads(l) for l in open('stick_%s.jsonl' % tag,""",
     """    rows = [json.loads(l) for l in open(os.path.join(OUTDIR,
                                          'stick_%s.jsonl' % tag),"""),
])

# ---------------------------------------------------------- docstring 用法行
for fn, old, new in (
        ('tests/test_orbit.py', '用法: python3 tmp_orbit_test.py',
         '用法: python3 tests/test_orbit.py'),
        ('tests/test_hide.py', '用法: python3 tmp_hide_test.py',
         '用法: python3 tests/test_hide.py'),
        ('tests/test_stick.py', 'tmp_stick_test.py',
         'tests/test_stick.py')):
    TOTAL += edit(fn, [(old, new)])

print('总替换 %d 处' % TOTAL)
