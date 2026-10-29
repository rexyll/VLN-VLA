#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""给 ~/.gz/sim/8/gui.config 的 MarkerManager 插件补上 <topic_name>，
否则该插件不会 advertise /marker/list、/marker/add、/marker/add_multiple，
外部就无法把 marker 送到 GUI 渲染队列（日志里表现为 No subscribers on [/marker]）。"""
import shutil

P = '/home/earl/.gz/sim/8/gui.config'
OLD = '<plugin filename="MarkerManager" name="Marker manager">'
NEW = OLD + '\n    <topic_name>/marker</topic_name>'

s = open(P, encoding='utf-8').read()
if '<topic_name>' in s:
    print('ALREADY_PATCHED')
else:
    if s.count(OLD) != 1:
        raise SystemExit('找不到唯一的 MarkerManager 节点, count=%d' % s.count(OLD))
    shutil.copy(P, P + '.bak_marker')
    open(P, 'w', encoding='utf-8').write(s.replace(OLD, NEW))
    print('PATCHED')

for i, line in enumerate(open(P, encoding='utf-8'), 1):
    if 'MarkerManager' in line or 'topic_name' in line:
        print(i, line.rstrip())
