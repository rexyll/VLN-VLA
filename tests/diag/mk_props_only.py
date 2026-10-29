#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""mk_props_only.py —— 从 work_area.sdf 里剥出"桌子+壶+杯"做一个最小世界

为什么: P84 里壶/杯一启动就被弹到几米外。工作区大世界里有机器人、人、墙、
       障碍物共十几万行, 直接在里面排查等于大海捞针。把三个模型原样抽出来
       放进一个只有地板+桌子的空世界:
         - 若照样飞  → 问题在物件自身(碰撞/关节/惯量), 与大世界无关
         - 若稳稳坐住 → 问题是大世界里别的东西撞的(机器人/人/初始重叠)
       这是唯一能一刀切开的实验。

用法:
    python3 tests/diag/mk_props_only.py            # 写 /tmp/props_only.sdf
    python3 tests/diag/mk_props_only.py --with robot,person
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
WORLD = os.path.join(ROOT, 'vla_ws', 'src', 'vla_follow', 'worlds',
                     'work_area.sdf')
OUT = '/tmp/props_only.sdf'


def grab(src, name):
    """取出 <model name="name"> ... 对应 </model> 的整段(按缩进配对)"""
    m = re.search(r'( *)<model name="%s">' % re.escape(name), src)
    if not m:
        raise SystemExit('找不到模型 %s' % name)
    indent = m.group(1)
    start = m.start()
    # 从模型开头往后找, 深度计数
    depth = 0
    i = start
    tag = re.compile(r'<model\b|</model>')
    for t in tag.finditer(src, start):
        if t.group(0) == '</model>':
            depth -= 1
            if depth == 0:
                end = t.end()
                break
        else:
            depth += 1
    return src[start:end]


def main():
    src = open(WORLD, encoding='utf-8').read()
    want = ['table', 'kettle', 'cup']
    if '--with' in sys.argv:
        want += sys.argv[sys.argv.index('--with') + 1].split(',')
    parts = []
    for n in want:
        try:
            parts.append(grab(src, n))
        except SystemExit as e:
            print('跳过: %s' % e)
    head = """<?xml version="1.0"?>
<sdf version="1.9">
  <world name="follow_world">
    <physics name="fast" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <model name="floor">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <pose>0 0 -0.05 0 0 0</pose>
          <geometry><box><size>12.3 12.3 0.1</size></box></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
      </link>
    </model>
"""
    body = '\n'.join(parts)
    tail = "\n  </world>\n</sdf>\n"
    with open(OUT, 'w', encoding='utf-8') as fh:
        fh.write(head + body + tail)
    print('写出 %s  (%d 行, 含 %s)' % (OUT, len((head + body + tail).splitlines()),
                                     ', '.join(want)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
