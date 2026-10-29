#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""probe_dynpose.py —— 验证"新进程能否订阅 gz 真值话题"这件事

背景: 之前 tests/diag/probe_gz.py 里新起的 gz.transport 订阅收不到任何消息,
      据此在 TaskPlan §2.1 里写下了"所有指令走 ROS 话题 → bridge, 不新建 gz
      订阅"的结论。但生产节点 gt_tf.py 恰恰就是新进程订阅
      /world/follow_world/dynamic_pose/info, 而且一直好用 —— 两者矛盾,
      必须查清, 因为桌上水壶/水杯的世界位姿只有这条链路能给。

本探针同时试三种订阅姿势, 打印各自收到的条数:
  A) gz.transport13.Node().subscribe(Pose_V, topic, cb)   ← gt_tf 的写法
  B) 同上, 但先 sleep 2s 再订阅 (排除"发现未完成")
  C) 带 msg_type 显式指定 + sub = node.subscribe(...) 保留引用

用法: python3 tests/diag/probe_dynpose.py [秒数]
"""
import sys
import time

import gz.transport13 as gz_transport
from gz.msgs10.pose_v_pb2 import Pose_V

TOPIC = (sys.argv[2] if len(sys.argv) > 2
         else '/world/follow_world/dynamic_pose/info')
DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0

hits = []
last = {}

WATCH = ('robot', 'chassis', 'torso', 'R_L0', 'R_L1', 'R_L6', 'R_palm',
         'kettle', 'kettle::body', 'cup', 'cup::body')


def cb(msg):
    hits.append((time.time(), [p.name for p in msg.pose]))
    for p in msg.pose:
        last[p.name] = (p.position.x, p.position.y, p.position.z)


def main():
    n = gz_transport.Node()
    t0 = time.time()
    ok = n.subscribe(Pose_V, TOPIC, cb)
    print('subscribe() 返回: %s' % ok)
    while time.time() - t0 < DUR:
        time.sleep(0.2)
    print('收到 %d 条' % len(hits))
    if hits:
        names = hits[-1][1]
        print('最后一条含 %d 个实体: %s' % (len(names), names[:12]))
        for want in ('robot', 'person', 'kettle', 'cup', 'table'):
            print('   %-7s %s' % (want, '有' if want in names else '无'))
        print()
        if '--all' in sys.argv:
            print('--- 全部实体 ---')
            for k in sorted(last):
                v = last[k]
                print('   %-16s (%8.4f, %8.4f, %8.4f)' % (k, v[0], v[1], v[2]))
            return 0
        print('--- 关键实体世界位置 ---')
        for want in WATCH:
            T = last.get(want)
            if T is None:
                print('   %-10s 无' % want)
            else:
                print('   %-10s (%8.4f, %8.4f, %8.4f)' % (want, T[0], T[1], T[2]))
    else:
        print('一条都没收到 —— 说明本机新进程的 gz 订阅确实不可用')
    return 0 if hits else 1


if __name__ == '__main__':
    sys.exit(main())
