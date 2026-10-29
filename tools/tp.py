#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tp.py —— 命令行摆位助手(调试/布景用)

用法: python3 tools/tp.py <model> <x> <y> [yaw] [z]
  z 缺省按模型取 test_hide 实证的落地高度: robot=0.15, person=0.02, 其余 0.0。
  gz service 从新进程发起, 服务发现不可靠(§2.1) → 失败自动重试 5 次。
"""
import math
import subprocess
import sys
import time

WORLD = 'follow_world'
Z0 = {'robot': 0.15, 'person': 0.02}


def main():
    name = sys.argv[1]
    x, y = float(sys.argv[2]), float(sys.argv[3])
    yaw = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
    z = float(sys.argv[5]) if len(sys.argv) > 5 else Z0.get(name, 0.0)
    req = ('name: "%s", position: {x: %s, y: %s, z: %s}, '
           'orientation: {z: %s, w: %s}'
           % (name, x, y, z, math.sin(yaw / 2), math.cos(yaw / 2)))
    for _ in range(5):
        r = subprocess.run(
            ['gz', 'service', '-s', '/world/%s/set_pose' % WORLD,
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '3000', '--req', req],
            capture_output=True, text=True)
        if 'true' in (r.stdout or '').lower():
            print('OK %s -> (%.2f, %.2f, z=%.3f, yaw=%.3f)' % (name, x, y, z, yaw))
            return 0
        time.sleep(0.4)
    print('FAIL set_pose %s' % name)
    return 1


if __name__ == '__main__':
    sys.exit(main())
