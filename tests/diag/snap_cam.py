#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""snap_cam.py —— 抓一张当前 /drink_cam 画面落盘(只读, 不发任何指令)。

用法: python3 tests/diag/snap_cam.py [输出文件名, 默认 gui_snap.png]
"""
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, 'vla_ws', 'src', 'vla_follow'))

import rclpy                                   # noqa: E402
from sensor_msgs.msg import Image              # noqa: E402

from vla_follow.drink_task import DrinkTask    # noqa: E402
from vla_follow.pngsnap import save_png        # noqa: E402


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else 'gui_snap.png'
    rclpy.init()
    node = DrinkTask(observer_only=True)
    box = {}
    node.create_subscription(Image, '/drink_cam/image',
                             lambda m: box.__setitem__('i', m), 5)
    t0 = time.time()
    while 'i' not in box and time.time() - t0 < 15.0:
        node.spin_for(0.2)
    if 'i' not in box:
        print('没拿到 /drink_cam/image')
        return 2
    m = box['i']
    path = os.path.join(ROOT, 'vla_ws', 'logs', name)
    save_png(path, m.data, m.width, m.height, m.encoding)
    print('已保存: %s (%dx%d, %s)' % (path, m.width, m.height, m.encoding))
    return 0


if __name__ == '__main__':
    sys.exit(main())
