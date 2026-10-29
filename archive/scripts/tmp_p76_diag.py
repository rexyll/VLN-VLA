#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_p76_diag.py —— 录 /follower/diag (10Hz JSON) 到文件"""
import argparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='diag.jsonl')
    ap.add_argument('--dur', type=float, default=180.0)
    a = ap.parse_args()
    rclpy.init()
    n = Node('p76_diag_rec')
    f = open(a.out, 'w', encoding='utf-8')
    st = {'n': 0}

    def cb(msg):
        f.write(msg.data + '\n')
        st['n'] += 1

    n.create_subscription(String, '/follower/diag', cb, 50)
    print('[DIAG] 开始记录', flush=True)
    try:
        while rclpy.ok() and st['n'] < a.dur * 10:
            rclpy.spin_once(n, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    f.flush()
    f.close()
    print('[DIAG] wrote %s  n=%d' % (a.out, st['n']), flush=True)
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
