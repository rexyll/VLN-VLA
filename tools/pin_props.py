#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pin_props.py —— 把壶/杯用插件 set 目标**软钉**在名义位姿。

自由体速度不衰减(§7 #58), 摆位后还会漂; set 目标由插件每 tick 重申,
在下一个 release/grab 之前壶/杯停在名义位不动。喝水周期开始后会被
周期自己的 grab/put_down 正常接管, 不留副作用。
"""
import math
import sys
import time

import rclpy
from std_msgs.msg import String

NOMINAL = {
    'kettle': (2.300, 0.200, 0.803, 1.571),
    'cup': (2.300, 0.440, 0.773, 0.0),
}
rclpy.init()
n = rclpy.create_node('pin_props')
pub = n.create_publisher(String, '/drink/carry', 10)
# P98 教训(#24): 新发布者最初 1~2s 的消息会整批丢在 DDS 发现期,
# 4 轮 / 1.2s 全军覆没 —— 突发拉长到 ~5s 确保发现完成后仍有多发落地。
for rnd in range(14):
    for name, (x, y, z, yaw) in NOMINAL.items():
        qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
        pub.publish(String(data=(
            'set %s %.6f %.6f %.6f 0 0 %.6f %.6f'
            % (name, x, y, z, qz, qw))))
        rclpy.spin_once(n, timeout_sec=0.05)
    time.sleep(0.35)
print('pinned kettle+cup at nominal')
n.destroy_node()
rclpy.shutdown()
