#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""诊断 /scan 的坐标系、量程与回波质量。

判据: 把每条射线按 (机器人真值位姿 + 雷达安装偏置) 转到世界坐标, 看落点
是否落在已知障碍的外接圆上。若命中率极低 → 角度零/方向不对;
若大量射线贴着量程下限 0.12 → 雷达被几何体包住, 或量程下限设得太高。
"""
import math
import time

import gz.transport13 as gz
from gz.msgs10.pose_v_pb2 import Pose_V
import rclpy
from rclpy.node import Node as RNode
from sensor_msgs.msg import LaserScan

OBST = [(3.5, 2.0, 0.57, 'ob1'), (-3.0, 3.2, 0.67, 'ob2'),
        (0.5, -3.2, 0.40, 'ob3'), (-4.2, -2.5, 0.50, 'ob4'),
        (4.2, -3.0, 0.42, 'ob5'), (-1.5, -4.5, 0.56, 'ob6'),
        (2.2, 4.6, 0.35, 'ob7')]
LIDAR_DX = 0.0        # SDF 已把雷达居中(x=0)

ST = {'pose': None, 'scan': None}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    gzn = gz.Node()

    def pose_cb(msg):
        for p in msg.pose:
            if p.name == 'robot':
                ST['pose'] = (p.position.x, p.position.y,
                              yaw_of(p.orientation))

    gzn.subscribe(Pose_V, '/world/follow_world/dynamic_pose/info', pose_cb)

    rclpy.init()
    ros = RNode('scan_diag')

    def scan_cb(msg):
        ST['scan'] = msg

    ros.create_subscription(LaserScan, '/scan', scan_cb, 10)
    t0 = time.time()
    while time.time() - t0 < 8.0:
        rclpy.spin_once(ros, timeout_sec=0.05)
        if ST['pose'] and ST['scan']:
            break
    if not ST['scan']:
        print('没收到 /scan')
        return 1

    sc = ST['scan']
    rx, ry, ryaw = ST['pose']
    print('机器人真值: (%+.3f, %+.3f)  yaw=%+.1f°'
          % (rx, ry, math.degrees(ryaw)))
    print('angle_min=%.4f (%.1f°)  angle_max=%.4f (%.1f°)  '
          'increment=%.6f (%.3f°)  点数=%d'
          % (sc.angle_min, math.degrees(sc.angle_min),
             sc.angle_max, math.degrees(sc.angle_max),
             sc.angle_increment, math.degrees(sc.angle_increment),
             len(sc.ranges)))
    print('range_min=%.3f  range_max=%.3f' % (sc.range_min, sc.range_max))

    rng = list(sc.ranges)
    fin = [r for r in rng if math.isfinite(r)]
    if fin:
        srt = sorted(fin)
        print('有效射线 %d/%d  min=%.3f  中位=%.3f  max=%.3f'
              % (len(fin), len(rng), srt[0], srt[len(srt) // 2], srt[-1]))
    low = sum(1 for r in fin if r <= 0.13)
    inf_c = len(rng) - len(fin)
    print('贴量程下限(<=0.13m): %d (%.1f%%)   无回波(inf): %d (%.1f%%)'
          % (low, 100.0 * low / max(1, len(rng)),
             inf_c, 100.0 * inf_c / max(1, len(rng))))

    cx = rx + LIDAR_DX * math.cos(ryaw)
    cy = ry + LIDAR_DX * math.sin(ryaw)
    print('\n雷达安装点(世界): (%+.3f, %+.3f)' % (cx, cy))

    def rate(sign, off):
        h = 0
        n = 0
        for i, r in enumerate(rng):
            if not math.isfinite(r) or r <= 0.13:
                continue
            ang = sc.angle_min + i * sc.angle_increment
            n += 1
            px = cx + r * math.cos(ryaw + sign * ang + off)
            py = cy + r * math.sin(ryaw + sign * ang + off)
            for ox, oy, orad, _ in OBST:
                if math.hypot(px - ox, py - oy) <= orad + 0.12:
                    h += 1
                    break
        return h, n

    print('坐标系反证 —— 哪种角度约定能让落点对上真实障碍:')
    for tag, sign, off in (('正常  yaw+ang', 1, 0.0),
                           ('取反  yaw-ang', -1, 0.0),
                           ('偏移 +90°', 1, math.pi / 2),
                           ('偏移 -90°', 1, -math.pi / 2),
                           ('偏移 180°', 1, math.pi)):
        h, n = rate(sign, off)
        print('  %-16s 命中 %3d/%3d  (%.1f%%)'
              % (tag, h, n, 100.0 * h / max(1, n)))

    # 逐方向打印最近回波(相对车头), 便于人工核对
    print('\n各方向最近回波(相对车头, 度 -> 米):')
    for lo, hi, tag in ((-25, 25, '前'), (25, 85, '左前'),
                        (85, 135, '左'), (-85, -25, '右前'),
                        (-135, -85, '右'), (135, 180, '左后'),
                        (-180, -135, '右后')):
        best = (1e9, 0)
        for i, r in enumerate(rng):
            if not math.isfinite(r) or r <= 0.0:
                continue
            ang = math.degrees(sc.angle_min + i * sc.angle_increment)
            if lo <= ang <= hi and r < best[0]:
                best = (r, ang)
        print('  %-4s [%+4d,%+4d]°  最近 %.3f m @ %+.0f°'
              % (tag, lo, hi, best[0] if best[0] < 1e8 else -1,
                 best[1]))
    ros.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
