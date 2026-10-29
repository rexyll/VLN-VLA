#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P60: 跟随全程追踪 pitch/roll, 捕捉车体倾斜的第一现场。
在正常跟随(person_walk 随机走)过程中以 10Hz 记录:
  机器人 pitch/roll/pos/yaw、8 扇区距离、follower 指令、机器人速度。
一旦 |pitch|>5° 或 |roll|>5° 立即打印前后 2s 的上下文。
"""
import json
import math
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import gz.transport13 as gt
from gz.msgs10.pose_v_pb2 import Pose_V

DUR = 90.0
robot = {}
gn = gt.Node()


def pose_cb(m):
    for p in m.pose:
        if p.name == 'robot':
            q = p.orientation
            roll = math.atan2(2 * (q.w * q.x + q.y * q.z),
                              1 - 2 * (q.x * q.x + q.y * q.y))
            pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            robot['v'] = (p.position.x, p.position.y, p.position.z,
                          math.degrees(roll), math.degrees(pitch),
                          math.degrees(yaw))


gn.subscribe(Pose_V, '/world/follow_world/dynamic_pose/info', pose_cb)

rclpy.init()
node = Node('p60_trace')
prox = {}
stat = {}


def pc(m):
    try:
        prox.update(json.loads(m.data))
    except Exception:
        pass


def sc(m):
    try:
        stat.update(json.loads(m.data))
    except Exception:
        pass


node.create_subscription(String, '/perception/proximity', pc, 10)
node.create_subscription(String, '/follower/status', sc, 10)

# 下跟随指令(持续发 2s, 等话题发现完成 —— 只发 3 次会被丢掉)
pub = node.create_publisher(String, '/robot/command', 10)
cmd = String()
cmd.data = json.dumps({'action': 'FOLLOW', 'distance': 1.0})
t_cmd = time.time()
while time.time() - t_cmd < 2.0:
    pub.publish(cmd)
    rclpy.spin_once(node, timeout_sec=0.05)
    time.sleep(0.1)
print('已下发 FOLLOW')

hist = []
t0 = time.time()
tilted = False
while time.time() - t0 < DUR:
    rclpy.spin_once(node, timeout_sec=0.03)
    time.sleep(0.02)
    v = robot.get('v')
    if not v:
        continue
    rec = (time.time() - t0, v[0], v[1], v[2], v[3], v[4], v[5],
           prox.get('front', -1), prox.get('fl', -1), prox.get('fr', -1),
           stat.get('cmd_v', 0.0), stat.get('cmd_w', 0.0),
           stat.get('prim', ''), (stat.get('note') or '')[:30])
    hist.append(rec)
    if not tilted and (abs(v[4]) > 5 or abs(v[3]) > 5):
        tilted = True
        print('\n!!! 倾斜出现 t=%.1f  pitch=%+.1f roll=%+.1f' % (rec[0], v[4], v[3]))
        print('前 2.5s 上下文:')
        for h in hist[-25:]:
            print('  t=%5.1f pos=(%6.2f,%6.2f,%6.3f) roll=%+5.1f pitch=%+5.1f '
                  'yaw=%+6.1f | f=%4.2f fl=%4.2f fr=%4.2f | v=%+.2f w=%+.2f %-6s %s'
                  % (h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7], h[8], h[9],
                     h[10], h[11], h[12], h[13]))
        print()
    if tilted and (time.time() - t0) > 0 and len(hist) > 0:
        pass

print('=== 结束: %s ===' % ('出现过倾斜' if tilted else '全程无倾斜'))
if hist:
    print('t     pos                       roll  pitch   yaw    f    fl   fr   '
          'v     w    prim note')
    for h in hist[::max(1, len(hist) // 40)]:
        print('%5.1f (%6.2f,%6.2f,%6.3f) %+5.1f %+6.1f %+6.1f %5.2f %5.2f '
              '%5.2f %+.2f %+.2f %-5s %s'
              % (h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7], h[8], h[9],
                 h[10], h[11], h[12], h[13]))
