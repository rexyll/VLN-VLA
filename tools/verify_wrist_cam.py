#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""verify_wrist_cam.py —— 腕部 RGB-D 出流验证(gz 原生订阅, 3s 帧计数)。"""
import time

import gz.transport13 as gz_transport
import gz.msgs10.image_pb2 as image_pb2

TOPICS = ('/wrist_cam_R/image', '/wrist_cam_R/depth_image',
          '/wrist_cam_L/image', '/wrist_cam_L/depth_image')
stats = {t: {'n': 0, 'w': 0, 'h': 0} for t in TOPICS}
node = gz_transport.Node()


def mk(topic):
    def cb(m):
        s = stats[topic]
        s['n'] += 1
        s['w'], s['h'] = m.width, m.height
    return cb


for t in TOPICS:
    node.subscribe(image_pb2.Image, t, mk(t))
t0 = time.time()
while time.time() - t0 < 4.0:
    time.sleep(0.1)
for t in TOPICS:
    s = stats[t]
    ok = s['n'] > 0
    print('%-28s %2d 帧  %dx%d  %s' % (t, s['n'], s['w'], s['h'],
                                       'OK' if ok else '★无流'))
sys_ok = all(stats[t]['n'] > 0 for t in TOPICS)
print('==== %s ====' % ('ALL OK' if sys_ok else 'FAILED'))
