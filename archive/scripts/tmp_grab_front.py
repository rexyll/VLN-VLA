#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""抓一帧 front 相机 RGB 图 —— 用于肉眼确认 person 模型是否完整(头/身体
是否掉在地上)。顺便把机器人/人摆成"正对 1.2m"的固定位形。

用法: python3 tmp_grab_front.py [out.png] [dist]
"""
import math
import subprocess
import sys
import time

import cv2
import numpy as np

import gz.transport13 as gt
from gz.msgs10.image_pb2 import Image

WORLD = 'follow_world'
OUT = sys.argv[1] if len(sys.argv) > 1 else '/tmp/front.png'
D = float(sys.argv[2]) if len(sys.argv) > 2 else 1.2

# gz.msgs.Image.pixel_format_type: 1=L_INT8 3=RGB_INT8 4=RGBA_INT8 5=BGRA_INT8
FMT = {1: 'gray', 3: 'rgb', 4: 'rgba', 5: 'bgra'}


def set_pose(name, x, y, z, yaw=0.0):
    req = ('name: "%s", position: {x: %s, y: %s, z: %s}, '
           'orientation: {z: %s, w: %s}'
           % (name, x, y, z, math.sin(yaw / 2), math.cos(yaw / 2)))
    subprocess.run(['gz', 'service', '-s', '/world/%s/set_pose' % WORLD,
                    '--reqtype', 'gz.msgs.Pose',
                    '--reptype', 'gz.msgs.Boolean', '--timeout', '2000',
                    '--req', req], capture_output=True, text=True)


got = []
node = gt.Node()


def cb(msg):
    if got:
        return
    got.append(msg)


node.subscribe(Image, '/camera', cb)

# 摆位形: 机器人在原点朝 +x, 人在正前方 D 米, 面朝机器人
set_pose('robot', 0.0, 0.0, 0.15, 0.0)
set_pose('person', D, 0.0, 0.02, math.pi)
time.sleep(2.0)
got.clear()
t0 = time.time()
while time.time() - t0 < 5.0 and not got:
    time.sleep(0.05)

if not got:
    print('✗ 5s 内没收到 /camera 图像')
    sys.exit(1)

m = got[-1]
h, w = m.height, m.width
kind = FMT.get(m.pixel_format_type, '?')
buf = np.frombuffer(m.data, dtype=np.uint8)
if kind == 'rgb':
    img = buf.reshape(h, w, 3)[:, :, ::-1]
elif kind == 'bgra':
    img = buf.reshape(h, w, 4)[:, :, :3]
elif kind == 'rgba':
    img = buf.reshape(h, w, 4)[:, :, 2::-1]
elif kind == 'gray':
    img = cv2.cvtColor(buf.reshape(h, w), cv2.COLOR_GRAY2BGR)
else:
    print('✗ 未知像素格式 %s' % m.pixel_format_type)
    sys.exit(1)

cv2.imwrite(OUT, img)
print('✓ %s  %dx%d  fmt=%s  人距 %.2fm'
      % (OUT, w, h, kind, D))
