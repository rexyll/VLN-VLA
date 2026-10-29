#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""抓一帧 gz 图像话题存 PNG: python3 gz_snap.py <topic> <out.png>"""
import sys
import time

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

topic = sys.argv[1] if len(sys.argv) > 1 else '/overhead/image'
out = sys.argv[2] if len(sys.argv) > 2 else '/tmp/gz_snap.png'

n = Node()
state = {'done': False}


def cb(msg):
    if state.get('busy') or state['done']:
        return
    state['busy'] = True   # 去重; done 只在整个写盘完成后才置位,
                           # 否则主线程会在 save 还没结束时 os._exit
    w, h = msg.width, msg.height
    data = bytes(msg.data)
    print('SNAP %dx%d len=%d fmt=%d' % (w, h, len(data), msg.pixel_format_type))
    try:
        from PIL import Image as PILImage
        img = PILImage.frombytes('RGB', (w, h), data[:w * h * 3])
        img.save(out)
        print('SAVED', out)
    except Exception as ex:
        print('PIL FAIL', repr(ex))
        with open(out + '.raw', 'wb') as f:
            f.write(data)
        print('raw ->', out + '.raw')
    sys.stdout.flush()
    state['done'] = True


n.subscribe(Image, topic, cb)
t0 = time.time()
while time.time() - t0 < 20 and not state['done']:
    time.sleep(0.1)
time.sleep(0.2)
# 不调 unsubscribe: gz 的 pybind11 绑定在解释器退出时偶发
# "PyGILState_Check() failure" 直接 abort, 会把已写的文件截成 0 字节。
# os._exit 跳过缓冲区 flush, 所以前面必须显式 flush。
import os
import sys
sys.stdout.flush()
os._exit(0 if state['done'] else 1)
