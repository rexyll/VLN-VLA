#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""检查 gz 话题是否真有数据: python3 gz_probe.py <topic> [秒]"""
import sys
import time

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

topic = sys.argv[1] if len(sys.argv) > 1 else '/overhead/image'
dur = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0

n = Node()
cnt = [0]
meta = {}


def cb(msg):
    cnt[0] += 1
    if cnt[0] == 1:
        meta['w'] = msg.width
        meta['h'] = msg.height
        meta['enc'] = msg.pixel_format_type
        meta['data'] = len(msg.data)
        meta['stamp'] = (msg.header.stamp.sec, msg.header.stamp.nsec)


ok = n.subscribe(Image, topic, cb)
t0 = time.time()
while time.time() - t0 < dur:
    time.sleep(0.1)
n.unsubscribe(topic)
print('PROBE', topic, 'subscribed=%s' % ok, 'msgs=%d in %.0fs' % (cnt[0], dur), meta)
