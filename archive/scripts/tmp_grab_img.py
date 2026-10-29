#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一次性抓一帧图像话题存 PNG:  python3 grab_img.py <topic> <out.png> [timeout_s]"""
import sys, os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    import numpy as np
    from PIL import Image as PILImage
except Exception:
    np = None


class Grab(Node):
    def __init__(self, topic, out):
        super().__init__('grab_img')
        self.out = out
        self.got = False
        self.info = ''
        self.create_subscription(Image, topic, self.cb, 5)

    def cb(self, msg):
        if self.got:
            return
        self.got = True
        self.info = 'encoding=%s %dx%d step=%d' % (
            msg.encoding, msg.width, msg.height, msg.step)
        if np is None:
            return
        enc = msg.encoding.lower()
        nch = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'mono8': 1}.get(enc, 3)
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        try:
            arr = arr.reshape(msg.height, msg.step // nch if nch > 1 else msg.step)
            arr = arr[:, :msg.width * nch if nch > 1 else msg.width]
            arr = arr.reshape(msg.height, msg.width, nch) if nch > 1 else arr
        except Exception as ex:
            self.info += ' reshape失败:%r' % ex
            return
        if nch == 1:
            img = arr
        elif enc.startswith('bgr'):
            img = arr[:, :, ::-1]
        else:
            img = arr[:, :, :3]
        PILImage.fromarray(img).save(self.out)
        self.info += ' -> %s' % self.out


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else '/overhead/image'
    out = sys.argv[2] if len(sys.argv) > 2 else '/tmp/grab.png'
    tmo = float(sys.argv[3]) if len(sys.argv) > 3 else 15.0
    rclpy.init()
    n = Grab(topic, out)
    import time
    t0 = time.time()
    while rclpy.ok() and not n.got and time.time() - t0 < tmo:
        rclpy.spin_once(n, timeout_sec=0.3)
    print('GRAB', topic, 'got=%s' % n.got, n.info,
          'size=%s' % (os.path.getsize(out) if os.path.exists(out) else 0))
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
