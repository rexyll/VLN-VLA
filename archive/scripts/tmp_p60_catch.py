#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P60: 跟随过程中抓"近场伪影"现场。
一旦 /perception/proximity 的 front < 0.6m, 立刻抓 depth_front 的
深度图与 RGB 各一张, 同时记录机器人位姿与 proximity 全部扇区。
"""
import json
import math
import subprocess
import sys
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import gz.transport13 as gt
from gz.msgs10.image_pb2 import Image as GzImage
from gz.msgs10.pose_v_pb2 import Pose_V

WORLD = 'follow_world'
OUT = sys.argv[1] if len(sys.argv) > 1 else '/tmp/p60'


def set_pose(name, x, y, z, yaw=0.0):
    req = ('name: "%s", position: {x: %s, y: %s, z: %s}, '
           'orientation: {z: %s, w: %s}'
           % (name, x, y, z, math.sin(yaw / 2), math.cos(yaw / 2)))
    for _ in range(3):
        r = subprocess.run(
            ['gz', 'service', '-s', '/world/%s/set_pose' % WORLD,
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '2000', '--req', req],
            capture_output=True, text=True)
        if 'true' in (r.stdout or '').lower():
            return True
        time.sleep(0.2)
    return False


def decode_img(m):
    h, w = m.height, m.width
    if m.pixel_format_type == 3:
        return np.frombuffer(m.data, dtype=np.uint8).reshape(h, w, 3)
    if m.pixel_format_type == 1:
        return np.frombuffer(m.data, dtype=np.uint8).reshape(h, w)
    return None


def decode_dep(m):
    h, w = m.height, m.width
    if m.pixel_format_type == 13:
        return np.frombuffer(m.data, dtype=np.float32).reshape(h, w)
    return None


def main():
    rclpy.init()
    node = Node('p60_catch')
    prox = {}
    stat = {}
    pose = {}

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

    gn = gt.Node()

    def pose_cb(m):
        for p in m.pose:
            if p.name in ('robot', 'person'):
                q = p.orientation
                pose[p.name] = (
                    p.position.x, p.position.y,
                    math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                            1 - 2 * (q.y * q.y + q.z * q.z))))

    gn.subscribe(Pose_V, '/world/%s/dynamic_pose/info' % WORLD, pose_cb)

    # 摆位形: 机器人 (-3,0) 朝 +x, 人 (1,0)
    subprocess.run(['pkill', '-f', 'person_wal[k]'], capture_output=True)
    time.sleep(0.6)
    set_pose('robot', -3.0, 0.0, 0.15, 0.0)
    set_pose('person', 1.0, 0.0, 0.02, math.pi)
    time.sleep(1.0)

    pub = node.create_publisher(String, '/robot/command', 10)
    cmd = String()
    cmd.data = json.dumps({'action': 'FOLLOW', 'distance': 1.0})
    for _ in range(3):
        pub.publish(cmd)
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.15)

    caught = {'rgb': None, 'dep': None}
    gn.subscribe(GzImage, '/depth_front/image', lambda m: caught.update(rgb=m))
    gn.subscribe(GzImage, '/depth_front/depth_image',
                 lambda m: caught.update(dep=m))

    t0 = time.time()
    hit = False
    while time.time() - t0 < 30 and not hit:
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.02)
        f = prox.get('front', 99)
        if 0 < f < 0.6:
            hit = True
            print('捕获 front=%.2f  全扇区=%s' % (f, prox))
            print('robot=%s person=%s' % (pose.get('robot'), pose.get('person')))
            print('status=%s' % {k: stat.get(k) for k in
                                 ('cmd_v', 'cmd_w', 'prim', 'cam', 'note')})
            time.sleep(0.6)
            break
    if not hit:
        print('30s 内未捕获近场, 末次 prox=%s' % prox)
        print('status=%s' % {k: stat.get(k) for k in
                             ('cmd_v', 'cmd_w', 'prim', 'cam', 'note')})
        return

    rgb = decode_img(caught.get('rgb')) if caught.get('rgb') else None
    dep = decode_dep(caught.get('dep')) if caught.get('dep') else None
    if rgb is not None:
        cv2.imwrite(OUT + '_rgb.png', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        print('saved rgb %dx%d' % (rgb.shape[1], rgb.shape[0]))
    if dep is not None:
        d = np.where(np.isfinite(dep) & (dep > 0.05) & (dep < 12), dep, np.nan)
        print('dep stats: min=%.2f p05=%.2f med=%.2f  有效=%d/%d'
              % (np.nanmin(d), np.nanpercentile(d, 5), np.nanmedian(d),
                 int(np.isfinite(d).sum()), d.size))
        # 沿仰角带 +1.5°~+18° 取, 复现 depth_percept 的行带
        h, w = dep.shape
        r0 = int(h * (0.5 - 18.0 / 83.58))
        r1 = int(h * (0.5 - 1.5 / 83.58)) + 1
        band = dep[r0:r1, :]
        ok = np.isfinite(band) & (band > 0.28) & (band < 11.5)
        print('band rows %d..%d  有效点=%d' % (r0, r1, int(ok.sum())))
        if ok.sum() > 20:
            for k in range(4):
                c0 = int(w * 0.12 + (w * 0.76) * k / 4)
                c1 = int(w * 0.12 + (w * 0.76) * (k + 1) / 4)
                v = band[:, c0:c1][ok[:, c0:c1]]
                print('  扇区%d 分位=%.2f (n=%d)'
                      % (k, float(np.quantile(v, 0.05)) if v.size >= 15 else -1,
                         v.size))
        m = np.nan_to_num(d, nan=12.0)
        m = np.clip(m / 4.0 * 255, 0, 255).astype(np.uint8)
        cv2.imwrite(OUT + '_dep.png', cv2.applyColorMap(m, cv2.COLORMAP_TURBO))
        print('saved dep')


if __name__ == '__main__':
    main()
