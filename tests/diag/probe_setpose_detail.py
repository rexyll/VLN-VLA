#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""probe_setpose_detail.py —— set_pose 的 z 分量为何不生效

probe_follow_sag 显示: x/y 精确跟随, z 完全不响应(目标 0.90, 读回一路下坠
到 0.80)。本探针把请求/应答都摊开:
  [1] 打印 protobuf 里 Position 的实际字段与序列化字节
  [2] 打印服务返回的 Boolean 值(False = 服务端根本没找到实体/拒绝)
  [3] 纯 z 抬升 5cm, 立刻读回(0.15s 内, 排除自由落体)
  [4] 用 CLI `gz service -s ... --reqtype gz.msgs.Pose` 做旁证

用法: python3 tests/diag/probe_setpose_detail.py
"""
import subprocess
import sys
import time

import gz.transport13 as gz_transport
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.boolean_pb2 import Boolean

WORLD = 'follow_world'
TOPIC = '/world/%s/dynamic_pose/info' % WORLD
SVC = '/world/%s/set_pose' % WORLD
MODEL = 'kettle'

link = {}


def cb(msg):
    for p in msg.pose:
        link[p.name] = (p.position.x, p.position.y, p.position.z)


def main():
    node = gz_transport.Node()
    node.subscribe(Pose_V, TOPIC, cb)
    time.sleep(2.0)
    p0 = link[MODEL]
    print('起点 %s' % (p0,))

    m = Pose()
    m.name = MODEL
    m.orientation.w = 1.0
    m.position.x, m.position.y, m.position.z = p0[0], p0[1], p0[2] + 0.05
    print('[1] Pose 字段: name=%r  position=(%.4f %.4f %.4f)  bytes=%d'
          % (m.name, m.position.x, m.position.y, m.position.z,
             len(m.SerializeToString())))

    # 慢速: 用阻塞 request, 无 sleep
    # 注意: gz.transport13 的 Python 绑定返回的是 (bool, response) 元组,
    # 不是单独的 response —— 之前 `if ok is None` 的判断永远为假, 是个假阳性。
    for i in range(6):
        z = p0[2] + 0.05
        m.position.z = z
        t = time.time()
        ret = node.request(SVC, m, Pose, Boolean, 1000)
        dt = (time.time() - t) * 1000.0
        if isinstance(ret, tuple):
            okflag, resp = ret
            rdata = getattr(resp, 'data', 'n/a')
        else:
            okflag, rdata = 'n/a', getattr(ret, 'data', 'n/a')
        print('[2] call%d  ok=%s data=%s (%.2fms)  read_z=%.4f  target %.4f'
              % (i, okflag, rdata, dt, link[MODEL][2], z))
        time.sleep(0.30)          # 让物理下落 0.3s 便于对比

    # [4] CLI 旁证
    req = ('name: "%s", position: {x: %.4f, y: %.4f, z: %.4f}, '
           'orientation: {w: 1.0}' % (MODEL, p0[0], p0[1], p0[2]))
    cmd = ['gz', 'service', '-s', SVC, '--reqtype', 'gz.msgs.Pose',
           '--reptype', 'gz.msgs.Boolean', '--timeout', '1000',
           '--req', req]
    print('[4] CLI: %s' % ' '.join(cmd))
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=20)
        print('    rc=%d out=%r err=%r'
              % (out.returncode, out.stdout.strip()[:200],
                 out.stderr.strip()[:200]))
    except Exception as e:                             # pragma: no cover
        print('    CLI 失败: %s' % e)
    time.sleep(0.4)
    print('    CLI 之后 read_z=%.4f (target %.4f)' % (link[MODEL][2], p0[2]))

    # 复原
    m.position.x, m.position.y, m.position.z = p0
    node.request(SVC, m, Pose, Boolean, 1000)
    time.sleep(1.0)
    print('还原 z=%.4f' % link[MODEL][2])
    return 0


if __name__ == '__main__':
    sys.exit(main())
