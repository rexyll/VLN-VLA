#!/bin/bash
# tmp_p66_verify.sh —— T6 验证: person 行走过程中 torso/head 是否保持连接
source /opt/ros/jazzy/setup.bash
python3 - <<'PYEOF'
import time, math
import gz.transport13 as gzt
from gz.msgs10.pose_v_pb2 import Pose_V

node = gzt.Node()
P = {}   # name -> (t, x, y, z)
lock = __import__('threading').Lock()
def cb(m):
    with lock:
        for p in m.pose:
            P[p.name] = (time.time(), p.position.x, p.position.y, p.position.z)
node.subscribe(Pose_V, '/world/follow_world/dynamic_pose/info', cb)
t0 = time.time()
print('采样 30s(人在行走)...')
time.sleep(30)
with lock:
    snap = dict(P)
need = ('skirt', 'torso', 'head')
if not all(n in snap for n in need):
    print('缺少部件:', [n for n in need if n not in snap]); raise SystemExit
# 首末对比
def off(a, b):
    return (math.hypot(snap[a][1]-snap[b][1], snap[a][2]-snap[b][2]),
            snap[a][3]-snap[b][3])
print('=== 末时刻相对偏移 ===')
for part in ('torso', 'head'):
    dxy, dz = off(part, 'skirt')
    print('%s: 距skirt水平偏移=%.4fm 垂直差=%+.3fm (z=%.3f)' % (
        part, dxy, dz, snap[part][3]))
print('skirt 末位置: (%.2f, %.2f) — 人在移动中' % (snap['skirt'][1], snap['skirt'][2]))
# 连续监测: 再采 20s, 检查水平偏移全程 < 0.05m
bad = 0; n = 0
t1 = time.time()
while time.time() - t1 < 20:
    with lock:
        s = dict(P)
    if all(k in s for k in need):
        n += 1
        for part in ('torso', 'head'):
            d = math.hypot(s[part][1]-s['skirt'][1], s[part][2]-s['skirt'][2])
            if d > 0.05 or s[part][3] < 0.5:
                bad += 1
    time.sleep(0.2)
print('连续监测 20s: 样本 %d, 掉落/脱连样本 %d → %s' % (
    n, bad, 'PASS 部件无掉落' if bad == 0 else 'FAIL'))
PYEOF
