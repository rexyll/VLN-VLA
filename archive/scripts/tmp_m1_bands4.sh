#!/bin/bash
# tmp_m1_bands4.sh —— 四相机分行带有效深度统计(判定各自地平线位置)
source /opt/ros/jazzy/setup.bash
python3 - <<'PYEOF'
import time
import numpy as np
import gz.transport13 as gzt
from gz.msgs10.image_pb2 import Image
from gz.msgs10.pose_v_pb2 import Pose_V
import math

node = gzt.Node()
cams = {n: [] for n in ('front','left','right','back')}
def mk(nm):
    def cb(m):
        if not cams[nm]: cams[nm].append(m)
    return cb
for n in cams:
    node.subscribe(Image, '/depth_' + n, mk(n))
pose = []
def pcb(m):
    if not pose:
        for p in m.pose:
            if p.name == 'robot':
                q = p.orientation
                pose.append((p.position.x, p.position.y,
                             math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.x*q.x))))
node.subscribe(Pose_V, '/world/follow_world/dynamic_pose/info', pcb)
t0 = time.time()
while time.time()-t0 < 10 and not (all(cams.values()) and pose):
    time.sleep(0.1)
rx, ry, yaw = pose[0] if pose else (0,0,0)
print('robot (%.2f,%.2f) yaw=%.0f°' % (rx, ry, math.degrees(yaw)))
WALLS = []
for name in ('front','left','right','back'):
    if not cams[name]:
        print(name, ': 无帧'); continue
    m = cams[name][0]
    a = np.frombuffer(m.data[:m.width*m.height*4], dtype='<f4').reshape(m.height, m.width)
    fin = np.isfinite(a) & (a > 0.05) & (a < 11.5)
    h = m.height
    parts = []
    for i in range(10):
        r0, r1 = int(h*i/10), int(h*(i+1)/10)
        cnt = int(fin[r0:r1, :].sum())
        if cnt > 50:
            v = a[r0:r1, :][fin[r0:r1, :]]
            parts.append('%d0%%:%dm(中位%.1f)' % (i, cnt, float(np.median(v))))
        else:
            parts.append('%d0%%:-' % i)
    print('%s: %s' % (name, ' '.join(parts)))
PYEOF
