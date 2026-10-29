#!/bin/bash
# tmp_m1_pts.sh —— 点云坐标系标定: 各相机点云的轴范围 vs 真值
source /opt/ros/jazzy/setup.bash
python3 - <<'PYEOF'
import time, math
import numpy as np
import gz.transport13 as gzt
from gz.msgs10.pointcloud_packed_pb2 import PointCloudPacked
from gz.msgs10.pose_v_pb2 import Pose_V

node = gzt.Node()
clouds = {n: [] for n in ('front','left','right','back')}
def mk(nm):
    def cb(m):
        if not clouds[nm]: clouds[nm].append(m)
    return cb
for n in clouds:
    node.subscribe(PointCloudPacked, '/depth_' + n + '/points', mk(n))
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
while time.time()-t0 < 12 and not (all(clouds.values()) and pose):
    time.sleep(0.1)
rx, ry, yaw = pose[0] if pose else (0,0,0)
print('robot (%.2f,%.2f) yaw=%.0f°' % (rx, ry, math.degrees(yaw)))
for name in ('front','left','right','back'):
    if not clouds[name]:
        print(name, ': 无点云'); continue
    m = clouds[name][0]
    flds = [(f.name, f.offset) for f in m.field]
    step = m.point_step if m.point_step >= 12 else 12
    npt = len(m.data) // step
    raw = np.frombuffer(m.data[:npt*step], dtype=np.uint8).reshape(npt, step)
    a = raw[:, :12].copy().view('<f4').reshape(npt, 3)
    fin = np.isfinite(a).all(axis=1) & (np.abs(a) < 30).all(axis=1)
    a = a[fin]
    if a.size == 0:
        print(name, ': 无有限点'); continue
    rng = [(float(np.min(a[:,i])), float(np.median(a[:,i])), float(np.max(a[:,i]))) for i in range(3)]
    print('%s: fields=%s step=%d 点数=%d(有效%d)' % (name, flds, step, npt, len(a)))
    for ax, (mn, md, mx) in zip('xyz', rng):
        print('   %s: min=%.2f 中位=%.2f max=%.2f' % (ax, mn, md, mx))
PYEOF
