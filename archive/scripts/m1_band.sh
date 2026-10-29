#!/bin/bash
# tmp_m1_band.sh —— 分行带深度统计 + 真值位姿, 判定图像方向
source /opt/ros/jazzy/setup.bash
python3 - <<'PYEOF'
import time, math
import numpy as np
import gz.transport13 as gzt
from gz.msgs10.image_pb2 import Image
from gz.msgs10.pose_v_pb2 import Pose_V

node = gzt.Node()
frames, poses = [], []
def icb(m):
    if len(frames) < 3: frames.append(m)
node.subscribe(Image, '/depth_front', icb)
def pcb(m):
    if not poses:
        for p in m.pose:
            if p.name == 'robot':
                q = p.orientation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.x*q.x))
                poses.append((p.position.x, p.position.y, yaw))
node.subscribe(Pose_V, '/world/follow_world/dynamic_pose/info', pcb)
t0 = time.time()
while time.time()-t0 < 8 and (len(frames) < 3 or not poses):
    time.sleep(0.1)
assert frames and poses, 'no data'
m = frames[-1]
a = np.frombuffer(m.data[:m.width*m.height*4], dtype='<f4').reshape(m.height, m.width)
rx, ry, ryaw = poses[0]
print('robot pose x=%.2f y=%.2f yaw=%.0fdeg' % (rx, ry, math.degrees(ryaw)))
# 前向真值(仅墙): 到 x=±6 / y=±6 墙的直线距离
c, s = math.cos(ryaw), math.sin(ryaw)
best = 12.0
if c > 1e-6: best = min(best, (6.0-rx)/c)
if c < -1e-6: best = min(best, (-6.0-rx)/c)
if s > 1e-6: best = min(best, (6.0-ry)/s)
if s < -1e-6: best = min(best, (-6.0-ry)/s)
print('前向墙距(真值, 忽略障碍)=%.1fm' % best)
h = m.height
for name, r0, r1 in [('顶部0-20%', 0, int(h*0.2)), ('中部40-60%', int(h*0.4), int(h*0.6)), ('底部80-100%', int(h*0.8), h)]:
    band = a[r0:r1, :].ravel()
    valid = band[np.isfinite(band) & (band > 0.05) & (band < 11.5)]
    if valid.size:
        print('%s: 有效%d/%d 中位=%.2f 5%%分位=%.2f 95%%分位=%.2f' % (
            name, valid.size, band.size, float(np.median(valid)),
            float(np.quantile(valid, 0.05)), float(np.quantile(valid, 0.95))))
    else:
        print('%s: 无有效值(全inf?)' % name)
PYEOF
