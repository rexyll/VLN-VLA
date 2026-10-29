#!/bin/bash
# tmp_m1_yaw.sh —— 验证机器人是否真的在旋转 + 手动旋转指令测试
source /opt/ros/jazzy/setup.bash
python3 - <<'PYEOF'
import time, math
import gz.transport13 as gzt
from gz.msgs10.pose_v_pb2 import Pose_V
node = gzt.Node()
poses = []
def cb(m):
    for p in m.pose:
        if p.name == 'robot':
            q = p.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.x*q.x))
            poses.append((p.position.x, p.position.y, yaw))
node.subscribe(Pose_V, '/world/follow_world/dynamic_pose/info', cb)
t0 = time.time()
while time.time()-t0 < 4 and len(poses) < 200:
    time.sleep(0.05)
if poses:
    a, b = poses[0], poses[-1]
    dyaw = math.degrees(b[2]-a[2])
    while dyaw > 180: dyaw -= 360
    while dyaw < -180: dyaw += 360
    print('robot(前): x=%.2f y=%.2f yaw=%.0f°' % (a[0], a[1], math.degrees(a[2])))
    print('robot(后): x=%.2f y=%.2f yaw=%.0f°' % (b[0], b[1], math.degrees(b[2])))
    print('4秒内 yaw 变化: %+.1f°, 位移 %.2fm' % (dyaw, math.hypot(b[0]-a[0], b[1]-a[1])))
PYEOF
echo "=== 手动旋转指令测试(3秒 w=1.5) ==="
timeout 4 ros2 topic pub /robot/cmd_vel geometry_msgs/msg/Twist '{angular: {z: 1.5}}' --rate 10 >/dev/null 2>&1
python3 - <<'PYEOF'
import time, math
import gz.transport13 as gzt
from gz.msgs10.pose_v_pb2 import Pose_V
node = gzt.Node()
poses = []
def cb(m):
    for p in m.pose:
        if p.name == 'robot':
            q = p.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.x*q.x))
            poses.append((p.position.x, p.position.y, yaw))
node.subscribe(Pose_V, '/world/follow_world/dynamic_pose/info', cb)
t0 = time.time()
while time.time()-t0 < 4 and len(poses) < 200:
    time.sleep(0.05)
if poses:
    a, b = poses[0], poses[-1]
    dyaw = math.degrees(b[2]-a[2])
    while dyaw > 180: dyaw -= 360
    while dyaw < -180: dyaw += 360
    print('手动指令后 4秒 yaw 变化: %+.1f°, 位移 %.2fm' % (
        dyaw, math.hypot(b[0]-a[0], b[1]-a[1])))
PYEOF
