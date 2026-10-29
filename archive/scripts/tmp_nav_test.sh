#!/bin/bash
# tmp_nav_test.sh —— P65 独立验证: Nav2 绕 ob3 到达其后方目标(P63 冻结场景)
source /opt/ros/jazzy/setup.bash
cd /home/earl/workspace/VLA
echo "=== 1. 摆位: robot→(-4.2,-1.0) (P63 冻结点) ==="
timeout 10 gz service -s /world/follow_world/set_pose --reqtype gz.msgs.Pose \
  --reptype gz.msgs.Boolean --timeout 1000 \
  --payload 'name: "robot", position: {x: -4.2, y: -1.0, z: 0.03}, orientation: {w: 1.0}' 2>/dev/null
sleep 2
echo "=== 2. 激活 Nav2 通道 ==="
timeout 60 ros2 topic pub /nav2_active std_msgs/msg/Bool "{data: true}" --rate 5 >/dev/null 2>&1 &
ACT=$!
sleep 2
echo "=== 3. 下发 NavigateToPose 目标 (-3.8,-4.6) ==="
timeout 90 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -3.8, y: -4.6}}}}" --feedback 2>&1 | tail -6
kill $ACT 2>/dev/null
timeout 5 ros2 topic pub /nav2_active std_msgs/msg/Bool "{data: false}" --once >/dev/null 2>&1
echo "=== 4. 结果核验 ==="
python3 - <<'PYEOF'
import time, math
import gz.transport13 as gzt
from gz.msgs10.pose_v_pb2 import Pose_V
node = gzt.Node()
pos = []
def cb(m):
    for p in m.pose:
        if p.name == 'robot':
            pos.append((p.position.x, p.position.y))
node.subscribe(Pose_V, '/world/follow_world/dynamic_pose/info', cb)
t0 = time.time()
while time.time()-t0 < 3 and not pos: time.sleep(0.1)
if pos:
    x, y = pos[-1]
    d = math.hypot(x + 3.8, y + 4.6)
    print('机器人终位 (%.2f, %.2f), 距目标 %.2fm → %s' % (
        x, y, d, 'PASS(绕障成功)' if d < 0.9 else 'FAIL'))
PYEOF
