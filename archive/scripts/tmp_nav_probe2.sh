#!/bin/bash
# tmp_nav_probe2.sh —— 聚焦探针: 角速度+gz 侧消息计数+机器人位姿轨迹
source /opt/ros/jazzy/setup.bash
cd /home/earl/workspace/VLA
timeout 10 gz service -s /world/follow_world/set_pose --reqtype gz.msgs.Pose \
  --reptype gz.msgs.Boolean --timeout 1000 \
  --payload 'name: "robot", position: {x: -4.2, y: -1.0, z: 0.03}, orientation: {w: 1.0}' >/dev/null 2>&1
timeout 40 ros2 topic pub /nav2_active std_msgs/msg/Bool "{data: true}" --rate 5 >/dev/null 2>&1 &
ACT=$!
sleep 1.5
echo "=== 激活后 mux 日志 ==="
timeout 90 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -3.8, y: -4.6}}}}" >/dev/null 2>&1 &
GOAL=$!
sleep 5
echo "=== /nav2_cmd_vel 5s 采样(linear.x + angular.z) ==="
timeout 5 ros2 topic echo /nav2_cmd_vel geometry_msgs/msg/Twist 2>/dev/null | grep -E '^ *x:|^ *z:' | head -8
echo "=== /robot/cmd_vel 5s 采样 ==="
timeout 5 ros2 topic echo /robot/cmd_vel geometry_msgs/msg/Twist 2>/dev/null | grep -E '^ *x:|^ *z:' | head -8
echo "=== gz 侧消息计数(3s) ==="
timeout 3 gz topic -e /model/robot/cmd_vel -n 500 2>/dev/null | grep -c 'linear'
echo "=== 机器人位姿轨迹(8s) ==="
python3 - <<'PYEOF'
import time, math
import gz.transport13 as gzt
from gz.msgs10.pose_v_pb2 import Pose_V
node = gzt.Node()
pos = []
def cb(m):
    for p in m.pose:
        if p.name == 'robot':
            q = p.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.x*q.x))
            pos.append((time.time(), p.position.x, p.position.y, math.degrees(yaw)))
node.subscribe(Pose_V, '/world/follow_world/dynamic_pose/info', cb)
t0 = time.time()
while time.time()-t0 < 8: time.sleep(0.2)
if pos:
    for p in pos[::10]:
        print('  (%.2f, %.2f) yaw=%.0f°' % (p[1], p[2], p[3]))
    a, b = pos[0], pos[-1]
    print('位移 %.2fm, Δyaw %.0f°' % (math.hypot(b[1]-a[1], b[2]-a[2]), b[3]-a[3]))
PYEOF
kill $ACT $GOAL 2>/dev/null
wait 2>/dev/null
echo "=== 期间 mux/CMD 日志 ==="
grep -E 'CMD_MUX' /home/earl/workspace/VLA/vla_ws/logs/demo.log | tail -4 | cut -c1-120
