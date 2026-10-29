#!/bin/bash
# tmp_nav_probe.sh —— Nav2 速度链路逐级探针
source /opt/ros/jazzy/setup.bash
cd /home/earl/workspace/VLA
timeout 10 gz service -s /world/follow_world/set_pose --reqtype gz.msgs.Pose \
  --reptype gz.msgs.Boolean --timeout 1000 \
  --payload 'name: "robot", position: {x: -4.2, y: -1.0, z: 0.03}, orientation: {w: 1.0}' >/dev/null 2>&1
timeout 25 ros2 topic pub /nav2_active std_msgs/msg/Bool "{data: true}" --rate 5 >/dev/null 2>&1 &
ACT=$!
sleep 2
timeout 90 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -3.8, y: -4.6}}}}" >/dev/null 2>&1 &
GOAL=$!
sleep 6
echo "=== 各级速度采样(6s) ==="
for t in /cmd_vel_nav /nav2_cmd_vel /robot/cmd_vel; do
  echo -n "$t: "
  timeout 4 ros2 topic echo $t geometry_msgs/msg/Twist --field linear 2>/dev/null | head -2 | tr '\n' ' '
  echo ""
done
echo "=== gz 侧 /model/robot/cmd_vel ==="
timeout 4 gz topic -e /model/robot/cmd_vel -n 2 2>/dev/null | grep -A1 linear | head -4
kill $ACT $GOAL 2>/dev/null
wait 2>/dev/null
