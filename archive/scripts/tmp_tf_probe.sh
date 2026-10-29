#!/bin/bash
# tmp_tf_probe.sh —— TF 数据源诊断
source /opt/ros/jazzy/setup.bash
python3 - <<'PYEOF'
import time
import gz.transport13 as gzt
from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.msgs10.pose_v_pb2 import Pose_V
node = gzt.Node()
stats, poses = [], []
def scb(m): stats.append(1)
def pcb(m): poses.append(len(m.pose))
node.subscribe(WorldStatistics, '/stats', scb)
node.subscribe(Pose_V, '/world/follow_world/dynamic_pose/info', pcb)
t0 = time.time()
while time.time()-t0 < 8: time.sleep(0.2)
print('/stats 帧: %d | dynamic_pose 帧: %d' % (len(stats), len(poses)))
if poses:
    print('最新一帧实体数: %d' % poses[-1])
PYEOF
echo '=== tf2_echo map→base_link ==='
timeout 8 ros2 run tf2_ros tf2_echo map base_link 2>&1 | grep -m1 -E 'Translation|does not exist'
echo '=== gt_tf 日志尾 ==='
grep 'gt_tf-8' /home/earl/workspace/VLA/vla_ws/logs/demo.log | tail -3 | cut -c1-130
