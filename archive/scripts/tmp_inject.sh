#!/bin/bash
# tmp_inject.sh —— 视觉管道注入测试: 用 python 发布干净的假人检测, 验证 follower 切换[视觉]
source /opt/ros/jazzy/setup.bash
python3 - <<'PYEOF'
import rclpy, json, time
from std_msgs.msg import String
rclpy.init()
n = rclpy.create_node('inject')
pub = n.create_publisher(String, '/perception/person', 10)
t0 = time.time()
while time.time() - t0 < 4.0:
    m = String()
    m.data = json.dumps({'detected': True, 'cam': 'front', 'dist': 3.0,
                         'bearing': 0.0, 'bearing_cam': 0.0, 'u': 160,
                         'area': 800, 'age': 0.0})
    pub.publish(m)
    time.sleep(0.1)
print('injected 4s fake person(front,dist=3.0)')
rclpy.shutdown()
PYEOF
