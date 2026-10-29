#!/bin/bash
# tmp_calib_run.sh —— 每次测试前把机器人放回空地, 避开"顶墙"造成的假低速
set +u
source /opt/ros/jazzy/setup.bash
source /home/earl/workspace/VLA/vla_ws/install/setup.bash
cd /home/earl/workspace/VLA || exit 1

reset_robot() {
  gz service -s /world/follow_world/set_pose --reqtype gz.msgs.Pose \
    --reptype gz.msgs.Boolean --timeout 3000 \
    --req 'name: "robot", position: {x: -4.0, y: -4.0, z: 0.15}, orientation: {w: 1}' \
    >/dev/null 2>&1
  sleep 1.0
}

echo "=== 1) 直行 v=0.30 ==="
reset_robot; python3 tmp_calib2.py 0.3 0.0 4.0

echo "=== 2) 直行 v=0.60 ==="
reset_robot; python3 tmp_calib2.py 0.6 0.0 3.0

echo "=== 3) 原地旋转 w=1.00 ==="
reset_robot; python3 tmp_calib2.py 0.0 1.0 4.0

echo "=== 4) 原地旋转 w=2.00 ==="
reset_robot; python3 tmp_calib2.py 0.0 2.0 3.0

echo "=== 5) 原地旋转 w=0.40 ==="
reset_robot; python3 tmp_calib2.py 0.0 0.4 4.0

reset_robot
echo "=== 标定结束 ==="
