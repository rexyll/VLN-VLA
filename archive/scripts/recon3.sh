#!/bin/bash
echo ===ROS===
ls /opt/ros/ 2>/dev/null
source /opt/ros/jazzy/setup.bash 2>/dev/null
echo "ROS_DISTRO=$ROS_DISTRO"
which ros2 colcon
ros2 --version 2>/dev/null | head -1
echo ===ROS_GZ===
ros2 pkg list 2>/dev/null | grep -iE "ros_gz|gz_sim|gz_bridge" | head -10
echo ===RCLPY===
python3 -c "import rclpy; print('rclpy OK')" 2>&1 | tail -1
echo ===BASHRC===
grep -n "ros\|GZ" ~/.bashrc | head -8
echo ===WORKSPACES===
ls ~/vla_ws ~/ros2_ws 2>/dev/null
echo ===DONE===
