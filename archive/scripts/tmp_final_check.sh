#!/bin/bash
# 重启 demo, 检查 RTF / 节点 / 顶视订阅, 并截图
set +u
pkill -9 -f '[r]os2 launch vla_follow' 2>/dev/null
pkill -9 -f '[v]la_follow/lib' 2>/dev/null
pkill -9 -f '[p]arameter_bridge' 2>/dev/null
pkill -9 -f '[g]z sim' 2>/dev/null
pkill -9 -f '[g]z_viz' 2>/dev/null
sleep 5
echo "残留: $(pgrep -f '[g]z sim' | wc -l)"
cd /home/earl/workspace/VLA || exit 1
DISPLAY=:0 setsid nohup bash vla_demo/start_demo.sh > demo_final.log 2>&1 < /dev/null &
for i in $(seq 1 40); do
    sleep 2
    grep -q 'PERSON 行走' vla_ws/logs/demo.log 2>/dev/null && break
done
sleep 20
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source vla_ws/install/setup.bash >/dev/null 2>&1
echo "=== clock (RTF 参考: 应接近 30~50):"
timeout 8 ros2 topic hz /clock 2>/dev/null | head -3
echo "=== diag:"
timeout 8 ros2 topic echo --once /follower/diag 2>/dev/null | head -4
echo "=== overhead 订阅者:"
timeout 5 gz topic -i -t /overhead/image 2>/dev/null | grep -c tcp
echo "=== 进程: gz=$(pgrep -f '[g]z sim' | wc -l) viz=$(pgrep -f '[g]z_viz' | wc -l) fol=$(pgrep -f '[f]ollower' | wc -l)"
DISPLAY=:0 ffmpeg -y -loglevel error -f x11grab -i :0 -frames:v 1 /tmp/cam_final2.png
ls -l /tmp/cam_final2.png
echo DONE
