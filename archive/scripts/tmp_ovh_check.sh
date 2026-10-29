#!/bin/bash
# 重启 demo 并验证 overhead 相机 + 顶视图窗口
set +u
pkill -9 -f '[r]os2 launch vla_follow' 2>/dev/null
pkill -9 -f '[v]la_follow/lib' 2>/dev/null
pkill -9 -f '[p]arameter_bridge' 2>/dev/null
pkill -9 -f '[g]z sim' 2>/dev/null
pkill -9 -f '[g]z_viz' 2>/dev/null
sleep 5
echo "残留: $(pgrep -f '[g]z sim' | wc -l)"
cd /home/earl/workspace/VLA || exit 1
DISPLAY=:0 setsid nohup bash vla_demo/start_demo.sh > demo_ovh.log 2>&1 < /dev/null &
for i in $(seq 1 40); do
    sleep 2
    grep -q 'PERSON 行走' vla_ws/logs/demo.log 2>/dev/null && break
done
sleep 8
echo "--- gui fix 输出:"; grep -i 'GUI-FIX\|overhead' demo_ovh.log | head -3
echo "--- config 里是否有 Overhead:"; grep -c 'Overhead View' ~/.gz/sim/8/gui.config
echo "--- overhead 话题:"; timeout 5 gz topic -l 2>/dev/null | grep -i overhead
echo "--- 图像信息:"; timeout 5 gz topic -i -t /overhead/image 2>/dev/null | head -6
DISPLAY=:0 ffmpeg -y -loglevel error -f x11grab -i :0 -frames:v 1 /tmp/cam_ovh.png
ls -l /tmp/cam_ovh.png
echo DONE
