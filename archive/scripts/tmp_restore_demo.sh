#!/bin/bash
# 恢复默认引擎并把 demo 重新拉起来（普通随机行走）
set +u
pkill -9 -f '[r]os2 launch vla_follow' 2>/dev/null
pkill -9 -f '[v]la_follow/lib' 2>/dev/null
pkill -9 -f '[p]arameter_bridge' 2>/dev/null
pkill -9 -f '[g]z sim' 2>/dev/null
pkill -9 -f '[g]z_viz' 2>/dev/null
sleep 5
echo "残留 gz: $(pgrep -f '[g]z sim' | wc -l)"
cd /home/earl/workspace/VLA || exit 1
DISPLAY=:0 setsid nohup bash vla_demo/start_demo.sh > demo_restore.log 2>&1 < /dev/null &
for i in $(seq 1 40); do
    sleep 2
    grep -q 'PERSON 行走' vla_ws/logs/demo.log 2>/dev/null && break
done
sleep 6
echo "--- gui.config:"; grep engine ~/.gz/sim/8/gui.config | head -2; grep camera_pose ~/.gz/sim/8/gui.config
echo "--- 进程:"; pgrep -f '[g]z sim' | wc -l; pgrep -f '[g]z_viz' | wc -l; pgrep -f '[f]ollower' | wc -l
echo "--- 话题:"; timeout 6 ros2 topic hz /clock 2>/dev/null | head -3
echo "--- diag:"; timeout 6 ros2 topic echo --once /follower/diag 2>/dev/null | head -3
echo DONE
