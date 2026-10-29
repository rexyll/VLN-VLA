#!/bin/bash
# tmp_m1_restart.sh —— M1 排障: 重启并密切观察 gz 服务端存活与深度话题
source /opt/ros/jazzy/setup.bash
bash ~/vla_demo/stop_demo.sh
sleep 1
bash ~/vla_demo/start_demo.sh auto_voice:=false
echo "=== T+10s 进程与话题 ==="
sleep 10
pgrep -af 'gz sim' | grep -v pgrep
timeout 8 gz topic -l 2>/dev/null | grep -i depth
echo "=== T+20s 再查一次 ==="
sleep 10
pgrep -af 'gz sim' | grep -v pgrep
timeout 8 gz topic -l 2>/dev/null | grep -i depth
echo "=== gz 段日志尾部 ==="
grep -n '\[gz-1\]' ~/vla_ws/logs/demo.log | tail -6 | cut -c1-180
