#!/usr/bin/env bash
# vm_collect.sh —— 安全启动采集（避免 pkill 模式自匹配）
cd /home/earl/vla/code/test
pkill -f "collect_gazebo_data" 2>/dev/null || true
sleep 1
nohup python3 -u collect_gazebo_data.py --n 40 --out gz_data.npz --max-minutes 22 > collect.log 2>&1 &
echo "LAUNCHED pid $!"
